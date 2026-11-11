"""Controlled protection ablation of the preserved LNS random + PP adapter.

Only the per-flight delay bound changes between free, initial and monotone.
This module leaves all step-3 solver source and records unchanged.
"""
import random
import time
import final_model as fm
import lns_resource_baseline as base
from indexed_scheduler import IntervalIndex, schedule_flights, ref


def selection(s):
    return {'id':s.flight_id, 'priority':s.priority, 'delay_sec':s.delay_sec,
            'finish_sec':s.finish_sec, 'route_index':s.route_index,
            'layers':list(s.layer_plan), 'lanes':list(s.lane_plan),
            'speed_mode':s.speed_mode, 'structural_cost':s.structural_cost,
            'airborne_sec':s.finish_sec-s.scheduled_start_sec}


def solve(fs,net,types,*,candidate_builder,mode='initial',seed=0,
          improvement_limit=10.,neighborhood_size=8,iteration_limit=100000):
    if mode not in ('free','initial','monotone'):
        raise ValueError(mode)
    started=time.perf_counter()
    rng=random.Random(seed)
    templates=base.make_templates(fs,net,types,candidate_builder)
    template_time=time.perf_counter()-started
    ids=sorted(templates)
    policy,_=fm.policy('SPC')
    schedules=schedule_flights(fs,net,types,policy,candidate_builder=candidate_builder).schedules
    index=IntervalIndex()
    for s in schedules.values():
        index.reserve(s)
    initial=dict(schedules)
    caps={fid:s.delay_sec for fid,s in initial.items()}
    complete=len(schedules)==len(fs)
    improvement_start=time.perf_counter()
    deadline=improvement_start+improvement_limit
    initial_cost=base.weighted(schedules)
    objective=initial_cost
    trace=[{'iteration':0,'improvement_sec':0.,'weighted_delay_sec':objective}]
    iterations=completed_iterations=0
    updates=[]
    status='INITIALIZATION_FAILED' if not complete else 'ITERATION_LIMIT'
    while complete and iterations<iteration_limit:
        if time.perf_counter()>=deadline:
            status='TIME_LIMIT'; break
        if objective==0:
            status='ZERO_OBJECTIVE'; break
        iterations+=1
        selected=rng.sample(ids,min(neighborhood_size,len(ids)))
        old={fid:schedules[fid] for fid in selected}
        old_cost=base.weighted(old)
        for s in old.values():
            index.release(s)
        replacement={}
        remaining=old_cost
        expired=False
        try:
            for fid in selected:
                limit=remaining/fm.WEIGHTS[old[fid].priority]
                if mode=='initial':
                    limit=min(limit,caps[fid])
                elif mode=='monotone':
                    limit=min(limit,old[fid].delay_sec)
                response=base.best_response(templates[fid],index,net,
                                           max_delay=limit,deadline=deadline)
                if response is None:
                    break
                replacement[fid]=response
                remaining-=fm.WEIGHTS[old[fid].priority]*response.delay_sec
                index.reserve(response)
        except base.BudgetExpired:
            expired=True
        if not expired and len(replacement)==len(selected) and remaining>ref.EPS:
            objective-=remaining
            schedules.update(replacement)
            updates.append({'iteration':iterations,'ids':selected,
                'before_weighted_sec':old_cost,'after_weighted_sec':base.weighted(replacement),
                'individual_delays_before':{fid:s.delay_sec for fid,s in old.items()},
                'individual_delays_after':{fid:s.delay_sec for fid,s in replacement.items()},
                'replacements':[selection(replacement[fid]) for fid in selected]})
            # The timestamp includes the recorded accepted replacement, for anytime use.
            trace.append({'iteration':iterations,'improvement_sec':time.perf_counter()-improvement_start,
                          'weighted_delay_sec':objective})
        else:
            for s in replacement.values():
                index.release(s)
            for s in old.values():
                index.reserve(s)
        if expired:
            status='TIME_LIMIT'; break
        completed_iterations=iterations
    finished=time.perf_counter()
    metadata={'status':status,'mode':mode,'initialization':'SPC','rng_seed':seed,
        'neighborhood_size':neighborhood_size,'improvement_limit_sec':improvement_limit,
        'iteration_limit':iteration_limit,'iterations':iterations,
        'completed_iterations':completed_iterations,'accepted_moves':len(updates),
        'template_build_sec':template_time,'initialization_sec':improvement_start-started-template_time,
        'improvement_sec':finished-improvement_start,'total_time_sec':finished-started,
        'initial_weighted_delay_sec':initial_cost,'weighted_delay_sec':objective,
        'initial_signature':fm.signature(initial) if complete else None,
        'individual_initial_caps_sec':caps,'trace':trace,'updates':updates,'proved_optimal':False,
        'variant':'Controlled individual-delay bounds on MAPF-LNS random + PP resource adaptation'}
    return schedules,metadata,initial
