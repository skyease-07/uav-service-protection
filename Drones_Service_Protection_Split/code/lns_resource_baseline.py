"""MAPF-LNS random-neighborhood/PP variant, Li et al. IJCAI 2021 Sections 4,5.3,7.

Shared resources replace vertex/swap conflicts; objective is weighted ground
delay. This is the published random-neighborhood variant, not full ALNS.
"""
import math
import random
import time
from indexed_scheduler import IntervalIndex, resource_intervals, shifted, schedule_flights, ref
import final_model as fm


class BudgetExpired(Exception):
    pass


def make_templates(fs,net,types,builder):
    result={}
    for f in fs:
        choices=[]
        for choice in ref.enumerate_structural_choices(f,net):
            base=builder(f,choice,0,net,types)
            choices.append((base,resource_intervals(base.resources)))
        result[f.flight_id]=choices
    return result


def best_response(choices,index,net,max_delay=None,deadline=math.inf):
    limit=net.max_delay_sec if max_delay is None else min(net.max_delay_sec,max_delay)
    max_step=int(math.floor((limit+ref.EPS)/net.time_slot_sec))
    if max_step<0:
        return None
    best=best_key=None
    for base,intervals in choices:
        if time.perf_counter()>=deadline:
            raise BudgetExpired
        step=index.earliest(intervals,max_step)
        if step is None:
            continue
        key=(step,base.structural_cost,base.finish_sec,base.route_index,
             base.layer_plan,base.lane_plan,base.speed_mode)
        if best_key is None or key<best_key:
            best,best_key=base,key
            max_step=step
    return shifted(best,best_key[0],net,ref.SchedulingPolicy(delay_weights=fm.WEIGHTS)) if best else None


def weighted(schedules):
    return sum(fm.WEIGHTS[s.priority]*s.delay_sec for s in schedules.values())


def solve(fs,net,types,*,candidate_builder,initialization='SPC',seed=0,
          improvement_limit=10.,neighborhood_size=8,iteration_limit=100000):
    started=time.perf_counter()
    rng=random.Random(seed)
    templates=make_templates(fs,net,types,candidate_builder)
    template_time=time.perf_counter()-started
    ids=sorted(templates)
    index=IntervalIndex()
    if initialization=='SPC':
        policy,_=fm.policy('SPC')
        schedules=schedule_flights(fs,net,types,policy,candidate_builder=candidate_builder).schedules
        for s in schedules.values():
            index.reserve(s)
    elif initialization=='random-PP':
        order=list(ids); rng.shuffle(order)
        schedules={}
        for fid in order:
            response=best_response(templates[fid],index,net)
            if response is not None:
                schedules[fid]=response
                index.reserve(response)
    else:
        raise ValueError(initialization)
    initial_schedules=dict(schedules)
    complete=len(schedules)==len(fs)
    improvement_start=time.perf_counter()
    deadline=improvement_start+improvement_limit
    initial_cost=weighted(schedules)
    objective=initial_cost
    trace=[{'improvement_sec':0.,'weighted_delay_sec':objective}]
    iterations=accepted=0
    accepted_moves=[]
    status='INITIALIZATION_FAILED' if not complete else 'ITERATION_LIMIT'
    while complete and iterations<iteration_limit:
        if time.perf_counter()>=deadline:
            status='TIME_LIMIT'; break
        if objective==0:
            status='ZERO_OBJECTIVE'; break
        iterations+=1
        selected=rng.sample(ids,min(neighborhood_size,len(ids)))
        # random.sample already draws an ordered uniformly random neighborhood.
        old={fid:schedules[fid] for fid in selected}
        old_cost=weighted(old)
        for s in old.values():
            index.release(s)
        replacement={}
        remaining=old_cost
        expired=False
        try:
            for fid in selected:
                weight=fm.WEIGHTS[old[fid].priority]
                response=best_response(templates[fid],index,net,max_delay=remaining/weight,deadline=deadline)
                if response is None:
                    break
                replacement[fid]=response
                remaining-=weight*response.delay_sec
                index.reserve(response)
        except BudgetExpired:
            expired=True
        if not expired and len(replacement)==len(selected) and remaining>ref.EPS:
            objective-=remaining
            schedules.update(replacement)
            accepted+=1
            now=time.perf_counter()-improvement_start
            trace.append({'improvement_sec':now,'weighted_delay_sec':objective})
            accepted_moves.append({'iteration':iterations,'ids':selected,'before_weighted_sec':old_cost,
                'after_weighted_sec':weighted(replacement),
                'individual_delays_before':{fid:s.delay_sec for fid,s in old.items()},
                'individual_delays_after':{fid:s.delay_sec for fid,s in replacement.items()}})
        else:
            for s in replacement.values():
                index.release(s)
            for s in old.values():
                index.reserve(s)
        if expired:
            status='TIME_LIMIT'; break
    metadata={'status':status,'initialization':initialization,'rng_seed':seed,
        'neighborhood_size':neighborhood_size,'improvement_limit_sec':improvement_limit,
        'iteration_limit':iteration_limit,'iterations':iterations,'accepted_moves':accepted,
        'template_build_sec':template_time,'initialization_sec':improvement_start-started-template_time,
        'improvement_sec':time.perf_counter()-improvement_start,'total_time_sec':time.perf_counter()-started,
        'initial_weighted_delay_sec':initial_cost,'weighted_delay_sec':objective,
        'initial_signature':fm.signature(initial_schedules) if complete else None,
        'trace':trace,'updates':accepted_moves,'proved_optimal':False,
        'variant':'MAPF-LNS random neighborhood with randomized prioritized repair; resource adaptation'}
    return schedules,metadata,initial_schedules
