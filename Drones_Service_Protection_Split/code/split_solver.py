"""Exact WSR branches with reconstructible, multi-label service diagnostics."""
import math
import time
from indexed_scheduler import IntervalIndex,resource_intervals,shifted,signature,ref
import final_model as fm
import service_metrics as sm
from split_common import METRICS,P,selection

GUARDS={k:v for k,v in P['guards'].items() if k!='SPC'}
def violations(after,before):
    return sum(1<<i for i,k in enumerate(METRICS)
               if after[k]>before[k]+(1e-6 if k.endswith('_m') else 1e-8))

def prepare(fs,net,types):
    start=time.perf_counter(); templates={}
    for f in fs:
        templates[f.flight_id]=[]
        for choice in ref.enumerate_structural_choices(f,net):
            base=fm.build_candidate(f,choice,0,net,types)
            templates[f.flight_id].append((base,resource_intervals(base.resources),sm.schedule_costs(base,net)))
    return templates,time.perf_counter()-start

def construct(fs,net,templates):
    start=time.perf_counter(); policy,_=fm.policy('SPC')
    index=IntervalIndex(); schedules={}; failures=[]
    limit=int(math.floor((net.max_delay_sec+ref.EPS)/net.time_slot_sec))
    for f in sorted(fs,key=lambda f:ref._dispatch_key(f,policy)):
        best=best_key=None
        for base,intervals,_ in templates[f.flight_id]:
            step=index.earliest(intervals,limit)
            if step is None: continue
            c=shifted(base,step,net,policy)
            key=(c.selection_cost,c.delay_sec,c.finish_sec,c.route_index,c.layer_plan,c.lane_plan)
            if best_key is None or key<best_key: best,best_key=c,key
        if best is None: failures.append(f.flight_id)
        else: schedules[f.flight_id]=best; index.reserve(best)
    return schedules,failures,time.perf_counter()-start

def scan(current,index,templates,net,policy):
    """Index has released current; every improvement is measured against current."""
    bound=int(math.floor((current.delay_sec+ref.EPS)/net.time_slot_sec))
    before=sm.schedule_costs(current,net); current_key=signature(current)
    best={m:(current_key,None,None) for m in GUARDS}
    steps=[]; improving=[]; infeasible=nonimproving=0
    for tid,(base,intervals,costs) in enumerate(templates):
        step=index.earliest(intervals,bound); steps.append(-1 if step is None else step)
        if step is None: infeasible+=1; continue
        candidate=shifted(base,step,net,policy); key=signature(candidate)
        if key>=current_key: nonimproving+=1; continue
        # Keep exactly the original pre-shift service calculation for parity.
        after={**costs,'finish_sec':costs['finish_sec']+step*net.time_slot_sec}
        mask=violations(after,before)
        improving.append({'template':tid,'step':step,'mask':mask,
            'delta':[after[k]-before[k] for k in METRICS],
            'delay_delta':candidate.delay_sec-current.delay_sec,
            'changes':[candidate.route_index!=current.route_index,
                       candidate.layer_plan!=current.layer_plan,candidate.lane_plan!=current.lane_plan,
                       candidate.speed_mode!=current.speed_mode]})
        for method,guard in GUARDS.items():
            if not (mask & guard) and key<best[method][0]: best[method]=(key,tid,candidate)
    event={'id':current.flight_id,'before':selection(current),'steps':steps,
           'infeasible':infeasible,'nonimproving':nonimproving,'improving':improving,
           'best':{m:v[1] for m,v in best.items()}}
    return best,event

def improve(initial,net,templates,method,failures=(),sink=None):
    start=time.perf_counter(); frozen=fm.signature(initial)
    policy,_=fm.policy('SPC' if method=='SPC' else 'SPC-WSR')
    schedules=dict(initial); index=IntervalIndex()
    for s in schedules.values(): index.reserve(s)
    updates=[]; history=[]; visits=0
    stop='construction_failed' if failures else ('construction_only' if method=='SPC' else 'pass_cap')
    for p in range(1,policy.local_reinsert_passes+1) if not failures else []:
        before=sum(s.delay_sec for s in schedules.values()); changed=0
        order=sorted(schedules.values(),key=lambda s:(-policy.delay_weights[s.priority]*s.delay_sec,s.priority,s.flight_id))
        for current in order:
            index.release(current)
            best,event=scan(current,index,templates[current.flight_id],net,policy)
            selected=best[method][2] or current
            event.update({'pass':p,'visit':visits,'accepted':best[method][1]})
            if sink is not None: sink(event)
            visits+=1
            if selected is not current:
                changed+=1
                updates.append({'pass':p,'before':selection(current),'after':selection(selected),
                                'elapsed_improvement_sec':time.perf_counter()-start})
            schedules[current.flight_id]=selected; index.reserve(selected)
        history.append({'pass':p,'total_delay_before_sec':before,
                        'total_delay_after_sec':sum(s.delay_sec for s in schedules.values()),'changed':changed})
        if not changed: stop='no_change'; break
    assert fm.signature(initial)==frozen
    return schedules,{'improvement_sec':time.perf_counter()-start,'stop_reason':stop,
        'passes':len(history),'history':history,'updates':updates,'visits':visits,
        'initial_signature':frozen,'service_guard_mask':P['guards'][method]}
