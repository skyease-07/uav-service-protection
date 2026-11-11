"""Instrumented parity adapter and optional service guard; original solver preserved."""
import math
import time
from indexed_scheduler import IntervalIndex,resource_intervals,shifted,signature,ref
import final_model as fm
from protected_lns import selection
import service_metrics as sm


def solve(fs,net,types,method,guard=False):
    start=time.perf_counter(); policy,_=fm.policy(method)
    templates={}
    for f in fs:
        templates[f.flight_id]=[]
        for choice in ref.enumerate_structural_choices(f,net):
            base=fm.build_candidate(f,choice,0,net,types)
            templates[f.flight_id].append((base,resource_intervals(base.resources),sm.schedule_costs(base,net)))
    index=IntervalIndex(); schedules={}; failures=[]
    limit=int(math.floor((net.max_delay_sec+ref.EPS)/net.time_slot_sec))
    for f in sorted(fs,key=lambda f:ref._dispatch_key(f,policy)):
        best=best_key=None
        for base,intervals,_costs in templates[f.flight_id]:
            step=index.earliest(intervals,limit)
            if step is None: continue
            c=shifted(base,step,net,policy)
            key=(c.selection_cost,c.delay_sec,c.finish_sec,c.route_index,c.layer_plan,c.lane_plan)
            if best_key is None or key<best_key: best,best_key=c,key
        if best is None: failures.append(f.flight_id)
        else: schedules[f.flight_id]=best; index.reserve(best)
    initial=dict(schedules); construction=time.perf_counter()-start
    updates=[]; history=[]; stop='construction_only' if not policy.local_reinsert_passes else 'pass_cap'
    for pass_no in range(1,policy.local_reinsert_passes+1) if not failures else []:
        before=sum(s.delay_sec for s in schedules.values()); changed=0
        order=sorted(schedules.values(),key=lambda s:(-policy.delay_weights[s.priority]*s.delay_sec,s.priority,s.flight_id))
        for current in order:
            index.release(current); selected=current; selected_key=signature(current)
            bound=int(math.floor((current.delay_sec+ref.EPS)/net.time_slot_sec))
            current_costs=sm.schedule_costs(current,net) if guard else None
            for base,intervals,costs in templates[current.flight_id]:
                step=index.earliest(intervals,bound)
                if step is None: continue
                if guard:
                    after={**costs,'finish_sec':costs['finish_sec']+step*net.time_slot_sec}
                    if not sm.allowed(after,current_costs): continue
                candidate=shifted(base,step,net,policy); key=signature(candidate)
                if key<selected_key: selected,selected_key=candidate,key
            if selected is not current:
                changed+=1
                updates.append({'pass':pass_no,'before':selection(current),'after':selection(selected),
                                'elapsed_sec':time.perf_counter()-start})
            schedules[current.flight_id]=selected; index.reserve(selected)
        history.append({'pass':pass_no,'total_delay_before_sec':before,
                        'total_delay_after_sec':sum(s.delay_sec for s in schedules.values()),'changed':changed})
        if changed==0: stop='no_change'; break
    improvement_end=time.perf_counter()
    conflicts=ref.audit_reservation_conflicts(schedules)
    meta={'runtime_sec':time.perf_counter()-start,'construction_sec':construction,
          'improvement_sec':improvement_end-start-construction,'stop_reason':stop,
          'passes':len(history),'history':history,'updates':updates,'service_guard':guard,
          'initial_signature':fm.signature(initial),'unscheduled':failures,
          'reservation_conflicts':len(conflicts)}
    return schedules,meta,initial
