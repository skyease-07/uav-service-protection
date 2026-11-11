"""Independent event arithmetic, update replay and final nominal geometry."""
from collections import Counter
from dataclasses import asdict
import csv
import gzip
import final_model as fm
import independent_geometry as ig
import service_metrics as sm
from indexed_scheduler import signature,shifted
import split_solver as solver
from split_common import *

def rebuild(fs,rows,net,types):
    schedules=fm.rebuild(fs,rows,net,types)
    assert all(selection(schedules[r['id']])==r for r in rows)
    return schedules

def audit_events(path,r,initial,templates,net):
    """Recompute classifications from all logged steps, then replay accepted moves.

    Feasible improving candidates are checked against full reservation keys.
    Earliest-step computation uses the frozen, previously tested interval index;
    its outputs for infeasible/non-improving templates are logged, not independently
    exhaustively re-solved here. Full old-policy parity provides a separate check.
    """
    current=dict(initial); occupied={k:fid for fid,s in current.items() for k in s.resources}
    policy,_=fm.policy('SPC-WSR'); updates=[]; nvisits=ncandidates=0
    last_pass=0; order=[]; position=0; history=[]; changed=0; before_total=0.
    with gzip.open(path,'rt',encoding='utf-8') as stream:
        for line in stream:
            e=json.loads(line)
            if e['pass']!=last_pass:
                if last_pass:
                    history.append({'pass':last_pass,'total_delay_before_sec':before_total,
                                    'total_delay_after_sec':sum(s.delay_sec for s in current.values()),'changed':changed})
                assert e['pass']==last_pass+1
                last_pass=e['pass']; position=0; changed=0; before_total=sum(s.delay_sec for s in current.values())
                order=sorted(current.values(),key=lambda s:(-fm.WEIGHTS[s.priority]*s.delay_sec,s.priority,s.flight_id))
            assert e['visit']==nvisits and e['id']==order[position].flight_id
            nvisits+=1; position+=1
            before=current[e['id']]; assert selection(before)==e['before']
            for k in before.resources: assert occupied.pop(k)==before.flight_id
            old_key=signature(before); oldcost=sm.schedule_costs(before,net)
            best={m:(old_key,None) for m in solver.GUARDS}; improving=[]; infeasible=nonimproving=0
            tlist=templates[e['id']]; assert len(tlist)==len(e['steps'])
            for tid,((base,intervals,costs),step) in enumerate(zip(tlist,e['steps'])):
                ncandidates+=1
                if step==-1: infeasible+=1; continue
                assert isinstance(step,int) and 0<=step<=int(before.delay_sec/net.time_slot_sec)
                delay=step*net.time_slot_sec; finish=base.finish_sec+delay
                airborne=finish-(base.scheduled_start_sec+delay)
                cost=1.0*delay+base.structural_cost+policy.air_time_weight*airborne
                key=(delay,cost,finish,base.route_index,base.layer_plan,base.lane_plan)
                if key>=old_key: nonimproving+=1; continue
                after={**costs,'finish_sec':costs['finish_sec']+delay}
                mask=sum(1<<j for j,k in enumerate(METRICS) if after[k]>oldcost[k]+(1e-6 if k.endswith('_m') else 1e-8))
                entry={'template':tid,'step':step,'mask':mask,'delta':[after[k]-oldcost[k] for k in METRICS],
                    'delay_delta':delay-before.delay_sec,'changes':[base.route_index!=before.route_index,
                        base.layer_plan!=before.layer_plan,base.lane_plan!=before.lane_plan,base.speed_mode!=before.speed_mode]}
                improving.append(entry)
                assert all(k[:-1]+(k[-1]+step,) not in occupied for k in base.resources)
                assert not (mask&1) or bool(mask&2),'F violation without T violation despite protected delay'
                for m,g in solver.GUARDS.items():
                    if not mask&g and key<best[m][0]: best[m]=(key,tid)
            assert improving==e['improving']
            assert infeasible==e['infeasible'] and nonimproving==e['nonimproving']
            assert len(tlist)==infeasible+nonimproving+len(improving)
            assert {m:v[1] for m,v in best.items()}==e['best']
            assert e['accepted']==e['best'][r['method']]
            tid=e['accepted']; after=before
            if tid is not None:
                after=shifted(tlist[tid][0],e['steps'][tid],net,policy)
                assert after.delay_sec<=before.delay_sec and signature(after)<old_key
                assert not solver.violations(sm.schedule_costs(after,net),oldcost)&P['guards'][r['method']]
                updates.append((e['pass'],selection(before),selection(after))); changed+=1
            for k in after.resources:
                assert k not in occupied; occupied[k]=after.flight_id
            current[e['id']]=after
    if last_pass:
        assert position==len(current)
        history.append({'pass':last_pass,'total_delay_before_sec':before_total,
            'total_delay_after_sec':sum(s.delay_sec for s in current.values()),'changed':changed})
    assert nvisits==r['solver']['visits']
    assert history==r['solver']['history']
    assert updates==[(u['pass'],u['before'],u['after']) for u in r['solver']['updates']]
    assert fm.signature(current)==r['schedule_signature']
    return nvisits,ncandidates

def verify_record(path,g,seed,lock,cache,service_rows,templates):
    r=read(path); method=r['method']; folder=case_dir(g,seed)
    assert (r['network'],r['seed'],method)==(g['name'],seed,path.parent.name)
    assert r['lock_sha256']==lock
    for n,h in r['input_hashes'].items(): assert sha(folder/n)==h
    spc=read(record_path(g,seed,'SPC'))
    assert r['input_hashes']==spc['input_hashes'] and r['initial_signature']==spc['schedule_signature']
    if 'prior_parity' in r: assert all(v for k,v in r['prior_parity'].items() if k!='prior_run_sha256')
    events=path.parent/'events.jsonl.gz'; assert sha(events)==r['events_sha256']
    check={'file':path.relative_to(ROOT).as_posix(),'run_sha256':sha(path),'network':g['name'],'seed':seed,'method':method}
    if not r['complete']: return {**check,'status':'incomplete','error':r['error']}
    fs=fm.flights(g['n'],seed,g['window_sec'],g['name']); net,types=fm.network(g['name']),fm.types()
    assert fs==fm.ref.load_scenario_csv(folder/'scenario.csv')
    assert read(folder/'network.json')==json.loads(json.dumps(asdict(net)))
    assert read(folder/'types.json')==json.loads(json.dumps({k:asdict(v) for k,v in types.items()}))
    initial=rebuild(fs,r['initial_per_flight'],net,types); final=rebuild(fs,r['per_flight'],net,types)
    assert len(initial)==len(final)==g['n'] and fm.signature(initial)==r['initial_signature']
    visits,candidates=audit_events(events,r,initial,templates,net)
    assert weighted(final)==r['weighted_delay_sec'] and fm.signature(final)==r['schedule_signature']
    assert sum(s.delay_sec for s in final.values())==r['total_delay_sec']
    assert not fm.ref.audit_reservation_conflicts(final)
    assert fm.ref.audit_edge_capacity(final,net)['overloaded_edge_layer_slots']==0
    assert fm.audit_speed(fs,final,net,types)['passed']
    scenario=ig.read_scenario(folder/'scenario.csv'); args=(read(folder/'network.json'),read(folder/'types.json'))
    motions,thresholds=ig.reconstruct(scenario,r['per_flight'],*args)
    init_motions,_=ig.reconstruct(scenario,r['initial_per_flight'],*args)
    sig=ig.motion_signature(motions,thresholds); reused=sig in cache
    if not reused:
        cfg=read(ROOT/'geometry_capacity_protocol.json')['geometry']
        cache[sig]=ig.audit(motions,thresholds,guard=cfg['numerical_guard_m'],min_interval=cfg['min_interval_sec'],
                            max_depth=cfg['max_depth'],max_cells=cfg['max_cells_per_segment_pair'])
    for f in fs:
        fid=f.flight_id; val=sm.from_motion(motions[fid],f.requested_start_sec); old=sm.from_motion(init_motions[fid],f.requested_start_sec)
        assert final[fid].delay_sec<=initial[fid].delay_sec
        assert all(abs(val[k]-v)<1e-6 for k,v in sm.schedule_costs(final[fid],net).items())
        service_rows.append({'network':g['name'],'seed':seed,'method':method,'id':fid,'priority':f.priority,'uav_type':f.uav_type,
            'ground_delay_sec':final[fid].delay_sec,'ground_delta_sec':final[fid].delay_sec-initial[fid].delay_sec,
            **val,**{k+'_delta':val[k]-old[k] for k in val}})
    return {**check,'status':cache[sig]['status'],'updates_checked':len(r['solver']['updates']),
            'visits_checked':visits,'candidate_classifications_checked':candidates,
            'motion_signature':sig,'geometry_reused':reused,'geometry':cache[sig]}

def main():
    lock=verify_lock(); expected={record_path(g,s,m) for g,s in cases() for m in P['methods']}
    assert set((ROOT/'results').glob('*/*/*/run.json'))==expected
    cache_path=ROOT/'validation/geometry_cache.json'; cache=read(cache_path) if cache_path.exists() else {}
    checks=[]; errors=[]; rows=[]
    for g,s in cases():
        fs=fm.flights(g['n'],s,g['window_sec'],g['name']); net,types=fm.network(g['name']),fm.types()
        templates,_=solver.prepare(fs,net,types)
        for m in P['methods']:
            path=record_path(g,s,m)
            try: checks.append(verify_record(path,g,s,lock,cache,rows,templates))
            except Exception as exc: errors.append({'file':str(path),'error':repr(exc)})
        dump(cache_path,cache)
        dump(ROOT/'validation/audit_progress.json',{'last':[g['name'],s],'checked':len(checks),'errors':errors})
        print('AUDIT',g['name'],s,len(checks),'errors',len(errors),flush=True)
    prior=read(ROOT/'validation/prior_preservation.json')
    changed=[n for n,h in prior['files'].items() if sha(Path(prior['workspace'])/n)!=h]
    statuses=dict(Counter(c['status'] for c in checks))
    report={'records':len(expected),'checks':checks,'errors':errors,'statuses':statuses,'prior_changed':changed,
        'prior_files_checked':len(prior['files']),'complete':not errors and len(checks)==len(expected),
        'all_nominally_separated':not errors and len(checks)==statuses.get('separated',0),
        'archived_records_reproduced':sum(c['method'] in ('SPC','WSR','ALL') for c in checks),
        'lock_sha256':lock,'service_rows':len(rows),
        'limits':'Binary64 nominal interval checks. Logs classify all recorded earliest steps; improving candidate resource keys and every accepted update checked. Infeasible/non-improving earliest-step computation is not independently exhaustively re-solved.'}
    dump(ROOT/'validation/audit_checks.json',report)
    with (ROOT/'results/service_flights.csv').open('w',encoding='utf-8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]) if rows else ['network']); w.writeheader(); w.writerows(rows)
    print('FINAL',statuses,'errors',len(errors),'prior changed',len(changed),flush=True)
    assert not errors and not changed
if __name__=='__main__': main()
