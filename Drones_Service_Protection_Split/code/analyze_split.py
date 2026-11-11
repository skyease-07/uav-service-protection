"""Descriptive paired-scenario analysis; candidate counts are never sample size."""
from collections import defaultdict,Counter
import csv
import gzip
import numpy as np
from split_common import *

def gain(a,b): return (100*(a-b)/a) if a else (0. if b==0 else None)
def summarize(values,indices=None):
    x=np.asarray(values,dtype=float)
    assert np.isfinite(x).all()
    if not len(x): return {'n':0,'mean':None,'ci95':None,'values':[]}
    out={'n':len(x),'mean':float(x.mean()),'median':float(np.median(x)),
         'min':float(x.min()),'max':float(x.max()),'values':x.tolist()}
    if indices is not None: out['ci95']=np.percentile(np.mean(x[indices],axis=1),[2.5,97.5]).tolist()
    return out

def events_summary(path):
    out={'visits':0,'infeasible_candidates':0,'nonimproving_candidates':0,'improving_candidates':0,
         'opportunities':0,'candidate_masks':Counter(),'best_masks':Counter(),
         'guards':{m:Counter() for m in ('F','T','D','ALL')},'changes':Counter(),
         'positive_deltas':{k:[] for k in METRICS},'witnesses':{}}
    with gzip.open(path,'rt',encoding='utf-8') as f:
        for line in f:
            e=json.loads(line); out['visits']+=1
            out['infeasible_candidates']+=e['infeasible']; out['nonimproving_candidates']+=e['nonimproving']
            out['improving_candidates']+=len(e['improving'])
            if not e['improving']: continue
            out['opportunities']+=1
            candidates={c['template']:c for c in e['improving']}; best=candidates[e['best']['WSR']]
            out['best_masks'][str(best['mask'])]+=1
            for c in e['improving']:
                out['candidate_masks'][str(c['mask'])]+=1
                out['changes'][','.join(str(int(x)) for x in c['changes'])]+=1
                for i,k in enumerate(METRICS):
                    if c['mask']&(1<<i):
                        out['positive_deltas'][k].append(c['delta'][i])
                        if k not in out['witnesses']: out['witnesses'][k]={'pass':e['pass'],'visit':e['visit'],'before':e['before'],**c}
            for m,g in ((m,P['guards'][m]) for m in out['guards']):
                d=out['guards'][m]; d['opportunities']+=1
                d['rejected_candidates']+=sum(bool(c['mask']&g) for c in e['improving'])
                if best['mask']&g:
                    d['best_blocked']+=1
                    d['alternative_available' if e['best'][m] is not None else 'fully_blocked']+=1
                else: d['best_allowed']+=1
    out['positive_deltas']={k:({'n':len(v),'mean':float(np.mean(v)),'p95':float(np.percentile(v,95)),'max':max(v)} if v else {'n':0}) for k,v in out['positive_deltas'].items()}
    return out

def mechanism_group(rr,rng):
    totals={k:sum(r[k] for r in rr) for k in ('visits','infeasible_candidates','nonimproving_candidates','improving_candidates','opportunities')}
    for name in ('candidate_masks','best_masks','changes'):
        c=Counter()
        for r in rr: c.update(r[name])
        totals[name]=dict(c)
    totals['guards']={}
    for m in ('F','T','D','ALL'):
        counts=Counter()
        for r in rr: counts.update(r['guards'][m])
        vals={}
        for key in ('best_blocked','alternative_available','fully_blocked','best_allowed'):
            values=[100*r['guards'][m].get(key,0)/r['opportunities'] for r in rr if r['opportunities']]
            idx=rng.integers(0,len(values),size=(P['analysis']['bootstrap_replicates'],len(values))) if values else None
            vals[key+'_pct']=summarize(values,idx)
        totals['guards'][m]={'counts':dict(counts),'scenario_rates':vals,
                           'zero_opportunity_scenarios':sum(r['opportunities']==0 for r in rr)}
    return totals

def main():
    lock=verify_lock(); audit=read(ROOT/'validation/audit_checks.json')
    assert audit['complete'] and not audit['prior_changed']
    with (ROOT/'results/service_flights.csv').open(encoding='utf-8',newline='') as f: rows=list(csv.DictReader(f))
    by=defaultdict(list)
    for r in rows: by[r['network'],int(r['seed']),r['method']].append(r)
    out={'scope':P['status'],'lock_sha256':lock,'groups':{},'records':720,'independent_scenarios':120}
    rng=np.random.default_rng(P['analysis']['bootstrap_seed'])
    for g in P['groups']:
        seeds=list(range(*g['seeds'])); rec={(s,m):read(record_path(g,s,m)) for s in seeds for m in P['methods']}
        good=[s for s in seeds if all(rec[s,m]['complete'] for m in P['methods'])]
        if not good:
            out['groups'][g['name']]={'complete_paired_n':0}; continue
        idx=rng.integers(0,len(good),size=(P['analysis']['bootstrap_replicates'],len(good)))
        w={m:np.array([rec[s,m]['weighted_delay_sec'] for s in good]) for m in P['methods']}
        base=w['SPC']; unguarded=base-w['WSR']
        group={'prespecified_n':len(seeds),'complete_paired_n':len(good),'seeds':good,
               'zero_wsr_gain_scenarios':int(np.sum(unguarded==0)),'cells':{},'mechanisms':{}}
        for m in P['methods']:
            rr=[rec[s,m] for s in good]; saved=base-w[m]; recovery=[gain(a,b) for a,b in zip(base,w[m])]
            assert all(v is not None for v in recovery)
            cell={'W':summarize(w[m],idx),'recovery_pct':summarize(recovery,idx),
                'saved_weighted_sec':summarize(saved,idx),'loss_vs_wsr_weighted_sec':summarize(w[m]-w['WSR'],idx),
                'recovery_loss_vs_wsr_pp':summarize([gain(a,b)-gain(a,c) for a,b,c in zip(base,w['WSR'],w[m])],idx),
                'accepted_updates':summarize([len(r['solver']['updates']) for r in rr],idx),
                'strict_delay_updates':summarize([sum(u['after']['delay_sec']<u['before']['delay_sec'] for u in r['solver']['updates']) for r in rr],idx),
                'passes':summarize([r['solver']['passes'] for r in rr],idx),
                'pass_cap_count':sum(r['solver']['stop_reason']=='pass_cap' for r in rr),
                'no_change_count':sum(r['solver']['stop_reason']=='no_change' for r in rr),
                'zero_update_count':sum(not r['solver']['updates'] for r in rr),'service':{},'timing':{}}
            denom=unguarded[idx].sum(axis=1); valid=denom!=0
            cell['aggregate_gain_retention_pct']={'value':float(100*saved.sum()/unguarded.sum()) if unguarded.sum() else None,
                'ci95':np.percentile(100*saved[idx].sum(axis=1)[valid]/denom[valid],[2.5,97.5]).tolist() if valid.any() else None,
                'undefined_bootstrap_replicates':int(np.sum(~valid))}
            for metric in ('completion_from_request_sec','airborne_sec','horizontal_m','vertical_m'):
                delta=metric+'_delta'; tol=1e-6 if metric.endswith('_m') else 1e-8
                rrr=[by[g['name'],s,m] for s in good]
                counts=[sum(float(r[delta])>tol for r in r0) for r0 in rrr]
                pos=[float(r[delta]) for r0 in rrr for r in r0 if float(r[delta])>tol]
                cell['service'][metric]={'final_mean':summarize([np.mean([float(r[metric]) for r in r0]) for r0 in rrr],idx),
                    'change':summarize([np.mean([float(r[delta]) for r in r0]) for r0 in rrr],idx),
                    'worse_pct':summarize([100*n/g['n'] for n in counts],idx),'worse_count':sum(counts),
                    'positive_delta_tail':{'n':len(pos),'p95':float(np.percentile(pos,95)) if pos else 0.,'max':max(pos,default=0.)}}
            for metric in ('template_sec','construction_sec','improvement_sec','checks_sec'):
                cell['timing'][metric]=summarize([r['solver'][metric] for r in rr],idx)
            group['cells'][m]=cell
            if m!='SPC':
                ms=[]
                for s in good:
                    summary=events_summary(record_path(g,s,m).parent/'events.jsonl.gz')
                    dump(record_path(g,s,m).parent/'mechanism.json',summary); ms.append(summary)
                group['mechanisms'][m]=mechanism_group(ms,rng)
        group['T_minus_F_W']=summarize(w['T']-w['F'],idx)
        out['groups'][g['name']]=group
        print(g['name'],{m:round(c['recovery_pct']['mean'],6) for m,c in group['cells'].items()},flush=True)
    out['all_complete_and_nominally_separated']=audit['all_nominally_separated'] and all(g['complete_paired_n']==g['prespecified_n'] for g in out['groups'].values())
    dump(ROOT/'results/analysis.json',out)
if __name__=='__main__': main()
