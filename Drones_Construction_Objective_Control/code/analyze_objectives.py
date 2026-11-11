"""Prespecified descriptive paired-scenario summaries; no new primary tests."""
from collections import defaultdict
import csv
import numpy as np
from objective_common import *


def gain(reference,target):
    if reference==0:
        return 0. if target==0 else None
    return 100*(reference-target)/reference


def summarize(values,indices):
    x=np.asarray(values,dtype=float)
    assert np.isfinite(x).all()
    boot=np.mean(x[indices],axis=1)
    return {'n':len(x),'mean':float(x.mean()),'ci95':np.percentile(boot,[2.5,97.5]).tolist(),
            'median':float(np.median(x)),'min':float(x.min()),'max':float(x.max()),
            'positive':int(np.sum(x>1e-9)),'zero':int(np.sum(np.abs(x)<=1e-9)),
            'negative':int(np.sum(x< -1e-9)),'values':x.tolist()}


def main():
    lock=verify_lock()
    audit=read(ROOT/'validation/audit_checks.json')
    assert audit['complete'] and not audit['prior_changed']
    with (ROOT/'results/service_flights.csv').open(encoding='utf-8',newline='') as f:
        service=list(csv.DictReader(f))
    by_cell=defaultdict(list)
    for r in service:
        by_cell[(r['network'],int(r['seed']),r['construction'],r['method'])].append(r)
    out={'scope':P['status'],'lock_sha256':lock,'records':P['expected_records'],
         'independent_scenarios':P['independent_scenarios'],'groups':{},'undefined_endpoints':[]}
    rng=np.random.default_rng(P['analysis']['bootstrap_seed'])
    metrics=['ground_delay_sec','completion_from_request_sec','airborne_sec','horizontal_m','vertical_m']
    for group in P['groups']:
        seeds=list(range(*group['seeds']))
        records={(s,c,m):read(record_path(group,s,c,m)) for s in seeds for c in P['construction_modes'] for m in P['methods']}
        good=[s for s in seeds if all(records[s,c,m]['complete'] for c in P['construction_modes'] for m in P['methods'])]
        indices=rng.integers(0,len(good),size=(P['analysis']['bootstrap_replicates'],len(good))) if good else None
        g={'prespecified_n':len(seeds),'complete_paired_n':len(good),'seeds':good,'cells':{},'contrasts':{},'cross_mode':{}}
        if not good:
            out['groups'][group['name']]=g
            continue
        arrays={}
        for mode in P['construction_modes']:
            for method in P['methods']:
                rr=[records[s,mode,method] for s in good]
                arrays[mode,method]=np.array([r['weighted_delay_sec'] for r in rr])
                cell={'weighted_delay_sec':summarize(arrays[mode,method],indices),
                      'total_delay_sec':summarize([r['total_delay_sec'] for r in rr],indices),
                      'pass_cap_count':sum(r['solver']['stop_reason']=='pass_cap' for r in rr),
                      'no_change_count':sum(r['solver']['stop_reason']=='no_change' for r in rr),
                      'accepted_updates':summarize([len(r['solver']['updates']) for r in rr],indices),
                      'strict_delay_updates':summarize([sum(u['after']['delay_sec']<u['before']['delay_sec'] for u in r['solver']['updates']) for r in rr],indices),
                      'timing':{},'service':{}}
                for metric in ['template_sec','construction_sec','improvement_sec','checks_sec']:
                    cell['timing'][metric]=summarize([r['solver'][metric] for r in rr],indices)
                for metric in metrics:
                    cell['service'][metric]=summarize([np.mean([float(row[metric]) for row in by_cell[group['name'],s,mode,method]]) for s in good],indices)
                    delta='ground_delta_sec' if metric=='ground_delay_sec' else metric+'_delta'
                    cell['service'][metric+'_change']=summarize([np.mean([float(row[delta]) for row in by_cell[group['name'],s,mode,method]]) for s in good],indices)
                    tolerance=1e-6 if metric.endswith('_m') else 1e-8
                    counts=[sum(float(row[delta])>tolerance for row in by_cell[group['name'],s,mode,method]) for s in good]
                    cell['service'][metric+'_worse_pct']=summarize([100*n/group['n'] for n in counts],indices)
                    cell['service'][metric+'_worse_count']=sum(counts)
                g['cells'][mode+'/'+method]=cell
            for label,ref,target in [('ur_recovery','SPC','SPC-UR'),('wsr_recovery','SPC','SPC-WSR'),('weighting','SPC-UR','SPC-WSR')]:
                values=[gain(a,b) for a,b in zip(arrays[mode,ref],arrays[mode,target])]
                if any(v is None for v in values):
                    out['undefined_endpoints'].append([group['name'],mode,label])
                    continue
                g['contrasts'][mode+'/'+label]={'relative_pct':summarize(values,indices),
                    'absolute_weighted_sec_saved':summarize(arrays[mode,ref]-arrays[mode,target],indices)}
        for label in ['ur_recovery','wsr_recovery','weighting']:
            a=g['contrasts'].get('composite/'+label); b=g['contrasts'].get('delay_first/'+label)
            if a and b:
                g['cross_mode'][label+'_difference_pp']=summarize(np.array(b['relative_pct']['values'])-a['relative_pct']['values'],indices)
        for method in P['methods']:
            g['cross_mode'][method+'_weighted_delay_difference_sec']=summarize(arrays['delay_first',method]-arrays['composite',method],indices)
            for metric in metrics:
                a=g['cells']['composite/'+method]['service'][metric]['values']
                b=g['cells']['delay_first/'+method]['service'][metric]['values']
                g['cross_mode'][method+'_'+metric+'_difference']=summarize(np.array(b)-a,indices)
        out['groups'][group['name']]=g
    out['unconditional_descriptive_interpretation_permitted']=audit['all_nominally_separated'] and not out['undefined_endpoints'] and all(g['complete_paired_n']==g['prespecified_n'] for g in out['groups'].values())
    dump(ROOT/'results/analysis.json',out)
    for network,g in out['groups'].items():
        print(network,{k:round(v['relative_pct']['mean'],6) for k,v in g['contrasts'].items()})


if __name__=='__main__':
    main()
