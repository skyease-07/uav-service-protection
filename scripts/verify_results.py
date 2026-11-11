"""Check published results, study coverage, paired endpoints and archive hashes."""
from pathlib import Path
from collections import defaultdict
import csv
import hashlib
import json
import math
import statistics

ROOT=Path(__file__).resolve().parents[1]
RESULTS=ROOT/'results'
def read(p): return json.loads(p.read_text(encoding='utf-8'))
def close(a,b): assert math.isclose(a,b,rel_tol=1e-10,abs_tol=1e-7),(a,b)

manifest=read(RESULTS/'source_manifest.json')
for path,entry in manifest.items():
    assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==entry['sha256'],path

all_rows={}
for study,count in [('main',540),('construction',720),('service',720)]:
    with (RESULTS/study/'scenario_metrics.csv').open(encoding='utf-8',newline='') as f:
        rows=list(csv.DictReader(f))
    assert len(rows)==count
    keys={(r['network'],int(r['seed']),r['construction'],r['method']) for r in rows}
    assert len(keys)==count
    expected=set()
    for network,start,stop in [('single_merge',20000,20080),('layered20',21000,21020),('layered40',22000,22020)]:
        for seed in range(start,stop):
            methods=['SPC','SPC-UR','SPC-WSR','WSR-S'] if study=='main' else (['SPC','SPC-UR','SPC-WSR'] if study=='construction' else ['SPC','WSR','F','T','D','ALL'])
            if study=='main' and network=='single_merge' and seed<20030:methods+=['LNS-I','LNS-M']
            for construction in (['composite','delay_first'] if study=='construction' else ['composite']):
                expected.update((network,seed,construction,m) for m in methods)
    assert keys==expected
    groups=defaultdict(list)
    for r in rows:
        assert r['complete']=='True' and r['speed_passed']=='True' and r['reservation_conflicts']=='0'
        groups[(r['network'],r['construction'],r['method'])].append(float(r['weighted_delay_sec']))
    analysis=read(RESULTS/study/'analysis.json')
    for (network,construction,method),values in groups.items():
        if study=='main':
            g=analysis['groups']['main' if network=='single_merge' else network]['methods']
            target=g[method]['weighted_delay_sec'] if method in g else analysis['literature'][method]['weighted_delay_sec']
        elif study=='construction':target=analysis['groups'][network]['cells'][construction+'/'+method]['weighted_delay_sec']
        else:target=analysis['groups'][network]['cells'][method]['W']
        assert target['n']==len(values)
        close(statistics.mean(values),target['mean'])
    weights={'1':3,'2':2,'3':1}
    model={'main':'Drones_Revision_Stage7','construction':'Drones_Construction_Objective_Control','service':'Drones_Service_Protection_Split'}[study]
    weights=read(ROOT/model/'final_protocol.json')['policy']['evaluation_weights']
    sums=defaultdict(float);counts=defaultdict(int)
    with (RESULTS/study/'service_flights.csv').open(encoding='utf-8-sig',newline='') as f:
        for r in csv.DictReader(f):
            key=(r['network'],int(r['seed']),r.get('construction','composite'),r['method'])
            sums[key]+=weights[r['priority']]*float(r['ground_delay_sec']);counts[key]+=1
    assert set(sums)==keys
    for r in rows:
        key=(r['network'],int(r['seed']),r['construction'],r['method'])
        assert counts[key]==int(r['n'])
        close(sums[key],float(r['weighted_delay_sec']))
    all_rows[study]=rows
    print(study, len(rows), 'method records;',sum(counts.values()),'flight metrics: PASS')

main=read(RESULTS/'main/analysis.json')
lookup={(int(r['seed']),r['method']):float(r['weighted_delay_sec']) for r in all_rows['main'] if r['network']=='single_merge'}
for h in main['primary']:
    values=[100*(lookup[s,h['comparator']]-lookup[s,'SPC-WSR'])/lookup[s,h['comparator']] for s in range(20000,20080)]
    close(statistics.mean(values),h['mean_pp'])
    assert len(values)==h['n']
    for a,b in zip(values,h['values_pp'],strict=True):close(a,b)
print('Original-file hashes, exact scenario coverage, all weighted-delay means, flight totals and H1/H2 paired values: PASS')
