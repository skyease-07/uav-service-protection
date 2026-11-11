"""Fixed paths, design, hashes and canonical selections."""
from pathlib import Path
import hashlib
import json
import sys
sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
PRIOR = ROOT.parent/'Drones_Revision_Stage7'
P = json.loads((ROOT/'protocol.json').read_text(encoding='utf-8'))
METRICS = ('finish_sec','airborne_sec','horizontal_m','vertical_m')
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path): return json.loads(Path(path).read_text(encoding='utf-8'))
def dump(path,value):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
def source_hashes():
    return {p.relative_to(ROOT).as_posix():sha(p) for p in sorted((ROOT/'code').rglob('*.py'))
            if p.name not in ('report_split.py','package_split.py')}
def cases():
    for g in P['groups']:
        for s in range(*g['seeds']): yield g,s
def case_dir(g,s): return ROOT/'results'/g['name']/f'seed{s}'
def record_path(g,s,m): return case_dir(g,s)/m/'run.json'
def prior_path(g,s,m):
    old={'SPC':'SPC','WSR':'SPC-WSR','ALL':'WSR-S'}[m]
    group='main' if g['name']=='single_merge' else 'large'
    return PRIOR/'results/confirmation'/group/f"{g['name']}_n{g['n']}_tw{g['window_sec']}_seed{s}_{old}"/'run.json'
def verify_lock():
    lock=read(ROOT/'validation/execution_lock.json')
    assert lock['protocol_sha256']==sha(ROOT/'protocol.json')
    assert lock['source_hashes']==source_hashes(),'Locked source changed'
    for n,h in lock['model_protocol_hashes'].items(): assert sha(ROOT/n)==h
    return sha(ROOT/'validation/execution_lock.json')
def selection(s):
    return {'id':s.flight_id,'priority':s.priority,'delay_sec':s.delay_sec,'finish_sec':s.finish_sec,
            'route_index':s.route_index,'layers':list(s.layer_plan),'lanes':list(s.lane_plan),
            'speed_mode':s.speed_mode,'structural_cost':s.structural_cost,
            'airborne_sec':s.finish_sec-s.scheduled_start_sec}
def weighted(schedules):
    import final_model as fm
    return sum(fm.WEIGHTS[s.priority]*s.delay_sec for s in schedules.values())
