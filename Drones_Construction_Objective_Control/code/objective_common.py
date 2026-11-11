"""Shared paths, serialization and the fixed follow-up design."""
from pathlib import Path
import hashlib
import json
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
PRIOR = ROOT.parent / 'Drones_Revision_Stage7'
P = json.loads((ROOT/'protocol.json').read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def dump(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')


def source_hashes():
    return {p.relative_to(ROOT).as_posix(): sha(p) for p in sorted((ROOT/'code').rglob('*.py'))
            if p.name not in {'report_objectives.py', 'package_objectives.py'}}


def cases():
    for group in P['groups']:
        for seed in range(*group['seeds']):
            yield group, seed


def case_dir(group, seed):
    return ROOT/'results'/group['name']/f'seed{seed}'


def record_path(group, seed, mode, method):
    return case_dir(group, seed)/mode/method/'run.json'


def prior_path(group, seed, method):
    folder = 'main' if group['name'] == 'single_merge' else 'large'
    key = f"{group['name']}_n{group['n']}_tw{group['window_sec']}_seed{seed}_{method}"
    return PRIOR/'results/confirmation'/folder/key/'run.json'


def verify_lock():
    lock = read(ROOT/'validation/execution_lock.json')
    assert lock['protocol_sha256'] == sha(ROOT/'protocol.json')
    assert lock['source_hashes'] == source_hashes(), 'Locked source changed'
    for name, digest in lock['model_protocol_hashes'].items():
        assert sha(ROOT/name) == digest
    return sha(ROOT/'validation/execution_lock.json')


def selection(s):
    return {'id': s.flight_id, 'priority': s.priority, 'delay_sec': s.delay_sec,
            'finish_sec': s.finish_sec, 'route_index': s.route_index,
            'layers': list(s.layer_plan), 'lanes': list(s.lane_plan),
            'speed_mode': s.speed_mode, 'structural_cost': s.structural_cost,
            'airborne_sec': s.finish_sec-s.scheduled_start_sec}


def weighted(schedules):
    import final_model as fm
    return sum(fm.WEIGHTS[s.priority]*s.delay_sec for s in schedules.values())
