"""Portable entry point for the unmodified scientific solvers; one study per process."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parents[1]
STUDIES = {'main': 'Drones_Revision_Stage7',
           'construction': 'Drones_Construction_Objective_Control',
           'service': 'Drones_Service_Protection_Split'}


def normalized(value):
    if isinstance(value, dict):
        return {k: normalized(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [normalized(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study', choices=STUDIES, default='main')
    parser.add_argument('--network', choices=['single_merge', 'layered20', 'layered40'], default='single_merge')
    parser.add_argument('--n', type=int, default=12)
    parser.add_argument('--seed', type=int, default=100)
    parser.add_argument('--window', type=int, default=60)
    parser.add_argument('--include-lns', action='store_true', help='Main study only; fixed 1000-iteration LNS-I/M baselines')
    parser.add_argument('--output', type=Path, required=True, help='New directory; existing paths are rejected')
    args = parser.parse_args()
    if args.n < 1 or args.window < 0:
        parser.error('n must be positive and window nonnegative')
    if args.include_lns and args.study != 'main':
        parser.error('--include-lns is supported only for --study main')
    args.output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(ROOT / STUDIES[args.study] / 'code'))
    import numpy as np
    import scipy
    from threadpoolctl import threadpool_limits, threadpool_info
    import final_model as fm
    import protected_lns as lns
    import service_metrics as sm
    fs, net, types = fm.flights(args.n, args.seed, args.window, args.network), fm.network(args.network), fm.types()
    records = []

    def record(name, schedules, meta, initial):
        conflicts = fm.ref.audit_reservation_conflicts(schedules)
        speed = fm.audit_speed(fs, schedules, net, types)
        complete = len(schedules) == args.n
        rows = [lns.selection(s) for _, s in sorted(schedules.items())]
        records.append({'method': name, 'complete': complete,
                        'weighted_delay_sec': sum(fm.WEIGHTS[s.priority] * s.delay_sec for s in schedules.values()) if complete else None,
                        'initial_signature': fm.signature(initial), 'schedule_signature': fm.signature(schedules),
                        'initial_per_flight': [lns.selection(s) for _, s in sorted(initial.items())],
                        'per_flight': rows,
                        'service_metrics': {fid: sm.schedule_costs(s, net) for fid, s in schedules.items()},
                        'solver': normalized(meta), 'audits': {'reservation_conflicts': len(conflicts), 'speed': speed}})
        print(name, 'W=', records[-1]['weighted_delay_sec'], 'complete=', complete, flush=True)
        if not complete or conflicts or not speed['passed']:
            raise RuntimeError(f'{name}: incomplete schedule or failed resource/speed audit')

    with threadpool_limits(limits=1):
        environment = {'python': sys.version, 'platform': platform.platform(), 'numpy': np.__version__,
                       'scipy': scipy.__version__, 'threadpools': threadpool_info()}
        if args.study == 'main':
            import confirmation_solver as solver
            for method in ('SPC', 'SPC-UR', 'SPC-WSR', 'WSR-S'):
                schedules, meta, initial = solver.solve(fs, net, types, 'SPC-WSR' if method == 'WSR-S' else method, guard=method == 'WSR-S')
                record(method, schedules, meta, initial)
            if args.include_lns:
                for method, mode in [('LNS-I', 'initial'), ('LNS-M', 'monotone')]:
                    schedules, meta, initial = lns.solve(fs, net, types, candidate_builder=fm.builder(), mode=mode,
                        seed=1030000 + args.seed, improvement_limit=math.inf, neighborhood_size=8, iteration_limit=1000)
                    record(method, schedules, meta, initial)
        elif args.study == 'construction':
            import objective_solver as solver
            templates, _ = solver.prepare(fs, net, types)
            for mode in ('composite', 'delay_first'):
                initial, failures, _ = solver.construct(fs, net, templates, mode)
                for method in ('SPC', 'SPC-UR', 'SPC-WSR'):
                    schedules, meta = solver.improve(initial, net, templates, method, failures)
                    record(mode + '/' + method, schedules, meta, initial)
        else:
            import gzip
            import split_solver as solver
            templates, _ = solver.prepare(fs, net, types)
            initial, failures, _ = solver.construct(fs, net, templates)
            for method in ('SPC', 'WSR', 'F', 'T', 'D', 'ALL'):
                with gzip.open(args.output / (method + '.events.jsonl.gz'), 'wt', encoding='utf-8') as stream:
                    def sink(event):
                        stream.write(json.dumps(event, allow_nan=False) + '\n')
                    schedules, meta = solver.improve(initial, net, templates, method, failures, sink)
                record(method, schedules, meta, initial)
    inputs = {'scenario': [asdict(f) for f in fs], 'network': asdict(net), 'types': {k: asdict(v) for k, v in types.items()}}
    fm.dump(args.output / 'inputs.json', inputs)
    fm.dump(args.output / 'results.json', {'study': args.study, 'network': args.network, 'n': args.n,
        'seed': args.seed, 'window_sec': args.window, 'environment': environment, 'records': records})
    print('Saved:', args.output.resolve())


if __name__ == '__main__':
    main()
