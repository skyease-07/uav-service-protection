"""One physics/resource model for all current experiments and manuscript claims."""
from dataclasses import replace
from functools import partial
from pathlib import Path
import hashlib
import json
import math
import random
import numpy as np
from indexed_scheduler import ref

ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = json.loads((ROOT / 'final_protocol.json').read_text(encoding='utf-8'))
PHYSICS = PROTOCOL['physics']
WEIGHTS = {int(k): v for k, v in PROTOCOL['policy']['evaluation_weights'].items()}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')


def derivative_bound(edge, net):
    p0, p2 = (np.asarray(net.nodes_xy[n], dtype=float) for n in (edge.start_node, edge.end_node))
    if edge.geometry == 'line':
        assert not any(edge.lane_offsets_m)
        return float(np.linalg.norm(p2-p0))
    p1 = np.asarray(edge.control_point_xy, dtype=float)
    return float(2*max(np.linalg.norm(p1-p0), np.linalg.norm(p2-p1))
                 + math.pi*max(abs(v) for v in edge.lane_offsets_m))


def network(name='single_merge'):
    net = ref.default_network()
    if name != 'single_merge':
        width = {'layered20': 4, 'layered40': 8}[name]
        nodes = {f'N{s}_{j}': (s*400, j*300) for s in range(5) for j in range(width)}
        edges = {}
        for s in range(4):
            for j in range(width):
                for k in (j, j+1):
                    if k < width:
                        key = f'E{s}_{j}_{k}'
                        edges[key] = ref.EdgeConfig(key, f'N{s}_{j}', f'N{s+1}_{k}', (1, 2), 1., (0.,))
        net = replace(net, nodes_xy=nodes, edges=edges)
    return replace(net,
        edges={key: replace(edge, base_duration_sec=derivative_bound(edge, net)/PHYSICS['reference_speed_mps'])
               for key, edge in net.edges.items()},
        time_slot_sec=PHYSICS['slot_sec'], edge_headway_sec=PHYSICS['edge_trailing_sec'],
        node_buffer_before_sec=PHYSICS['node_buffer_each_side_sec'],
        node_buffer_after_sec=PHYSICS['node_buffer_each_side_sec'],
        vertical_buffer_sec=PHYSICS['vertical_buffer_each_side_sec'],
        max_delay_sec=PHYSICS['max_delay_sec'], trajectory_sample_sec=PHYSICS['sample_sec'])


def types():
    return {k: replace(t, vertical_speed_mps=PHYSICS['vertical_speed_mps'][k])
            for k, t in ref.default_uav_types().items()}


def routes(width, source, target):
    out = []
    def walk(stage, j, path):
        if stage == 4:
            if j == target:
                out.append(tuple(path))
            return
        for k in (j, j+1):
            if k < width:
                walk(stage+1, k, path+[f'E{stage}_{j}_{k}'])
    walk(0, source, [])
    return tuple(sorted(out)[:4])


def flights(n, seed, window, name='single_merge'):
    rng = random.Random(seed*31+7)
    fs = [replace(f, uav_type=rng.choice(('rescue', 'cargo', 'inspection')))
          for f in ref.generate_scenario(n, seed, window)]
    if name == 'single_merge':
        return fs
    width = {'layered20': 4, 'layered40': 8}[name]
    rng = random.Random(seed*101+19)
    out = []
    for f in fs:
        source = rng.randrange(width)
        target = source+rng.randrange(min(4, width-1-source)+1)
        out.append(replace(f, candidate_paths=routes(width, source, target)))
    return out


def build_candidate(flight, choice, delay, net, vehicle_types, *, structural_multiplier=1.):
    adjusted = replace(choice, structural_cost=structural_multiplier*choice.structural_cost)
    s = ref.build_schedule_candidate(flight, adjusted, delay, net, vehicle_types)
    origin = net.edges[choice.route[0]].start_node
    keys = ref._node_resource_keys(origin, (choice.layer_plan[0],),
                                   s.scheduled_start_sec, s.scheduled_start_sec, net)
    return replace(s, resources=s.resources | keys)


def builder(multiplier=1.):
    return partial(build_candidate, structural_multiplier=multiplier)


def policy(method, pressure=None):
    neutral = method in ('FCFS-ID', 'NP-UR-ID')
    reinsertion = method not in ('FCFS-ID', 'SPC')
    weights = pressure or (WEIGHTS if method == 'SPC-WSR' else {2: 1., 5: 1., 10: 1.})
    return ref.SchedulingPolicy(name=method, dispatch_rule='fcfs' if neutral else 'strict',
        delay_weights=weights, air_time_weight=PROTOCOL['policy']['air_time_weight'],
        weight_candidate_delay=False,
        local_reinsert_passes=PROTOCOL['policy']['passes'] if reinsertion else 0), neutral


def rebuild(fs, rows, net, vehicle_types, multiplier=1.):
    by_id = {f.flight_id: f for f in fs}
    schedules = {}
    for row in rows:
        f = by_id[row['id']]
        choice = next(c for c in ref.enumerate_structural_choices(f, net)
            if c.route_index == row['route_index'] and list(c.layer_plan) == row['layers']
            and list(c.lane_plan) == row['lanes'] and c.speed_mode == row['speed_mode'])
        schedules[f.flight_id] = build_candidate(f, choice, row['delay_sec'], net,
            vehicle_types, structural_multiplier=multiplier)
    return schedules


def signature(schedules):
    return sha_bytes(repr(sorted((fid, s.delay_sec, s.finish_sec, s.route_index,
        s.layer_plan, s.lane_plan, s.speed_mode, sorted(s.resources))
        for fid, s in schedules.items())).encode())


def sha_bytes(value):
    return hashlib.sha256(value).hexdigest()


def audit_speed(fs, schedules, net, vehicle_types):
    by_id = {f.flight_id: f for f in fs}
    hr, vr = [], []
    for fid, s in schedules.items():
        ut = vehicle_types[by_id[fid].uav_type]
        for index, segment in enumerate(s.segments):
            if segment.kind == 'edge':
                cap = PHYSICS['reference_speed_mps']*ut.cruise_factor
                if index == 0 and s.speed_mode == 'metered':
                    cap *= .82
                hr.append(derivative_bound(net.edges[segment.edge_id], net)/segment.duration_sec/cap)
            else:
                dz = abs(net.layer_altitude_m[segment.layer_to]-net.layer_altitude_m[segment.layer_from])
                vr.append(dz/segment.duration_sec/ut.vertical_speed_mps)
    h, v = max(hr, default=0), max(vr, default=0)
    return {'passed': h <= 1+1e-9 and v <= 1+1e-9,
            'horizontal_ratio': h, 'vertical_ratio': v}
