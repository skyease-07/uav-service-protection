"""Change only the construction key; reuse exact common starts per mode."""
import math
import time
from indexed_scheduler import IntervalIndex, resource_intervals, shifted, signature, ref
import final_model as fm
from objective_common import selection


def construction_key(candidate, mode):
    tail = (candidate.finish_sec, candidate.route_index, candidate.layer_plan, candidate.lane_plan)
    if mode == 'composite':
        return (candidate.selection_cost, candidate.delay_sec, *tail)
    if mode == 'delay_first':
        return (candidate.delay_sec, candidate.selection_cost, *tail)
    raise ValueError(mode)


def prepare(fs, net, types):
    started = time.perf_counter()
    templates = {}
    for f in fs:
        templates[f.flight_id] = []
        for choice in ref.enumerate_structural_choices(f, net):
            base = fm.build_candidate(f, choice, 0, net, types)
            templates[f.flight_id].append((base, resource_intervals(base.resources)))
    return templates, time.perf_counter()-started


def construct(fs, net, templates, mode):
    started = time.perf_counter()
    policy, _ = fm.policy('SPC')
    index = IntervalIndex()
    schedules, failures = {}, []
    limit = int(math.floor((net.max_delay_sec+ref.EPS)/net.time_slot_sec))
    for f in sorted(fs, key=lambda f: ref._dispatch_key(f, policy)):
        best = best_key = None
        for base, intervals in templates[f.flight_id]:
            step = index.earliest(intervals, limit)
            if step is None:
                continue
            candidate = shifted(base, step, net, policy)
            key = construction_key(candidate, mode)
            if best_key is None or key < best_key:
                best, best_key = candidate, key
        if best is None:
            failures.append(f.flight_id)
        else:
            schedules[f.flight_id] = best
            index.reserve(best)
    return schedules, failures, time.perf_counter()-started


def improve(initial, net, templates, method, failures=()):
    started = time.perf_counter()
    initial_signature = fm.signature(initial)
    policy, _ = fm.policy(method)
    schedules = dict(initial)
    index = IntervalIndex()
    for schedule in schedules.values():
        index.reserve(schedule)
    updates, history = [], []
    stop = 'construction_failed' if failures else ('construction_only' if method == 'SPC' else 'pass_cap')
    for pass_no in range(1, policy.local_reinsert_passes+1) if not failures else []:
        before = sum(s.delay_sec for s in schedules.values())
        changed = 0
        order = sorted(schedules.values(), key=lambda s: (-policy.delay_weights[s.priority]*s.delay_sec,
                                                          s.priority, s.flight_id))
        for current in order:
            index.release(current)
            selected, selected_key = current, signature(current)
            bound = int(math.floor((current.delay_sec+ref.EPS)/net.time_slot_sec))
            for base, intervals in templates[current.flight_id]:
                step = index.earliest(intervals, bound)
                if step is None:
                    continue
                candidate = shifted(base, step, net, policy)
                key = signature(candidate)
                if key < selected_key:
                    selected, selected_key = candidate, key
            if selected is not current:
                changed += 1
                updates.append({'pass': pass_no, 'before': selection(current), 'after': selection(selected),
                                'elapsed_improvement_sec': time.perf_counter()-started})
            schedules[current.flight_id] = selected
            index.reserve(selected)
        history.append({'pass': pass_no, 'total_delay_before_sec': before,
                        'total_delay_after_sec': sum(s.delay_sec for s in schedules.values()), 'changed': changed})
        if not changed:
            stop = 'no_change'
            break
    assert fm.signature(initial) == initial_signature, 'Common start mutated'
    return schedules, {'improvement_sec': time.perf_counter()-started, 'stop_reason': stop,
                       'passes': len(history), 'history': history, 'updates': updates,
                       'initial_signature': initial_signature}
