"""Exact interval index for the frozen reservation grid; no policy changes by default."""
from __future__ import annotations

import math
import sys
import time
from collections import defaultdict
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "frozen"))
import uav_scheduler_delay_optimized as ref


def resource_intervals(keys):
    groups = defaultdict(list)
    for key in keys:
        groups[key[:-1]].append(key[-1])
    intervals = []
    for prefix, slots in groups.items():
        slots = sorted(slots)
        lo = hi = slots[0]
        for slot in slots[1:]:
            if slot == hi + 1:
                hi = slot
            else:
                intervals.append((prefix, lo, hi))
                lo = hi = slot
        intervals.append((prefix, lo, hi))
    return intervals


class IntervalIndex:
    def __init__(self):
        self.by_resource = defaultdict(list)

    def reserve(self, schedule):
        for prefix, lo, hi in resource_intervals(schedule.resources):
            self.by_resource[prefix].append((lo, hi, schedule.flight_id))

    def release(self, schedule):
        for prefix in {key[:-1] for key in schedule.resources}:
            self.by_resource[prefix] = [x for x in self.by_resource[prefix]
                                        if x[2] != schedule.flight_id]

    def earliest(self, intervals, max_step):
        blocked = np.zeros(max_step + 1, dtype=bool)
        # Two inclusive slot intervals overlap iff the shift is in [a-hi, b-lo].
        for prefix, lo, hi in intervals:
            for a, b, _owner in self.by_resource[prefix]:
                start, stop = max(0, a - hi), min(max_step, b - lo)
                if start <= stop:
                    blocked[start:stop + 1] = True
        valid = np.flatnonzero(~blocked)
        return int(valid[0]) if valid.size else None


def shifted(base, step, network, policy):
    delay = step * network.time_slot_sec
    labels = [x for x in base.strategy_labels if x not in ("unchanged", "ground-delay")]
    if delay > ref.EPS:
        labels.append("ground-delay")
    schedule = replace(
        base, scheduled_start_sec=base.scheduled_start_sec + delay, delay_sec=delay,
        finish_sec=base.finish_sec + delay,
        segments=[replace(s, start_sec=s.start_sec + delay, end_sec=s.end_sec + delay)
                  for s in base.segments],
        resources={key[:-1] + (key[-1] + step,) for key in base.resources},
        strategy_labels=tuple(labels or ["unchanged"]),
    )
    return replace(schedule, selection_cost=ref._candidate_policy_cost(
        schedule, schedule.priority, policy))


def signature(schedule):
    return (schedule.delay_sec, schedule.selection_cost, schedule.finish_sec,
            schedule.route_index, schedule.layer_plan, schedule.lane_plan)


def schedule_flights(flights, network, uav_types, policy=None, *, neutral_ties=False,
                     candidate_builder=ref.build_schedule_candidate, trace=None):
    policy = policy or ref.SchedulingPolicy()
    start = time.perf_counter()
    templates = {}
    for f in flights:
        templates[f.flight_id] = []
        for choice in ref.enumerate_structural_choices(f, network):
            base = candidate_builder(f, choice, 0, network, uav_types)
            templates[f.flight_id].append((base, resource_intervals(base.resources)))
    index = IntervalIndex()
    schedules, failures, history = {}, [], []
    attempts = 0
    dispatch_key = (lambda f: (f.requested_start_sec, f.flight_id)) if neutral_ties else (
        lambda f: ref._dispatch_key(f, policy))
    max_steps = int(math.floor((network.max_delay_sec + ref.EPS) / network.time_slot_sec))
    for f in sorted(flights, key=dispatch_key):
        best, best_key = None, None
        for base, intervals in templates[f.flight_id]:
            step = index.earliest(intervals, max_steps)
            attempts += max_steps + 1 if step is None else step + 1
            if step is None:
                continue
            candidate = shifted(base, step, network, policy)
            key = (candidate.selection_cost, candidate.delay_sec, candidate.finish_sec,
                   candidate.route_index, candidate.layer_plan, candidate.lane_plan)
            if best_key is None or key < best_key:
                best, best_key = candidate, key
        if best is None:
            failures.append(f.flight_id)
        else:
            schedules[f.flight_id] = best
            index.reserve(best)
    construction_time = time.perf_counter() - start
    construction_delay = sum(s.delay_sec for s in schedules.values())
    if trace is not None:
        from final_model import signature as schedule_fingerprint
        trace['construction_signature'] = schedule_fingerprint(schedules)
        trace['construction_delays'] = {fid: s.delay_sec for fid, s in schedules.items()}
        trace['updates'] = []
    improvement_start = time.perf_counter()
    if not failures:
        for pass_no in range(1, policy.local_reinsert_passes + 1):
            before = sum(s.delay_sec for s in schedules.values())
            checks, changed = 0, 0
            if neutral_ties:
                order = sorted(schedules.values(), key=lambda s: (-s.delay_sec, s.flight_id))
            else:
                order = sorted(schedules.values(), key=lambda s: (
                    -policy.delay_weights[s.priority] * s.delay_sec, s.priority, s.flight_id))
            for current in order:
                index.release(current)
                selected, selected_key = current, signature(current)
                limit = int(math.floor((current.delay_sec + ref.EPS) / network.time_slot_sec))
                for base, intervals in templates[current.flight_id]:
                    step = index.earliest(intervals, limit)
                    checks += limit + 1 if step is None else step + 1
                    if step is None:
                        continue
                    candidate = shifted(base, step, network, policy)
                    key = signature(candidate)
                    if key < selected_key:
                        selected, selected_key = candidate, key
                changed += selected is not current
                if trace is not None and selected is not current:
                    assert selected.delay_sec <= current.delay_sec + ref.EPS
                    trace['updates'].append({'pass': pass_no, 'id': current.flight_id,
                        'before_sec': current.delay_sec, 'after_sec': selected.delay_sec,
                        'structural_cost_before': current.structural_cost,
                        'structural_cost_after': selected.structural_cost})
                schedules[current.flight_id] = selected
                index.reserve(selected)
            after = sum(s.delay_sec for s in schedules.values())
            assert after <= before + ref.EPS
            history.append(ref.ImprovementPass(pass_no, before, after, changed, checks))
            attempts += checks
            if changed == 0:
                break
    improvement_time = time.perf_counter() - improvement_start
    conflicts = ref.audit_reservation_conflicts(schedules)
    return ref.SchedulingResult(
        success=not failures and not conflicts, schedules=schedules,
        unscheduled_flights=failures, runtime_sec=time.perf_counter() - start,
        reservation_count=len(set().union(*(s.resources for s in schedules.values()))) if schedules else 0,
        search_attempts=attempts, message="Indexed grid scheduling",
        construction_runtime_sec=construction_time, improvement_runtime_sec=improvement_time,
        construction_total_delay_sec=construction_delay, improvement_history=history,
        policy_config={"dispatch_rule": policy.dispatch_rule,
                       "neutral_ties": neutral_ties,
                       "delay_weights": dict(policy.delay_weights),
                       "local_reinsert_passes": policy.local_reinsert_passes},
    )
