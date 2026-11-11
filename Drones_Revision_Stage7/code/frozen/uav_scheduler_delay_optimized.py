#!/usr/bin/env python3
"""Priority-aware spatiotemporal reservation scheduler for a layered UAV network.

The paper-default algorithm is Strict Priority Construction with Weighted Safe
Reinsertion (SPC-WSR):

1. Stage I processes requests in strict mission-priority order and constructs an
   auditable reservation-consistent schedule.
2. Stage II repeatedly removes delayed flights and safely reinserts them into
   earlier feasible gaps without increasing any individual flight's delay.
3. Each flight enumerates only physically and topologically legal candidates
   (route, per-edge layer plan, physical lane plan and optional upstream speed
   metering).
4. A candidate is committed only if every edge, junction and vertical-transition
   reservation is free; failure is explicit.
5. All policy parameters, before/after delay statistics and audit results are
   exported to CSV/JSON for reproducibility.

FCFS and aging dispatch modes remain in the implementation only for controlled
baseline and fairness experiments; they are not the paper-default method.

The model is a strategic network scheduler, not a full six-degree-of-freedom
flight-dynamics simulator. The sampled trajectory audit is an independent replay check at the configured
sampling interval; it is not a continuous-time formal safety proof.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import random
import statistics
import sys
import time
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping, Sequence

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError as exc:  # pragma: no cover - checked at startup
    raise SystemExit(
        "This program requires numpy and matplotlib. Install them with: "
        "python -m pip install numpy matplotlib"
    ) from exc


EPS = 1e-9

SOFTWARE_NAME = "Layered UAV Strategic Reservation Scheduler"
SOFTWARE_VERSION = "0.1.0-exp0"
ALGORITHM_ID = "SPC-WSR"
ALGORITHM_NAME = "Strict Priority Construction with Weighted Safe Reinsertion"
OUTPUT_SCHEMA_VERSION = "1.0.0"


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class UAVType:
    name: str
    cruise_factor: float
    vertical_speed_mps: float
    horizontal_separation_m: float
    vertical_separation_m: float


@dataclass(frozen=True)
class EdgeConfig:
    edge_id: str
    start_node: str
    end_node: str
    allowed_layers: tuple[int, ...]
    base_duration_sec: float
    lane_offsets_m: tuple[float, ...]
    geometry: str = "line"  # "line" or "quadratic_bezier"
    control_point_xy: tuple[float, float] | None = None

    @property
    def lane_count(self) -> int:
        return len(self.lane_offsets_m)


@dataclass(frozen=True)
class NetworkConfig:
    time_slot_sec: float
    layer_altitude_m: Mapping[int, float]
    nodes_xy: Mapping[str, tuple[float, float]]
    edges: Mapping[str, EdgeConfig]
    edge_headway_sec: float
    node_buffer_before_sec: float
    node_buffer_after_sec: float
    vertical_buffer_sec: float
    trajectory_sample_sec: float
    max_delay_sec: float


@dataclass(frozen=True)
class FlightPlan:
    flight_id: str
    priority: int  # smaller number = higher priority
    requested_start_sec: float
    origin_group: str
    candidate_paths: tuple[tuple[str, ...], ...]
    uav_type: str
    default_layer: int = 1


@dataclass(frozen=True)
class Segment:
    kind: str  # edge or vertical
    segment_id: str
    start_sec: float
    end_sec: float
    edge_id: str | None = None
    node_id: str | None = None
    layer_from: int | None = None
    layer_to: int | None = None
    lane: int | None = None

    @property
    def duration_sec(self) -> float:
        return self.end_sec - self.start_sec


ResourceKey = tuple[str, str, int, int, int]
# (resource_kind, resource_name, layer_or_layer_code, lane, time_slot)


@dataclass(frozen=True)
class StructuralChoice:
    route_index: int
    route: tuple[str, ...]
    layer_plan: tuple[int, ...]
    lane_plan: tuple[int, ...]
    speed_mode: str
    structural_cost: float
    strategy_labels: tuple[str, ...]


@dataclass
class FlightSchedule:
    flight_id: str
    priority: int
    requested_start_sec: float
    scheduled_start_sec: float
    delay_sec: float
    route_index: int
    route: tuple[str, ...]
    layer_plan: tuple[int, ...]
    lane_plan: tuple[int, ...]
    speed_mode: str
    structural_cost: float
    selection_cost: float
    strategy_labels: tuple[str, ...]
    finish_sec: float
    segments: list[Segment]
    resources: set[ResourceKey]


@dataclass
class SchedulingResult:
    success: bool
    schedules: dict[str, FlightSchedule]
    unscheduled_flights: list[str]
    runtime_sec: float
    reservation_count: int
    search_attempts: int
    message: str
    construction_runtime_sec: float = 0.0
    improvement_runtime_sec: float = 0.0
    construction_total_delay_sec: float = 0.0
    improvement_history: list[ImprovementPass] = field(default_factory=list)
    policy_config: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class SafetyEvent:
    time_sec: float
    flight_a: str
    flight_b: str
    horizontal_m: float
    vertical_m: float
    distance_3d_m: float


@dataclass
class SafetyReport:
    passed: bool
    sample_dt_sec: float
    horizontal_threshold_m: float
    vertical_threshold_m: float
    minimum_horizontal_m: float | None
    minimum_vertical_m: float | None
    minimum_3d_m: float | None
    loss_of_separation_samples: int
    unique_loss_pairs: int
    events: list[SafetyEvent] = field(default_factory=list)


@dataclass(frozen=True)
class SchedulingPolicy:
    """Configuration for construction ordering and delay improvement.

    ``aging_offsets_sec`` implements priority aging through a virtual release
    time ``requested_start + offset(priority)``. Smaller offsets receive earlier
    service; a sufficiently old lower-priority request can therefore overtake a
    later higher-priority request.

    ``delay_weights`` affect candidate selection and the order in which delayed
    flights receive access to gaps during local reinsertion. The local search
    never accepts a larger delay for any individual flight, so total unweighted
    delay is monotonically non-increasing.
    """

    name: str = "spc-wsr"
    dispatch_rule: str = "strict"  # strict, fcfs, or aging
    aging_offsets_sec: Mapping[int, float] = field(
        default_factory=lambda: {2: 0.0, 5: 30.0, 10: 60.0}
    )
    delay_weights: Mapping[int, float] = field(
        default_factory=lambda: {2: 3.0, 5: 2.0, 10: 1.0}
    )
    air_time_weight: float = 0.05
    weight_candidate_delay: bool = False
    local_reinsert_passes: int = 5


@dataclass(frozen=True)
class ImprovementPass:
    pass_index: int
    total_delay_before_sec: float
    total_delay_after_sec: float
    improved_flights: int
    candidate_checks: int


# ---------------------------------------------------------------------------
# Default scenario/network
# ---------------------------------------------------------------------------


def default_uav_types() -> dict[str, UAVType]:
    return {
        "rescue": UAVType(
            name="rescue",
            cruise_factor=1.10,
            vertical_speed_mps=25.0,
            horizontal_separation_m=20.0,
            vertical_separation_m=30.0,
        ),
        "cargo": UAVType(
            name="cargo",
            cruise_factor=0.95,
            vertical_speed_mps=18.0,
            horizontal_separation_m=25.0,
            vertical_separation_m=35.0,
        ),
        "inspection": UAVType(
            name="inspection",
            cruise_factor=0.85,
            vertical_speed_mps=15.0,
            horizontal_separation_m=20.0,
            vertical_separation_m=30.0,
        ),
    }


def default_network(sample_dt: float = 0.25, max_delay_sec: float = 1800.0) -> NetworkConfig:
    nodes = {
        "O1": (0.0, 150.0),
        "O2": (0.0, -150.0),
        "C": (400.0, 0.0),
        "D": (1000.0, 0.0),
    }
    edges = {
        "E1": EdgeConfig(
            edge_id="E1",
            start_node="O1",
            end_node="C",
            allowed_layers=(1, 2),
            base_duration_sec=10.0,
            lane_offsets_m=(0.0,),
        ),
        "E2": EdgeConfig(
            edge_id="E2",
            start_node="O2",
            end_node="C",
            allowed_layers=(1, 2),
            base_duration_sec=10.0,
            lane_offsets_m=(0.0,),
        ),
        "E3": EdgeConfig(
            edge_id="E3",
            start_node="C",
            end_node="D",
            allowed_layers=(1, 2),
            base_duration_sec=10.0,
            lane_offsets_m=(0.0,),
        ),
        # E4's capacity of two is represented by two physically separated lanes,
        # rather than by allowing two UAVs on one identical geometric curve.
        "E4": EdgeConfig(
            edge_id="E4",
            start_node="C",
            end_node="D",
            allowed_layers=(1,),
            base_duration_sec=14.0,
            lane_offsets_m=(-30.0, 30.0),
            geometry="quadratic_bezier",
            control_point_xy=(700.0, 240.0),
        ),
    }
    return NetworkConfig(
        time_slot_sec=2.0,
        layer_altitude_m={1: 100.0, 2: 200.0},
        nodes_xy=nodes,
        edges=edges,
        edge_headway_sec=2.0,
        node_buffer_before_sec=2.0,
        node_buffer_after_sec=2.0,
        vertical_buffer_sec=2.0,
        trajectory_sample_sec=sample_dt,
        max_delay_sec=max_delay_sec,
    )


def generate_scenario(
    num_flights: int,
    seed: int,
    start_window_sec: int,
) -> list[FlightPlan]:
    """Generate a deterministic heterogeneous scenario.

    The route structure intentionally follows the uploaded prototype: flights from
    O1 use E1-E3; flights from O2 may use E2-E3 or the E2-E4 detour.
    """

    rng = random.Random(seed)
    plans: list[FlightPlan] = []
    type_names = ("rescue", "cargo", "inspection")

    for index in range(1, num_flights + 1):
        requested = float(rng.randint(0, start_window_sec))
        priority = rng.choice((2, 5, 10))
        # Correlate high-priority missions with rescue vehicles only probabilistically;
        # priority remains a mission attribute, not a synonym for vehicle type.
        if priority == 2 and rng.random() < 0.65:
            uav_type = "rescue"
        else:
            uav_type = rng.choice(type_names)

        if rng.random() < 0.5:
            origin = "O1"
            paths = (("E1", "E3"),)
        else:
            origin = "O2"
            paths = (("E2", "E3"), ("E2", "E4"))

        plans.append(
            FlightPlan(
                flight_id=f"F{index:03d}",
                priority=priority,
                requested_start_sec=requested,
                origin_group=origin,
                candidate_paths=paths,
                uav_type=uav_type,
                default_layer=1,
            )
        )

    # The returned list is stable and useful for scenario CSVs. Scheduling order is
    # applied separately so that the source data remain in requested-time order.
    return sorted(plans, key=lambda f: (f.requested_start_sec, f.flight_id))


def load_scenario_csv(path: Path) -> list[FlightPlan]:
    """Load the exact scenario format exported by :func:`export_all`.

    Loading a previously exported scenario closes the reproducibility loop: the
    scheduler can rerun the same flight IDs, mission priorities, requested start
    times, UAV types and candidate routes without invoking the random generator.
    """

    if not path.is_file():
        raise FileNotFoundError(f"Scenario CSV not found: {path}")
    plans: list[FlightPlan] = []
    seen_ids: set[str] = set()
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        required = {
            "id",
            "priority",
            "requested_start_sec",
            "origin_group",
            "uav_type",
            "default_layer",
            "candidate_paths",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                "Scenario CSV is missing required column(s): "
                + ", ".join(sorted(missing))
            )
        for row_number, row in enumerate(reader, start=2):
            flight_id = str(row["id"]).strip()
            if not flight_id:
                raise ValueError(f"Empty flight ID at row {row_number}")
            if flight_id in seen_ids:
                raise ValueError(f"Duplicate flight ID {flight_id!r} at row {row_number}")
            seen_ids.add(flight_id)
            try:
                raw_paths = json.loads(row["candidate_paths"])
                if not isinstance(raw_paths, list) or not all(
                    isinstance(route, (list, tuple)) for route in raw_paths
                ):
                    raise ValueError("candidate_paths must be a JSON list of route lists")
                candidate_paths = tuple(tuple(map(str, route)) for route in raw_paths)
                if not candidate_paths or any(not route for route in candidate_paths):
                    raise ValueError("candidate_paths must contain non-empty routes")
                plans.append(
                    FlightPlan(
                        flight_id=flight_id,
                        priority=int(row["priority"]),
                        requested_start_sec=float(row["requested_start_sec"]),
                        origin_group=str(row["origin_group"]),
                        candidate_paths=candidate_paths,
                        uav_type=str(row["uav_type"]),
                        default_layer=int(row["default_layer"]),
                    )
                )
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                raise ValueError(f"Invalid scenario row {row_number}: {exc}") from exc
    if not plans:
        raise ValueError("Scenario CSV contains no flight rows")
    return sorted(plans, key=lambda f: (f.requested_start_sec, f.flight_id))


# ---------------------------------------------------------------------------
# Time/resource mapping
# ---------------------------------------------------------------------------


def round_up_to_step(value: float, step: float) -> float:
    return math.ceil((value - EPS) / step) * step


def interval_slots(
    start_sec: float,
    end_sec: float,
    slot_sec: float,
    *,
    before_sec: float = 0.0,
    after_sec: float = 0.0,
) -> range:
    """Conservatively map a half-open continuous interval to discrete slots.

    The correct upper endpoint uses ceil(end / dt) - 1. This avoids the missed-slot
    error caused by floor(end / dt) - 1 when end is not exactly on a slot boundary.
    """

    if end_sec < start_sec - EPS:
        raise ValueError(f"Invalid interval [{start_sec}, {end_sec})")
    expanded_start = start_sec - before_sec
    expanded_end = end_sec + after_sec
    first = math.floor((expanded_start + EPS) / slot_sec)
    last = math.ceil((expanded_end - EPS) / slot_sec) - 1
    if last < first:
        last = first
    return range(first, last + 1)


def edge_duration_sec(
    edge: EdgeConfig,
    uav_type: UAVType,
    network: NetworkConfig,
    *,
    metered: bool,
) -> float:
    speed_factor = uav_type.cruise_factor * (0.82 if metered else 1.0)
    raw = edge.base_duration_sec / speed_factor
    return max(network.time_slot_sec, round_up_to_step(raw, network.time_slot_sec))


def vertical_duration_sec(
    layer_from: int,
    layer_to: int,
    uav_type: UAVType,
    network: NetworkConfig,
) -> float:
    dz = abs(network.layer_altitude_m[layer_to] - network.layer_altitude_m[layer_from])
    raw = dz / uav_type.vertical_speed_mps
    return max(network.time_slot_sec, round_up_to_step(raw, network.time_slot_sec))


class ReservationTable:
    """Capacity-one reservation table over explicit edge lanes and node resources."""

    def __init__(self) -> None:
        self._owner_by_key: dict[ResourceKey, str] = {}
        self._keys_by_flight: dict[str, set[ResourceKey]] = defaultdict(set)

    def is_free(self, keys: Iterable[ResourceKey]) -> bool:
        return all(key not in self._owner_by_key for key in keys)

    def reserve(self, flight_id: str, keys: set[ResourceKey]) -> None:
        collisions = {key: self._owner_by_key[key] for key in keys if key in self._owner_by_key}
        if collisions:
            sample = next(iter(collisions.items()))
            raise ValueError(
                f"Reservation collision while committing {flight_id}: "
                f"{sample[0]} already owned by {sample[1]}"
            )
        for key in keys:
            self._owner_by_key[key] = flight_id
        self._keys_by_flight[flight_id].update(keys)

    @property
    def reservation_count(self) -> int:
        return len(self._owner_by_key)

    @property
    def owner_by_key(self) -> Mapping[ResourceKey, str]:
        return self._owner_by_key


# ---------------------------------------------------------------------------
# Candidate construction and scheduling
# ---------------------------------------------------------------------------


def validate_route_topology(route: Sequence[str], network: NetworkConfig) -> None:
    if not route:
        raise ValueError("A route cannot be empty")
    for edge_id in route:
        if edge_id not in network.edges:
            raise ValueError(f"Unknown edge {edge_id}")
    for first, second in zip(route, route[1:]):
        if network.edges[first].end_node != network.edges[second].start_node:
            raise ValueError(f"Disconnected route transition {first} -> {second}")


def enumerate_structural_choices(
    flight: FlightPlan,
    network: NetworkConfig,
) -> list[StructuralChoice]:
    """Enumerate legal route/layer/lane/speed combinations in deterministic order."""

    choices: list[StructuralChoice] = []
    for route_index, route in enumerate(flight.candidate_paths):
        validate_route_topology(route, network)
        allowed_layers = [network.edges[e].allowed_layers for e in route]
        lane_ranges = [range(network.edges[e].lane_count) for e in route]

        for layer_plan in itertools.product(*allowed_layers):
            for lane_plan in itertools.product(*lane_ranges):
                for speed_mode in ("nominal", "metered"):
                    speed_adjusted = speed_mode == "metered"
                    rerouted = route_index > 0
                    # A persistent non-default layer assignment is one strategic
                    # action, not one action per edge. A change between adjacent
                    # edges is an additional explicit vertical transition.
                    nondefault_layer_assignment = int(
                        any(layer != flight.default_layer for layer in layer_plan)
                    )
                    transition_count = sum(
                        1 for a, b in zip(layer_plan, layer_plan[1:]) if a != b
                    )
                    altitude_action_count = (
                        nondefault_layer_assignment + transition_count
                    )

                    labels: list[str] = []
                    if speed_adjusted:
                        labels.append("upstream-speed-metering")
                    if altitude_action_count:
                        labels.append("altitude-assignment/transition")
                    if rerouted:
                        labels.append("reroute")
                    if not labels:
                        labels.append("unchanged")

                    # Explicit, auditable structural-cost weights. They are expressed
                    # in delay-equivalent seconds and are combined with actual ground
                    # delay by the scheduler. This makes every trade-off reproducible.
                    structural_cost = (
                        3.0 * int(speed_adjusted)
                        + 8.0 * altitude_action_count
                        + 30.0 * int(rerouted)
                    )
                    choices.append(
                        StructuralChoice(
                            route_index=route_index,
                            route=tuple(route),
                            layer_plan=tuple(layer_plan),
                            lane_plan=tuple(lane_plan),
                            speed_mode=speed_mode,
                            structural_cost=structural_cost,
                            strategy_labels=tuple(labels),
                        )
                    )

    return sorted(
        choices,
        key=lambda c: (
            c.structural_cost,
            c.route_index,
            c.layer_plan,
            c.lane_plan,
            c.speed_mode,
        ),
    )


def _edge_resource_keys(
    edge_id: str,
    layer: int,
    lane: int,
    start_sec: float,
    end_sec: float,
    network: NetworkConfig,
) -> set[ResourceKey]:
    return {
        ("EDGE", edge_id, layer, lane, slot)
        for slot in interval_slots(
            start_sec,
            end_sec,
            network.time_slot_sec,
            after_sec=network.edge_headway_sec,
        )
    }


def _node_resource_keys(
    node_id: str,
    layers: Iterable[int],
    start_sec: float,
    end_sec: float,
    network: NetworkConfig,
) -> set[ResourceKey]:
    keys: set[ResourceKey] = set()
    for layer in set(layers):
        for slot in interval_slots(
            start_sec,
            end_sec,
            network.time_slot_sec,
            before_sec=network.node_buffer_before_sec,
            after_sec=network.node_buffer_after_sec,
        ):
            keys.add(("NODE", node_id, layer, 0, slot))
    return keys


def _vertical_resource_keys(
    node_id: str,
    layer_from: int,
    layer_to: int,
    start_sec: float,
    end_sec: float,
    network: NetworkConfig,
) -> set[ResourceKey]:
    layer_code = min(layer_from, layer_to) * 100 + max(layer_from, layer_to)
    return {
        ("VERTICAL", node_id, layer_code, 0, slot)
        for slot in interval_slots(
            start_sec,
            end_sec,
            network.time_slot_sec,
            before_sec=network.vertical_buffer_sec,
            after_sec=network.vertical_buffer_sec,
        )
    }


def build_schedule_candidate(
    flight: FlightPlan,
    choice: StructuralChoice,
    delay_sec: float,
    network: NetworkConfig,
    uav_types: Mapping[str, UAVType],
) -> FlightSchedule:
    if delay_sec < -EPS:
        raise ValueError("Delay cannot be negative")
    uav = uav_types[flight.uav_type]
    current_time = flight.requested_start_sec + delay_sec
    scheduled_start = current_time
    segments: list[Segment] = []
    resources: set[ResourceKey] = set()

    for index, edge_id in enumerate(choice.route):
        edge = network.edges[edge_id]
        layer = choice.layer_plan[index]
        lane = choice.lane_plan[index]
        if layer not in edge.allowed_layers:
            raise ValueError(f"Illegal layer {layer} on {edge_id}")
        if lane < 0 or lane >= edge.lane_count:
            raise ValueError(f"Illegal lane {lane} on {edge_id}")

        duration = edge_duration_sec(
            edge,
            uav,
            network,
            # Metering is applied only on the upstream/first edge; slowing inside
            # the bottleneck itself would increase bottleneck occupancy.
            metered=(choice.speed_mode == "metered" and index == 0),
        )
        edge_start = current_time
        edge_end = current_time + duration
        segment_id = f"{flight.flight_id}:{edge_id}:{index}"
        segments.append(
            Segment(
                kind="edge",
                segment_id=segment_id,
                start_sec=edge_start,
                end_sec=edge_end,
                edge_id=edge_id,
                node_id=None,
                layer_from=layer,
                layer_to=layer,
                lane=lane,
            )
        )
        resources.update(
            _edge_resource_keys(edge_id, layer, lane, edge_start, edge_end, network)
        )
        current_time = edge_end

        if index < len(choice.route) - 1:
            next_edge_id = choice.route[index + 1]
            next_edge = network.edges[next_edge_id]
            if edge.end_node != next_edge.start_node:
                raise ValueError(f"Route discontinuity {edge_id} -> {next_edge_id}")
            node_id = edge.end_node
            next_layer = choice.layer_plan[index + 1]
            node_start = current_time
            node_end = current_time

            if next_layer != layer:
                transition_duration = vertical_duration_sec(layer, next_layer, uav, network)
                node_end = current_time + transition_duration
                segments.append(
                    Segment(
                        kind="vertical",
                        segment_id=f"{flight.flight_id}:VERT:{node_id}:{index}",
                        start_sec=current_time,
                        end_sec=node_end,
                        node_id=node_id,
                        layer_from=layer,
                        layer_to=next_layer,
                        lane=0,
                    )
                )
                resources.update(
                    _vertical_resource_keys(
                        node_id,
                        layer,
                        next_layer,
                        current_time,
                        node_end,
                        network,
                    )
                )
                current_time = node_end

            resources.update(
                _node_resource_keys(
                    node_id,
                    (layer, next_layer),
                    node_start,
                    node_end,
                    network,
                )
            )

    final_edge = network.edges[choice.route[-1]]
    final_layer = choice.layer_plan[-1]
    resources.update(
        _node_resource_keys(
            final_edge.end_node,
            (final_layer,),
            current_time,
            current_time,
            network,
        )
    )

    # "unchanged" means no action at all. Do not keep that label when a
    # ground delay is subsequently applied; strategy counts remain interpretable.
    labels = [label for label in choice.strategy_labels if label != "unchanged"]
    if delay_sec > EPS:
        labels.append("ground-delay")
    if not labels:
        labels.append("unchanged")
    return FlightSchedule(
        flight_id=flight.flight_id,
        priority=flight.priority,
        requested_start_sec=flight.requested_start_sec,
        scheduled_start_sec=scheduled_start,
        delay_sec=delay_sec,
        route_index=choice.route_index,
        route=choice.route,
        layer_plan=choice.layer_plan,
        lane_plan=choice.lane_plan,
        speed_mode=choice.speed_mode,
        structural_cost=choice.structural_cost,
        selection_cost=(
            delay_sec
            + choice.structural_cost
            + 0.05 * (current_time - scheduled_start)
        ),
        strategy_labels=tuple(labels),
        finish_sec=current_time,
        segments=segments,
        resources=resources,
    )


def _policy_value(mapping: Mapping[int, float], priority: int, label: str) -> float:
    try:
        value = float(mapping[priority])
    except KeyError as exc:
        raise ValueError(f"Missing {label} for mission priority {priority}") from exc
    if not math.isfinite(value):
        raise ValueError(f"Non-finite {label} for mission priority {priority}")
    return value


def _dispatch_key(flight: FlightPlan, policy: SchedulingPolicy) -> tuple[Any, ...]:
    if policy.dispatch_rule == "strict":
        return (flight.priority, flight.requested_start_sec, flight.flight_id)
    if policy.dispatch_rule == "fcfs":
        return (flight.requested_start_sec, flight.priority, flight.flight_id)
    if policy.dispatch_rule == "aging":
        offset = _policy_value(
            policy.aging_offsets_sec, flight.priority, "aging offset"
        )
        return (
            flight.requested_start_sec + offset,
            flight.requested_start_sec,
            flight.priority,
            flight.flight_id,
        )
    raise ValueError(
        f"Unsupported dispatch_rule {policy.dispatch_rule!r}; "
        "expected 'strict', 'fcfs', or 'aging'"
    )


def _candidate_policy_cost(
    candidate: FlightSchedule,
    priority: int,
    policy: SchedulingPolicy,
) -> float:
    delay_weight = (
        _policy_value(policy.delay_weights, priority, "delay weight")
        if policy.weight_candidate_delay
        else 1.0
    )
    airborne_sec = candidate.finish_sec - candidate.scheduled_start_sec
    return (
        delay_weight * candidate.delay_sec
        + candidate.structural_cost
        + policy.air_time_weight * airborne_sec
    )


def _best_feasible_candidate(
    flight: FlightPlan,
    occupied_keys: set[ResourceKey] | Mapping[ResourceKey, str],
    network: NetworkConfig,
    uav_types: Mapping[str, UAVType],
    policy: SchedulingPolicy,
    *,
    maximum_delay_sec: float | None = None,
) -> tuple[FlightSchedule | None, int]:
    """Find the best earliest-feasible structural candidate for one flight."""

    limit = network.max_delay_sec if maximum_delay_sec is None else min(
        network.max_delay_sec, max(0.0, maximum_delay_sec)
    )
    max_delay_steps = int(math.floor((limit + EPS) / network.time_slot_sec))
    best: FlightSchedule | None = None
    best_key: tuple[Any, ...] | None = None
    attempts = 0

    for choice in enumerate_structural_choices(flight, network):
        earliest_for_choice: FlightSchedule | None = None
        for delay_step in range(max_delay_steps + 1):
            attempts += 1
            candidate = build_schedule_candidate(
                flight,
                choice,
                delay_step * network.time_slot_sec,
                network,
                uav_types,
            )
            if all(key not in occupied_keys for key in candidate.resources):
                policy_cost = _candidate_policy_cost(candidate, flight.priority, policy)
                earliest_for_choice = replace(
                    candidate,
                    selection_cost=policy_cost,
                )
                break

        if earliest_for_choice is None:
            continue
        candidate_key = (
            earliest_for_choice.selection_cost,
            earliest_for_choice.delay_sec,
            earliest_for_choice.finish_sec,
            earliest_for_choice.route_index,
            earliest_for_choice.layer_plan,
            earliest_for_choice.lane_plan,
        )
        if best_key is None or candidate_key < best_key:
            best = earliest_for_choice
            best_key = candidate_key

    return best, attempts


def improve_schedule_by_weighted_reinsertion(
    flights: Sequence[FlightPlan],
    schedules: Mapping[str, FlightSchedule],
    network: NetworkConfig,
    uav_types: Mapping[str, UAVType],
    policy: SchedulingPolicy,
) -> tuple[dict[str, FlightSchedule], list[ImprovementPass], int]:
    """Reduce delay by safe weighted local reinsertion.

    Flights with the largest weighted waiting pressure ``w_p * delay`` are
    reconsidered first. A flight is temporarily removed, all legal structures are
    searched up to its current delay, and an earlier feasible insertion is accepted.
    The acceptance key is lexicographic in actual delay, weighted policy cost and
    finish time. Consequently no individual delay can increase and total ground
    delay is monotone non-increasing across passes.
    """

    if policy.local_reinsert_passes <= 0 or not schedules:
        return dict(schedules), [], 0

    flight_by_id = {flight.flight_id: flight for flight in flights}
    improved = dict(schedules)
    occupied: set[ResourceKey] = set().union(
        *(schedule.resources for schedule in improved.values())
    )
    history: list[ImprovementPass] = []
    total_attempts = 0

    for pass_index in range(1, policy.local_reinsert_passes + 1):
        delay_before = sum(schedule.delay_sec for schedule in improved.values())
        pass_attempts = 0
        changed = 0
        ordered = sorted(
            improved.values(),
            key=lambda schedule: (
                -(
                    _policy_value(
                        policy.delay_weights,
                        schedule.priority,
                        "delay weight",
                    )
                    * schedule.delay_sec
                ),
                schedule.priority,
                schedule.flight_id,
            ),
        )

        for current in ordered:
            flight = flight_by_id[current.flight_id]
            occupied.difference_update(current.resources)
            selected = current
            selected_key = (
                current.delay_sec,
                _candidate_policy_cost(current, current.priority, policy),
                current.finish_sec,
                current.route_index,
                current.layer_plan,
                current.lane_plan,
            )
            max_delay_steps = int(
                math.floor((current.delay_sec + EPS) / network.time_slot_sec)
            )
            for choice in enumerate_structural_choices(flight, network):
                for delay_step in range(max_delay_steps + 1):
                    pass_attempts += 1
                    candidate = build_schedule_candidate(
                        flight,
                        choice,
                        delay_step * network.time_slot_sec,
                        network,
                        uav_types,
                    )
                    if not candidate.resources.isdisjoint(occupied):
                        continue
                    candidate = replace(
                        candidate,
                        selection_cost=_candidate_policy_cost(
                            candidate, current.priority, policy
                        ),
                    )
                    candidate_key = (
                        candidate.delay_sec,
                        candidate.selection_cost,
                        candidate.finish_sec,
                        candidate.route_index,
                        candidate.layer_plan,
                        candidate.lane_plan,
                    )
                    if candidate_key < selected_key:
                        selected = candidate
                        selected_key = candidate_key
                    # Later delays cannot improve this structural choice because
                    # delay is the first reinsertion objective component.
                    break

            if selected is not current:
                changed += 1

            improved[current.flight_id] = selected
            occupied.update(selected.resources)

        total_attempts += pass_attempts
        delay_after = sum(schedule.delay_sec for schedule in improved.values())
        if delay_after > delay_before + EPS:
            raise AssertionError(
                "Weighted reinsertion increased total delay, violating monotonicity"
            )
        history.append(
            ImprovementPass(
                pass_index=pass_index,
                total_delay_before_sec=delay_before,
                total_delay_after_sec=delay_after,
                improved_flights=changed,
                candidate_checks=pass_attempts,
            )
        )
        if changed == 0:
            break

    return improved, history, total_attempts


def schedule_flights(
    flights: Sequence[FlightPlan],
    network: NetworkConfig,
    uav_types: Mapping[str, UAVType],
    policy: SchedulingPolicy | None = None,
) -> SchedulingResult:
    """Construct and locally improve a reservation-consistent schedule.

    The paper-default policy is strict-priority construction followed by weighted
    safe reinsertion. It remains a deterministic heuristic, not a globally
    optimal solver.
    """

    policy = policy or SchedulingPolicy()
    start_clock = time.perf_counter()
    construction_start = start_clock
    reservations = ReservationTable()
    schedules: dict[str, FlightSchedule] = {}
    unscheduled: list[str] = []
    attempts = 0

    ordered = sorted(flights, key=lambda flight: _dispatch_key(flight, policy))

    for flight in ordered:
        best_candidate, candidate_attempts = _best_feasible_candidate(
            flight,
            reservations.owner_by_key,
            network,
            uav_types,
            policy,
        )
        attempts += candidate_attempts
        if best_candidate is None:
            unscheduled.append(flight.flight_id)
            continue
        reservations.reserve(flight.flight_id, best_candidate.resources)
        schedules[flight.flight_id] = best_candidate

    construction_runtime = time.perf_counter() - construction_start
    construction_total_delay = sum(
        schedule.delay_sec for schedule in schedules.values()
    )

    improvement_start = time.perf_counter()
    improvement_history: list[ImprovementPass] = []
    if not unscheduled and len(schedules) == len(flights):
        schedules, improvement_history, improvement_attempts = (
            improve_schedule_by_weighted_reinsertion(
                flights,
                schedules,
                network,
                uav_types,
                policy,
            )
        )
        attempts += improvement_attempts
    improvement_runtime = time.perf_counter() - improvement_start

    final_conflicts = audit_reservation_conflicts(schedules)
    success = (
        not unscheduled
        and len(schedules) == len(flights)
        and not final_conflicts
    )
    message = (
        "All flights scheduled with non-overlapping strategic reservations; "
        "weighted safe reinsertion completed."
        if success
        else f"Scheduling failed for {len(unscheduled)} flight(s) within the delay horizon."
    )
    policy_config = {
        "name": policy.name,
        "dispatch_rule": policy.dispatch_rule,
        "aging_offsets_sec": {
            str(key): float(value) for key, value in policy.aging_offsets_sec.items()
        },
        "delay_weights": {
            str(key): float(value) for key, value in policy.delay_weights.items()
        },
        "air_time_weight": policy.air_time_weight,
        "weight_candidate_delay": policy.weight_candidate_delay,
        "local_reinsert_passes": policy.local_reinsert_passes,
    }
    return SchedulingResult(
        success=success,
        schedules=schedules,
        unscheduled_flights=unscheduled,
        runtime_sec=time.perf_counter() - start_clock,
        reservation_count=len(set().union(*(s.resources for s in schedules.values())))
        if schedules
        else 0,
        search_attempts=attempts,
        message=message,
        construction_runtime_sec=construction_runtime,
        improvement_runtime_sec=improvement_runtime,
        construction_total_delay_sec=construction_total_delay,
        improvement_history=improvement_history,
        policy_config=policy_config,
    )


# ---------------------------------------------------------------------------
# Initial schedule and audits
# ---------------------------------------------------------------------------


def build_initial_schedules(
    flights: Sequence[FlightPlan],
    network: NetworkConfig,
    uav_types: Mapping[str, UAVType],
) -> dict[str, FlightSchedule]:
    schedules: dict[str, FlightSchedule] = {}
    for flight in flights:
        default_route = flight.candidate_paths[0]
        choice = StructuralChoice(
            route_index=0,
            route=default_route,
            layer_plan=tuple(flight.default_layer for _ in default_route),
            lane_plan=tuple(0 for _ in default_route),
            speed_mode="nominal",
            structural_cost=0.0,
            strategy_labels=("unchanged",),
        )
        schedules[flight.flight_id] = build_schedule_candidate(
            flight,
            choice,
            0.0,
            network,
            uav_types,
        )
    return schedules


def audit_reservation_conflicts(
    schedules: Mapping[str, FlightSchedule],
) -> dict[ResourceKey, list[str]]:
    owners: dict[ResourceKey, list[str]] = defaultdict(list)
    for schedule in schedules.values():
        for key in schedule.resources:
            owners[key].append(schedule.flight_id)
    return {key: fids for key, fids in owners.items() if len(fids) > 1}


def edge_occupancy_matrix(
    schedules: Mapping[str, FlightSchedule],
    network: NetworkConfig,
) -> tuple[np.ndarray, np.ndarray, list[str], int]:
    """Return actual edge occupancy and capacity matrices (without safety buffers)."""

    if not schedules:
        labels = [
            f"{edge_id}-L{layer}"
            for edge_id, edge in sorted(network.edges.items())
            for layer in edge.allowed_layers
        ]
        return np.zeros((len(labels), 1)), np.ones((len(labels), 1)), labels, 1

    max_finish = max(s.finish_sec for s in schedules.values())
    max_slot = max(1, math.ceil(max_finish / network.time_slot_sec) + 1)
    labels = [
        f"{edge_id}-L{layer}"
        for edge_id, edge in sorted(network.edges.items())
        for layer in edge.allowed_layers
    ]
    row_index = {label: idx for idx, label in enumerate(labels)}
    occupancy = np.zeros((len(labels), max_slot), dtype=float)
    capacity = np.zeros_like(occupancy)

    for edge_id, edge in network.edges.items():
        for layer in edge.allowed_layers:
            capacity[row_index[f"{edge_id}-L{layer}"], :] = edge.lane_count

    for schedule in schedules.values():
        for segment in schedule.segments:
            if segment.kind != "edge" or segment.edge_id is None or segment.layer_from is None:
                continue
            label = f"{segment.edge_id}-L{segment.layer_from}"
            for slot in interval_slots(
                segment.start_sec,
                segment.end_sec,
                network.time_slot_sec,
            ):
                if 0 <= slot < max_slot:
                    occupancy[row_index[label], slot] += 1

    return occupancy, capacity, labels, max_slot


def audit_edge_capacity(
    schedules: Mapping[str, FlightSchedule],
    network: NetworkConfig,
) -> dict[str, Any]:
    occupancy, capacity, labels, _ = edge_occupancy_matrix(schedules, network)
    overload = np.maximum(occupancy - capacity, 0.0)
    overloaded_cells = int(np.count_nonzero(overload > 0))
    total_excess = float(overload.sum())
    max_ratio = float(np.max(np.divide(occupancy, capacity, where=capacity > 0)))
    return {
        "overloaded_edge_layer_slots": overloaded_cells,
        "total_edge_capacity_excess": total_excess,
        "max_edge_utilization_ratio": max_ratio,
        "resource_labels": labels,
    }


# ---------------------------------------------------------------------------
# Geometry, trajectory export and continuous validation
# ---------------------------------------------------------------------------


def _line_point_with_lane_offset(
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    ratio: float,
    lane_offset_m: float,
) -> tuple[float, float]:
    x0, y0 = start_xy
    x1, y1 = end_xy
    x = x0 + ratio * (x1 - x0)
    y = y0 + ratio * (y1 - y0)
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    if length > EPS:
        x += (-dy / length) * lane_offset_m
        y += (dx / length) * lane_offset_m
    return x, y


def _quadratic_bezier_point(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    ratio: float,
    lane_offset_m: float,
) -> tuple[float, float]:
    u = 1.0 - ratio
    x = u * u * p0[0] + 2.0 * u * ratio * p1[0] + ratio * ratio * p2[0]
    y = u * u * p0[1] + 2.0 * u * ratio * p1[1] + ratio * ratio * p2[1]
    # Separate the two E4 lanes away from the common junctions. sin(pi*r)
    # gives zero offset at C/D and maximum physical separation mid-route.
    y += lane_offset_m * math.sin(math.pi * ratio)
    return x, y


def segment_position(
    segment: Segment,
    time_sec: float,
    network: NetworkConfig,
) -> tuple[float, float, float]:
    if time_sec < segment.start_sec - EPS or time_sec > segment.end_sec + EPS:
        raise ValueError("Requested time is outside the segment")
    duration = max(segment.duration_sec, EPS)
    ratio = min(1.0, max(0.0, (time_sec - segment.start_sec) / duration))

    if segment.kind == "vertical":
        if segment.node_id is None or segment.layer_from is None or segment.layer_to is None:
            raise ValueError("Incomplete vertical segment")
        x, y = network.nodes_xy[segment.node_id]
        z0 = network.layer_altitude_m[segment.layer_from]
        z1 = network.layer_altitude_m[segment.layer_to]
        return x, y, z0 + ratio * (z1 - z0)

    if segment.kind != "edge" or segment.edge_id is None:
        raise ValueError(f"Unsupported segment kind {segment.kind}")
    edge = network.edges[segment.edge_id]
    if segment.layer_from is None or segment.lane is None:
        raise ValueError("Incomplete edge segment")
    p0 = network.nodes_xy[edge.start_node]
    p2 = network.nodes_xy[edge.end_node]
    lane_offset = edge.lane_offsets_m[segment.lane]
    if edge.geometry == "line":
        x, y = _line_point_with_lane_offset(p0, p2, ratio, lane_offset)
    elif edge.geometry == "quadratic_bezier":
        if edge.control_point_xy is None:
            raise ValueError(f"Missing control point for {edge.edge_id}")
        x, y = _quadratic_bezier_point(
            p0,
            edge.control_point_xy,
            p2,
            ratio,
            lane_offset,
        )
    else:
        raise ValueError(f"Unknown geometry {edge.geometry}")
    z = network.layer_altitude_m[segment.layer_from]
    return x, y, z


def schedule_position_at(
    schedule: FlightSchedule,
    time_sec: float,
    network: NetworkConfig,
) -> tuple[float, float, float, Segment] | None:
    for segment in schedule.segments:
        # Use half-open intervals except for the final endpoint. This avoids two
        # segments claiming the same sample while preserving the destination row.
        is_final = segment is schedule.segments[-1]
        if segment.start_sec - EPS <= time_sec < segment.end_sec - EPS or (
            is_final and abs(time_sec - segment.end_sec) <= EPS
        ):
            return (*segment_position(segment, time_sec, network), segment)
    return None


def iter_sample_times(start_sec: float, end_sec: float, sample_dt: float) -> Iterator[float]:
    first_index = math.ceil((start_sec - EPS) / sample_dt)
    last_index = math.floor((end_sec + EPS) / sample_dt)
    for index in range(first_index, last_index + 1):
        yield round(index * sample_dt, 10)


def generate_trajectory_rows(
    flights: Sequence[FlightPlan],
    schedules: Mapping[str, FlightSchedule],
    network: NetworkConfig,
    uav_types: Mapping[str, UAVType],
) -> list[dict[str, Any]]:
    flight_by_id = {f.flight_id: f for f in flights}
    rows: list[dict[str, Any]] = []
    for flight_id, schedule in sorted(schedules.items()):
        flight = flight_by_id[flight_id]
        for segment_index, segment in enumerate(schedule.segments):
            for time_sec in iter_sample_times(
                segment.start_sec,
                segment.end_sec,
                network.trajectory_sample_sec,
            ):
                # Prevent duplicate rows at segment boundaries. The preceding
                # segment owns the shared point; adjacent segments are geometrically
                # continuous, so this does not create a position discontinuity.
                if segment_index > 0 and abs(time_sec - segment.start_sec) <= EPS:
                    continue
                x, y, z = segment_position(segment, time_sec, network)
                rows.append(
                    {
                        "time": round(time_sec, 3),
                        "id": flight_id,
                        "priority": flight.priority,
                        "uav_type": flight.uav_type,
                        "horizontal_sep_m": uav_types[flight.uav_type].horizontal_separation_m,
                        "vertical_sep_m": uav_types[flight.uav_type].vertical_separation_m,
                        "x": round(x, 3),
                        "y": round(y, 3),
                        "z": round(z, 3),
                        "segment_kind": segment.kind,
                        "segment_id": segment.segment_id,
                        "edge": segment.edge_id or "",
                        "layer_from": segment.layer_from or "",
                        "layer_to": segment.layer_to or "",
                        "lane": "" if segment.lane is None else segment.lane,
                        "route": "-".join(schedule.route),
                        "status": "finished"
                        if abs(time_sec - schedule.finish_sec) <= EPS
                        else "active",
                    }
                )
    return sorted(rows, key=lambda r: (float(r["time"]), str(r["id"])))


def validate_continuous_separation(
    flights: Sequence[FlightPlan],
    schedules: Mapping[str, FlightSchedule],
    network: NetworkConfig,
    uav_types: Mapping[str, UAVType],
    *,
    max_events_to_store: int = 5000,
) -> SafetyReport:
    if not schedules:
        return SafetyReport(
            passed=False,
            sample_dt_sec=network.trajectory_sample_sec,
            horizontal_threshold_m=0.0,
            vertical_threshold_m=0.0,
            minimum_horizontal_m=None,
            minimum_vertical_m=None,
            minimum_3d_m=None,
            loss_of_separation_samples=0,
            unique_loss_pairs=0,
            events=[],
        )

    flight_by_id = {f.flight_id: f for f in flights}
    start = min(s.scheduled_start_sec for s in schedules.values())
    end = max(s.finish_sec for s in schedules.values())
    min_h: float | None = None
    min_v: float | None = None
    min_3d: float | None = None
    loss_samples = 0
    loss_pairs: set[tuple[str, str]] = set()
    events: list[SafetyEvent] = []

    for time_sec in iter_sample_times(start, end, network.trajectory_sample_sec):
        active: list[tuple[str, float, float, float]] = []
        for flight_id, schedule in schedules.items():
            position = schedule_position_at(schedule, time_sec, network)
            if position is not None:
                x, y, z, _ = position
                active.append((flight_id, x, y, z))

        for (fa, xa, ya, za), (fb, xb, yb, zb) in itertools.combinations(active, 2):
            horizontal = math.hypot(xa - xb, ya - yb)
            vertical = abs(za - zb)
            distance_3d = math.sqrt(horizontal * horizontal + vertical * vertical)
            min_h = horizontal if min_h is None else min(min_h, horizontal)
            min_v = vertical if min_v is None else min(min_v, vertical)
            min_3d = distance_3d if min_3d is None else min(min_3d, distance_3d)

            type_a = uav_types[flight_by_id[fa].uav_type]
            type_b = uav_types[flight_by_id[fb].uav_type]
            horizontal_threshold = max(
                type_a.horizontal_separation_m,
                type_b.horizontal_separation_m,
            )
            vertical_threshold = max(
                type_a.vertical_separation_m,
                type_b.vertical_separation_m,
            )
            if horizontal < horizontal_threshold - EPS and vertical < vertical_threshold - EPS:
                loss_samples += 1
                pair = tuple(sorted((fa, fb)))
                loss_pairs.add(pair)
                if len(events) < max_events_to_store:
                    events.append(
                        SafetyEvent(
                            time_sec=time_sec,
                            flight_a=pair[0],
                            flight_b=pair[1],
                            horizontal_m=horizontal,
                            vertical_m=vertical,
                            distance_3d_m=distance_3d,
                        )
                    )

    max_h_threshold = max(t.horizontal_separation_m for t in uav_types.values())
    max_v_threshold = max(t.vertical_separation_m for t in uav_types.values())
    return SafetyReport(
        passed=loss_samples == 0,
        sample_dt_sec=network.trajectory_sample_sec,
        horizontal_threshold_m=max_h_threshold,
        vertical_threshold_m=max_v_threshold,
        minimum_horizontal_m=min_h,
        minimum_vertical_m=min_v,
        minimum_3d_m=min_3d,
        loss_of_separation_samples=loss_samples,
        unique_loss_pairs=len(loss_pairs),
        events=events,
    )


# ---------------------------------------------------------------------------
# Reporting and plotting
# ---------------------------------------------------------------------------


def strategy_counter(schedules: Mapping[str, FlightSchedule]) -> Counter[str]:
    counter: Counter[str] = Counter()
    for schedule in schedules.values():
        labels = set(schedule.strategy_labels)
        for label in (
            "unchanged",
            "upstream-speed-metering",
            "altitude-assignment/transition",
            "reroute",
            "ground-delay",
        ):
            if label in labels:
                counter[label] += 1
    return counter


def priority_metrics(
    flights: Sequence[FlightPlan],
    schedules: Mapping[str, FlightSchedule],
) -> list[dict[str, Any]]:
    flight_by_id = {f.flight_id: f for f in flights}
    groups: dict[int, list[FlightSchedule]] = defaultdict(list)
    for schedule in schedules.values():
        groups[flight_by_id[schedule.flight_id].priority].append(schedule)

    rows: list[dict[str, Any]] = []
    for priority in sorted(groups):
        group = groups[priority]
        delays = [s.delay_sec for s in group]
        reroutes = sum(s.route_index > 0 for s in group)
        altitude = sum(
            "altitude-assignment/transition" in s.strategy_labels for s in group
        )
        metering = sum("upstream-speed-metering" in s.strategy_labels for s in group)
        rows.append(
            {
                "priority": priority,
                "flight_count": len(group),
                "mean_delay_sec": statistics.fmean(delays) if delays else 0.0,
                "median_delay_sec": statistics.median(delays) if delays else 0.0,
                "p95_delay_sec": percentile(delays, 95.0) if delays else 0.0,
                "max_delay_sec": max(delays, default=0.0),
                "reroute_rate": reroutes / len(group),
                "altitude_action_rate": altitude / len(group),
                "speed_metering_rate": metering / len(group),
            }
        )
    return rows


def percentile(values: Sequence[float], p: float) -> float:
    if not values:
        raise ValueError("Cannot compute percentile of an empty sequence")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (p / 100.0) * (len(ordered) - 1)
    lo = math.floor(rank)
    hi = math.ceil(rank)
    if lo == hi:
        return ordered[lo]
    weight = rank - lo
    return ordered[lo] * (1.0 - weight) + ordered[hi] * weight


def plot_utilization_heatmap(
    initial: Mapping[str, FlightSchedule],
    final: Mapping[str, FlightSchedule],
    network: NetworkConfig,
    output_path: Path,
) -> None:
    occ_before, cap_before, labels, slots_before = edge_occupancy_matrix(initial, network)
    occ_after, cap_after, _, slots_after = edge_occupancy_matrix(final, network)
    max_slots = max(slots_before, slots_after)

    def pad(matrix: np.ndarray, width: int) -> np.ndarray:
        if matrix.shape[1] >= width:
            return matrix[:, :width]
        return np.pad(matrix, ((0, 0), (0, width - matrix.shape[1])))

    before_ratio = np.divide(
        pad(occ_before, max_slots),
        pad(cap_before, max_slots),
        out=np.zeros((len(labels), max_slots)),
        where=pad(cap_before, max_slots) > 0,
    )
    after_ratio = np.divide(
        pad(occ_after, max_slots),
        pad(cap_after, max_slots),
        out=np.zeros((len(labels), max_slots)),
        where=pad(cap_after, max_slots) > 0,
    )
    before_max = float(before_ratio.max(initial=0.0))
    after_max = float(after_ratio.max(initial=0.0))
    # Use the same fixed scale in both panels. Values above 2.0 are deliberately
    # saturated rather than letting an extreme initial overload make the valid
    # after-schedule panel visually indistinguishable from zero.
    display_vmax = 2.0

    fig, axes = plt.subplots(2, 1, figsize=(15, 8), sharex=True)
    for ax, matrix, title in (
        (
            axes[0],
            before_ratio,
            f"Before scheduling: utilization / capacity (max = {before_max:.1f})",
        ),
        (
            axes[1],
            after_ratio,
            f"After scheduling: utilization / capacity (max = {after_max:.1f})",
        ),
    ):
        image = ax.imshow(
            matrix,
            aspect="auto",
            origin="lower",
            interpolation="nearest",
            vmin=0.0,
            vmax=display_vmax,
            cmap="viridis",
        )
        ax.set_yticks(range(len(labels)))
        ax.set_yticklabels(labels)
        ax.set_ylabel("Edge-layer resource")
        ax.set_title(title)
        colorbar = fig.colorbar(
            image,
            ax=ax,
            label="Utilization ratio (1.0 = capacity; 2.0 = 2× or more)",
            extend="max",
        )
        colorbar.ax.axhline(1.0, linewidth=1.0)
    axes[1].set_xlabel(
        f"Time-slot index (slot length = {network.time_slot_sec:g} s)"
    )
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def plot_resource_gantt(
    schedules: Mapping[str, FlightSchedule],
    network: NetworkConfig,
    output_path: Path,
) -> None:
    resources: list[tuple[str, int, int]] = []
    for edge_id, edge in sorted(network.edges.items()):
        for layer in edge.allowed_layers:
            for lane in range(edge.lane_count):
                resources.append((edge_id, layer, lane))
    row_index = {resource: idx for idx, resource in enumerate(resources)}

    fig, ax = plt.subplots(figsize=(18, 9))
    priority_colors = {2: "tab:green", 5: "tab:orange", 10: "tab:red"}
    max_finish = 0.0
    for schedule in schedules.values():
        for segment in schedule.segments:
            if segment.kind != "edge" or segment.edge_id is None:
                continue
            assert segment.layer_from is not None and segment.lane is not None
            resource = (segment.edge_id, segment.layer_from, segment.lane)
            y = row_index[resource]
            ax.barh(
                y,
                segment.duration_sec,
                left=segment.start_sec,
                height=0.72,
                color=priority_colors.get(schedule.priority, "tab:blue"),
                edgecolor="black",
                linewidth=0.25,
                alpha=0.8,
            )
            max_finish = max(max_finish, segment.end_sec)
    ax.set_yticks(range(len(resources)))
    ax.set_yticklabels([f"{e}-L{l}-lane{n + 1}" for e, l, n in resources])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Explicit physical lane resource")
    ax.set_title("Final lane-level reservation schedule")
    ax.set_xlim(0.0, max_finish + 5.0)
    ax.grid(axis="x", linestyle="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def plot_priority_delay(
    metric_rows: Sequence[Mapping[str, Any]],
    output_path: Path,
) -> None:
    priorities = [str(row["priority"]) for row in metric_rows]
    means = [float(row["mean_delay_sec"]) for row in metric_rows]
    p95 = [float(row["p95_delay_sec"]) for row in metric_rows]
    x = np.arange(len(priorities))
    width = 0.36

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.bar(x - width / 2, means, width, label="Mean delay")
    ax.bar(x + width / 2, p95, width, label="95th percentile delay")
    ax.set_xticks(x)
    ax.set_xticklabels(priorities)
    ax.set_xlabel("Priority value (smaller = higher priority)")
    ax.set_ylabel("Delay (s)")
    ax.set_title("Priority-group delay audit")
    ax.legend()
    ax.grid(axis="y", linestyle="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def write_csv(path: Path, rows: Sequence[Mapping[str, Any]], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name, "") for name in fieldnames})


def export_all(
    output_dir: Path,
    flights: Sequence[FlightPlan],
    initial: Mapping[str, FlightSchedule],
    result: SchedulingResult,
    safety: SafetyReport,
    network: NetworkConfig,
    uav_types: Mapping[str, UAVType],
    run_config: Mapping[str, Any] | None = None,
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    paths: dict[str, Path] = {}

    scenario_rows = [
        {
            "id": f.flight_id,
            "priority": f.priority,
            "requested_start_sec": f.requested_start_sec,
            "origin_group": f.origin_group,
            "uav_type": f.uav_type,
            "default_layer": f.default_layer,
            "candidate_paths": json.dumps(f.candidate_paths),
        }
        for f in flights
    ]
    paths["scenario"] = output_dir / "scenario.csv"
    write_csv(
        paths["scenario"],
        scenario_rows,
        (
            "id",
            "priority",
            "requested_start_sec",
            "origin_group",
            "uav_type",
            "default_layer",
            "candidate_paths",
        ),
    )

    schedule_rows = []
    for schedule in sorted(result.schedules.values(), key=lambda s: s.flight_id):
        schedule_rows.append(
            {
                "id": schedule.flight_id,
                "priority": schedule.priority,
                "requested_start_sec": schedule.requested_start_sec,
                "scheduled_start_sec": schedule.scheduled_start_sec,
                "delay_sec": schedule.delay_sec,
                "route_index": schedule.route_index,
                "route": "-".join(schedule.route),
                "layer_plan": "-".join(map(str, schedule.layer_plan)),
                "lane_plan": "-".join(str(lane + 1) for lane in schedule.lane_plan),
                "speed_mode": schedule.speed_mode,
                "structural_cost": schedule.structural_cost,
                "selection_cost": schedule.selection_cost,
                "finish_sec": schedule.finish_sec,
                "strategies": ";".join(schedule.strategy_labels),
            }
        )
    paths["schedule"] = output_dir / "schedule.csv"
    write_csv(
        paths["schedule"],
        schedule_rows,
        (
            "id",
            "priority",
            "requested_start_sec",
            "scheduled_start_sec",
            "delay_sec",
            "route_index",
            "route",
            "layer_plan",
            "lane_plan",
            "speed_mode",
            "structural_cost",
            "selection_cost",
            "finish_sec",
            "strategies",
        ),
    )

    trajectory_rows = generate_trajectory_rows(
        flights, result.schedules, network, uav_types
    )
    paths["trajectory"] = output_dir / "uav_trajectories_refactored.csv"
    trajectory_fields = (
        "time",
        "id",
        "priority",
        "uav_type",
        "horizontal_sep_m",
        "vertical_sep_m",
        "x",
        "y",
        "z",
        "segment_kind",
        "segment_id",
        "edge",
        "layer_from",
        "layer_to",
        "lane",
        "route",
        "status",
    )
    write_csv(paths["trajectory"], trajectory_rows, trajectory_fields)

    safety_rows = [asdict(event) for event in safety.events]
    paths["safety_events"] = output_dir / "safety_events.csv"
    write_csv(
        paths["safety_events"],
        safety_rows,
        (
            "time_sec",
            "flight_a",
            "flight_b",
            "horizontal_m",
            "vertical_m",
            "distance_3d_m",
        ),
    )

    priority_rows = priority_metrics(flights, result.schedules)
    paths["priority_metrics"] = output_dir / "priority_metrics.csv"
    write_csv(
        paths["priority_metrics"],
        priority_rows,
        (
            "priority",
            "flight_count",
            "mean_delay_sec",
            "median_delay_sec",
            "p95_delay_sec",
            "max_delay_sec",
            "reroute_rate",
            "altitude_action_rate",
            "speed_metering_rate",
        ),
    )

    improvement_rows = [asdict(item) for item in result.improvement_history]
    paths["improvement_history"] = output_dir / "improvement_history.csv"
    write_csv(
        paths["improvement_history"],
        improvement_rows,
        (
            "pass_index",
            "total_delay_before_sec",
            "total_delay_after_sec",
            "improved_flights",
            "candidate_checks",
        ),
    )

    occupancy, capacity, labels, max_slot = edge_occupancy_matrix(result.schedules, network)
    occupancy_rows: list[dict[str, Any]] = []
    for row_index, label in enumerate(labels):
        for slot in range(max_slot):
            cap = float(capacity[row_index, slot])
            count = float(occupancy[row_index, slot])
            occupancy_rows.append(
                {
                    "resource": label,
                    "slot": slot,
                    "time_start_sec": slot * network.time_slot_sec,
                    "count": int(count),
                    "capacity": int(cap),
                    "utilization": count / cap if cap > 0 else math.inf,
                }
            )
    paths["occupancy"] = output_dir / "resource_occupancy.csv"
    write_csv(
        paths["occupancy"],
        occupancy_rows,
        (
            "resource",
            "slot",
            "time_start_sec",
            "count",
            "capacity",
            "utilization",
        ),
    )

    initial_reservation_conflicts = audit_reservation_conflicts(initial)
    final_reservation_conflicts = audit_reservation_conflicts(result.schedules)
    initial_edge_audit = audit_edge_capacity(initial, network)
    final_edge_audit = audit_edge_capacity(result.schedules, network)
    delays = [s.delay_sec for s in result.schedules.values()]
    strategies = strategy_counter(result.schedules)
    improvement_termination = (
        "disabled"
        if result.policy_config.get("local_reinsert_passes", 0) <= 0
        else (
            "no_change"
            if result.improvement_history
            and result.improvement_history[-1].improved_flights == 0
            else "pass_limit"
        )
    )
    summary = {
        "schema_version": OUTPUT_SCHEMA_VERSION,
        "software": {
            "name": SOFTWARE_NAME,
            "version": SOFTWARE_VERSION,
            "algorithm_id": ALGORITHM_ID,
            "algorithm_name": ALGORITHM_NAME,
        },
        "run_config": dict(run_config or {}),
        "scheduler_success": result.success,
        "sampled_trajectory_audit_passed": safety.passed,
        "overall_passed": result.success
        and not final_reservation_conflicts
        and final_edge_audit["overloaded_edge_layer_slots"] == 0
        and safety.passed,
        "message": result.message,
        "flight_count": len(flights),
        "scheduled_count": len(result.schedules),
        "unscheduled_flights": result.unscheduled_flights,
        "runtime_sec": result.runtime_sec,
        "construction_runtime_sec": result.construction_runtime_sec,
        "improvement_runtime_sec": result.improvement_runtime_sec,
        "search_attempts": result.search_attempts,
        "reservation_count": result.reservation_count,
        "policy": result.policy_config,
        "construction_total_delay_sec": result.construction_total_delay_sec,
        "total_delay_sec": sum(delays),
        "delay_reduction_from_construction_sec": (
            result.construction_total_delay_sec - sum(delays)
        ),
        "delay_reduction_from_construction_pct": (
            100.0 * (result.construction_total_delay_sec - sum(delays))
            / result.construction_total_delay_sec
            if result.construction_total_delay_sec > EPS
            else 0.0
        ),
        "improvement_history": [asdict(item) for item in result.improvement_history],
        "improvement_termination": improvement_termination,
        "mean_delay_sec": statistics.fmean(delays) if delays else None,
        "median_delay_sec": statistics.median(delays) if delays else None,
        "p95_delay_sec": percentile(delays, 95.0) if delays else None,
        "max_delay_sec": max(delays, default=None),
        "rerouted_flights": sum(s.route_index > 0 for s in result.schedules.values()),
        "strategy_usage_by_flight": dict(strategies),
        "initial_reservation_conflict_keys": len(initial_reservation_conflicts),
        "final_reservation_conflict_keys": len(final_reservation_conflicts),
        "initial_edge_capacity_audit": initial_edge_audit,
        "final_edge_capacity_audit": final_edge_audit,
        "safety_report": {
            key: value
            for key, value in asdict(safety).items()
            if key != "events"
        },
        "network": {
            "time_slot_sec": network.time_slot_sec,
            "edge_headway_sec": network.edge_headway_sec,
            "node_buffer_before_sec": network.node_buffer_before_sec,
            "node_buffer_after_sec": network.node_buffer_after_sec,
            "trajectory_sample_sec": network.trajectory_sample_sec,
            "max_delay_sec": network.max_delay_sec,
        },
    }
    paths["summary"] = output_dir / "summary.json"
    with paths["summary"].open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, ensure_ascii=False)

    paths["heatmap"] = output_dir / "utilization_heatmap.png"
    plot_utilization_heatmap(initial, result.schedules, network, paths["heatmap"])
    paths["gantt"] = output_dir / "resource_gantt.png"
    plot_resource_gantt(result.schedules, network, paths["gantt"])
    paths["priority_delay"] = output_dir / "priority_delay.png"
    plot_priority_delay(priority_rows, paths["priority_delay"])

    return paths


def print_console_summary(
    flights: Sequence[FlightPlan],
    result: SchedulingResult,
    safety: SafetyReport,
) -> None:
    delays = [s.delay_sec for s in result.schedules.values()]
    print("\n=== SPC-WSR UAV Strategic Reservation Scheduler ===")
    print(f"Flights requested        : {len(flights)}")
    print(f"Flights scheduled        : {len(result.schedules)}")
    print(f"Scheduler success        : {result.success}")
    print(f"Policy                   : {result.policy_config.get('name', 'unspecified')}")
    print(f"Runtime                  : {result.runtime_sec:.6f} s")
    print(f"  construction / improve : {result.construction_runtime_sec:.6f} / "
          f"{result.improvement_runtime_sec:.6f} s")
    print(f"Candidate checks         : {result.search_attempts}")
    print(f"Construction delay       : {result.construction_total_delay_sec:.1f} s")
    print(f"Final total ground delay : {sum(delays):.1f} s")
    if result.construction_total_delay_sec > EPS:
        reduction = result.construction_total_delay_sec - sum(delays)
        print(f"Local-search reduction   : {reduction:.1f} s "
              f"({100.0 * reduction / result.construction_total_delay_sec:.2f}%)")
    if delays:
        print(f"Mean / P95 / max delay   : {statistics.fmean(delays):.1f} / "
              f"{percentile(delays, 95.0):.1f} / {max(delays):.1f} s")
    print(
        "Sampled separation audit : "
        f"{'PASS' if safety.passed else 'FAIL'} "
        f"({safety.loss_of_separation_samples} violating samples, "
        f"{safety.unique_loss_pairs} pairs)"
    )
    if safety.minimum_3d_m is not None:
        print(f"Minimum sampled 3-D gap  : {safety.minimum_3d_m:.3f} m")
    if result.unscheduled_flights:
        print("Unscheduled flights      : " + ", ".join(result.unscheduled_flights))


def parse_priority_mapping(text: str, *, label: str) -> dict[int, float]:
    """Parse command-line mappings such as ``2:3,5:2,10:1``."""

    result: dict[int, float] = {}
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        try:
            raw_key, raw_value = item.split(":", 1)
            key = int(raw_key.strip())
            value = float(raw_value.strip())
        except (ValueError, TypeError) as exc:
            raise argparse.ArgumentTypeError(
                f"Invalid {label} entry {item!r}; expected priority:value"
            ) from exc
        if not math.isfinite(value):
            raise argparse.ArgumentTypeError(f"Non-finite {label} value in {item!r}")
        result[key] = value
    if not result:
        raise argparse.ArgumentTypeError(f"{label} mapping cannot be empty")
    return result


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Weighted-aging layered UAV reservation scheduler and NetLogo trajectory exporter."
    )
    parser.add_argument(
        "--scenario-csv",
        type=Path,
        default=None,
        help="Rerun an exact scenario.csv; when supplied, generation arguments are ignored.",
    )
    parser.add_argument("--num-flights", type=int, default=100)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--start-window", type=int, default=60, help="Requested-start window in seconds")
    parser.add_argument("--max-delay", type=float, default=1800.0, help="Maximum allowed search delay per flight")
    parser.add_argument("--sample-dt", type=float, default=0.25, help="Continuous replay sampling interval")
    parser.add_argument(
        "--dispatch-rule",
        choices=("strict", "fcfs", "aging"),
        default="strict",
        help=(
            "Construction order. The paper-safe default is strict priority; "
            "aging is available as an anti-starvation/fairness variant."
        ),
    )
    parser.add_argument(
        "--aging-offsets",
        default="2:0,5:30,10:60",
        help="Priority-to-offset mapping in seconds, e.g. 2:0,5:30,10:60.",
    )
    parser.add_argument(
        "--delay-weights",
        default="2:3,5:2,10:1",
        help="Priority-to-delay-weight mapping, e.g. 2:3,5:2,10:1.",
    )
    parser.add_argument(
        "--weight-candidate-delay",
        action="store_true",
        help=(
            "Apply priority weights inside each flight's structural candidate cost. "
            "Disabled by default because strong weights can increase total delay in "
            "severely saturated scenarios; weights still drive reinsertion order."
        ),
    )
    parser.add_argument(
        "--local-search-passes",
        type=int,
        default=5,
        help="Maximum safe weighted-reinsertion passes; set 0 to disable.",
    )
    parser.add_argument(
        "--air-time-weight",
        type=float,
        default=0.05,
        help="Delay-equivalent cost per second of airborne time.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("uav_delay_optimized_output"),
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.scenario_csv is None and args.num_flights <= 0:
        raise SystemExit("--num-flights must be positive")
    if args.scenario_csv is None and args.start_window < 0:
        raise SystemExit("--start-window cannot be negative")
    if args.max_delay < 0:
        raise SystemExit("--max-delay cannot be negative")
    if args.sample_dt <= 0:
        raise SystemExit("--sample-dt must be positive")
    if args.local_search_passes < 0:
        raise SystemExit("--local-search-passes cannot be negative")
    if args.air_time_weight < 0 or not math.isfinite(args.air_time_weight):
        raise SystemExit("--air-time-weight must be finite and non-negative")
    try:
        aging_offsets = parse_priority_mapping(args.aging_offsets, label="aging offset")
        delay_weights = parse_priority_mapping(args.delay_weights, label="delay weight")
    except argparse.ArgumentTypeError as exc:
        raise SystemExit(str(exc)) from exc
    if any(value < 0 for value in delay_weights.values()):
        raise SystemExit("Delay weights must be non-negative")
    # Requested starts are generated on a one-second grid and all strategic
    # durations/delays are aligned to the network time slot. Requiring the replay
    # step to divide both grids guarantees that segment endpoints are sampled.
    for base_step, label in ((1.0, "one second"), (2.0, "the 2-second time slot")):
        ratio = base_step / args.sample_dt
        if not math.isclose(ratio, round(ratio), rel_tol=0.0, abs_tol=1e-9):
            raise SystemExit(
                f"--sample-dt must evenly divide {label}; received {args.sample_dt}"
            )

    network = default_network(sample_dt=args.sample_dt, max_delay_sec=args.max_delay)
    uav_types = default_uav_types()
    if args.scenario_csv is not None:
        flights = load_scenario_csv(args.scenario_csv)
        run_config: dict[str, Any] = {
            "scenario_source": str(args.scenario_csv.resolve()),
            "generated": False,
        }
    else:
        flights = generate_scenario(args.num_flights, args.seed, args.start_window)
        run_config = {
            "scenario_source": "deterministic-generator",
            "generated": True,
            "seed": args.seed,
            "requested_flight_count": args.num_flights,
            "requested_start_window_sec": args.start_window,
        }

    unknown_types = sorted({f.uav_type for f in flights}.difference(uav_types))
    if unknown_types:
        raise SystemExit(
            "Scenario refers to undefined UAV type(s): " + ", ".join(unknown_types)
        )
    for flight in flights:
        if flight.default_layer not in network.layer_altitude_m:
            raise SystemExit(
                f"Flight {flight.flight_id} uses undefined default layer {flight.default_layer}"
            )
        for route in flight.candidate_paths:
            validate_route_topology(route, network)

    policy = SchedulingPolicy(
        name=(
            "age-wsr"
            if args.dispatch_rule == "aging"
            else (
                "spc-wsr"
                if args.dispatch_rule == "strict"
                else "fcfs-wsr"
            )
        ),
        dispatch_rule=args.dispatch_rule,
        aging_offsets_sec=aging_offsets,
        delay_weights=delay_weights,
        air_time_weight=args.air_time_weight,
        weight_candidate_delay=args.weight_candidate_delay,
        local_reinsert_passes=args.local_search_passes,
    )
    missing_priorities = sorted(
        {flight.priority for flight in flights}.difference(aging_offsets)
        if args.dispatch_rule == "aging"
        else set()
    )
    missing_weights = sorted(
        {flight.priority for flight in flights}.difference(delay_weights)
    )
    if missing_priorities:
        raise SystemExit(
            "Missing aging offsets for priority values: "
            + ", ".join(map(str, missing_priorities))
        )
    if missing_weights:
        raise SystemExit(
            "Missing delay weights for priority values: "
            + ", ".join(map(str, missing_weights))
        )

    initial = build_initial_schedules(flights, network, uav_types)
    result = schedule_flights(flights, network, uav_types, policy)

    # Sampled trajectory validation is meaningful only for scheduled flights. Failure is
    # still propagated through result.success and the JSON overall_passed flag.
    safety = validate_continuous_separation(
        flights,
        result.schedules,
        network,
        uav_types,
    )
    run_config.update(
        {
            "trajectory_sample_sec": args.sample_dt,
            "maximum_search_delay_sec": args.max_delay,
            "dispatch_rule": args.dispatch_rule,
            "aging_offsets_sec": aging_offsets,
            "delay_weights": delay_weights,
            "weight_candidate_delay": args.weight_candidate_delay,
            "local_search_passes": args.local_search_passes,
            "air_time_weight": args.air_time_weight,
        }
    )
    paths = export_all(
        args.output_dir,
        flights,
        initial,
        result,
        safety,
        network,
        uav_types,
        run_config,
    )
    print_console_summary(flights, result, safety)
    print("\nOutputs:")
    for name, path in sorted(paths.items()):
        print(f"  {name:18s} {path.resolve()}")

    final_edge_audit = audit_edge_capacity(result.schedules, network)
    overall_passed = (
        result.success
        and safety.passed
        and not audit_reservation_conflicts(result.schedules)
        and final_edge_audit["overloaded_edge_layer_slots"] == 0
    )
    return 0 if overall_passed else 2


if __name__ == "__main__":
    sys.exit(main())
