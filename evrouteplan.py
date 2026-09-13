import heapq
import math
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

try:
    import requests
except ImportError:
    requests = None  # only needed if OSRMProvider is actually used



@dataclass
class Location:
    id: str
    name: str
    lat: float
    lon: float


@dataclass
class ChargingStation(Location):
    power_kw: float = 50.0          # charging speed
    connector: str = "CCS"


@dataclass
class Vehicle:
    battery_capacity_kwh: float = 60.0
    consumption_kwh_per_km: float = 0.18   # ~180 Wh/km, typical mid-size EV
    min_reserve_pct: float = 10.0          # never plan to go below this
    max_charge_pct: float = 100.0          # cap charging target (buffer for battery health)



class DistanceProvider(ABC):
    @abstractmethod
    def get_matrix(self, nodes: list[Location]) -> dict:
        ...


class HaversineProvider(DistanceProvider):

    def __init__(self, avg_speed_kmh: float = 80.0):
        self.avg_speed_kmh = avg_speed_kmh

    @staticmethod
    def _haversine_km(a: Location, b: Location) -> float:
        R = 6371.0
        lat1, lon1, lat2, lon2 = map(math.radians, (a.lat, a.lon, b.lat, b.lon))
        dlat, dlon = lat2 - lat1, lon2 - lon1
        h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        return 2 * R * math.asin(math.sqrt(h))

    def get_matrix(self, nodes):
        result = {}
        for a in nodes:
            for b in nodes:
                if a.id == b.id:
                    continue
                d = self._haversine_km(a, b)
                t = (d / self.avg_speed_kmh) * 60.0
                result[(a.id, b.id)] = (d, t)
        return result


class OSRMProvider(DistanceProvider):
    """
    Uses OSRM's /table service for a full distance+duration matrix in one call.

    base_url resolution: explicit arg -> OSRM_BASE_URL env var -> public
    demo server (fine for testing, not for production -- rate limited,
    no SLA). Self-host for real use (osrm-extract/partition/customize +
    osrm-routed against a regional .osm.pbf extract).
    """

    def __init__(self, base_url: Optional[str] = None, profile: str = "driving"):
        self.base_url = (base_url or os.environ.get("OSRM_BASE_URL")
                          or "https://router.project-osrm.org").rstrip("/")
        self.profile = profile

    def get_matrix(self, nodes: list[Location]) -> dict:
        if requests is None:
            raise RuntimeError("The 'requests' package is required for OSRMProvider: pip install requests")

        coords = ";".join(f"{n.lon},{n.lat}" for n in nodes)
        url = f"{self.base_url}/table/v1/{self.profile}/{coords}"
        params = {"annotations": "distance,duration"}

        resp = requests.get(url, params=params, timeout=15)
        resp.raise_for_status()
        data = resp.json()

        if data.get("code") != "Ok":
            raise RuntimeError(f"OSRM error: {data.get('message', data.get('code'))}")

        distances = data["distances"]   # meters, NxN
        durations = data["durations"]   # seconds, NxN

        result = {}
        for i, a in enumerate(nodes):
            for j, b in enumerate(nodes):
                if i == j:
                    continue
                # OSRM returns null for a cell when no route exists between
                # that pair -- must check before dividing, or this crashes.
                if distances[i][j] is None or durations[i][j] is None:
                    continue  # treat as unreachable
                km = distances[i][j] / 1000.0
                minutes = durations[i][j] / 60.0
                result[(a.id, b.id)] = (km, minutes)
        return result



# 3. Charging model

def charge_time_min(vehicle: Vehicle, station: ChargingStation,
                     from_pct: float, to_pct: float) -> float:
    
    if to_pct <= from_pct:
        return 0.0

    def energy_kwh(pct):
        return vehicle.battery_capacity_kwh * pct / 100.0

    total_minutes = 0.0
    segments = [(from_pct, min(to_pct, 80.0), 1.0),      # full speed below 80%
                (max(from_pct, 80.0), to_pct, 0.4)]       # ~40% speed above 80%

    for seg_start, seg_end, speed_factor in segments:
        if seg_end > seg_start:
            kwh = energy_kwh(seg_end) - energy_kwh(seg_start)
            effective_kw = station.power_kw * speed_factor
            total_minutes += (kwh / effective_kw) * 60.0

    return total_minutes




@dataclass(order=True)
class PQItem:
    time: float
    seq: int
    node_id: str = field(compare=False)
    battery: float = field(compare=False)


@dataclass
class Move:
    action: str            # "drive" or "charge"
    from_node: str
    to_node: str
    detail: str
    duration_min: float
    battery_before: float
    battery_after: float


def plan_route(vehicle: Vehicle, start: Location, end: Location,
                stations: list[ChargingStation], start_battery_pct: float,
                battery_step: float = 2.0,
                provider: Optional[DistanceProvider] = None):

    provider = provider or OSRMProvider()

    nodes: dict[str, Location] = {start.id: start, end.id: end}
    for s in stations:
        nodes[s.id] = s
    station_ids = {s.id for s in stations}

    max_range_km = (vehicle.battery_capacity_kwh * (100 - vehicle.min_reserve_pct) / 100.0) \
        / vehicle.consumption_kwh_per_km

    def battery_after_drive(batt_pct, dist_km):
        used_pct = (dist_km * vehicle.consumption_kwh_per_km / vehicle.battery_capacity_kwh) * 100.0
        return batt_pct - used_pct

    def round_batt(b):
        # Used for charge targets, which are already exact multiples of
        # battery_step -- nearest-rounding here is just float cleanup.
        return round(round(b / battery_step) * battery_step, 2)

    def floor_batt(b):
        # Used after a DRIVE. Must round DOWN (never up) -- rounding a
        # measured/estimated battery level up would let the planner believe
        # it has more charge than it truly does, silently eroding the
        # safety reserve. Floor is the conservative, safe direction.
        return round(math.floor(max(b, 0.0) / battery_step) * battery_step, 2)

    ids = list(nodes.keys())
    try:
        matrix = provider.get_matrix(list(nodes.values()))
    except Exception as exc:
        print(f"Routing provider failed ({type(exc).__name__}: {exc}); no plan computed.")
        return None
    dist_cache = {k: v[0] for k, v in matrix.items()}   # (from,to) -> km
    time_cache = {k: v[1] for k, v in matrix.items()}   # (from,to) -> minutes (real, not estimated)

    start_state = (start.id, floor_batt(start_battery_pct))
    best_time = {start_state: 0.0}
    came_from: dict[tuple, tuple[tuple, Move]] = {}

    pq = []
    seq = 0
    heapq.heappush(pq, PQItem(0.0, seq, start.id, start_state[1]))
    visited = set()

    while pq:
        item = heapq.heappop(pq)
        state = (item.node_id, item.battery)
        if state in visited:
            continue
        visited.add(state)
        cur_time = item.time
        node_id, batt = state

        if node_id == end.id:
            return _reconstruct(came_from, state, cur_time)
        
        if node_id in station_ids:
            station = nodes[node_id]
            for target in _charge_targets(batt, vehicle.max_charge_pct, battery_step):
                dt = charge_time_min(vehicle, station, batt, target)
                new_time = cur_time + dt
                new_state = (node_id, round_batt(target))
                if new_state in visited:
                    continue  # already finalized -- never re-open (avoids came_from cycles)
                if new_time < best_time.get(new_state, math.inf):
                    best_time[new_state] = new_time
                    came_from[new_state] = (state, Move(
                        "charge", node_id, node_id,
                        f"Charge at {station.name} ({station.power_kw} kW)",
                        dt, batt, target))
                    seq += 1
                    heapq.heappush(pq, PQItem(new_time, seq, node_id, round_batt(target)))
                    
        for other_id in ids:
            if other_id == node_id:
                continue
            if (node_id, other_id) not in dist_cache:
                continue  # no route found between these two 
            d = dist_cache[(node_id, other_id)]
            if d > max_range_km:
                continue  # can't possibly reach even at full charge -- prune early
            new_batt = battery_after_drive(batt, d)
            if new_batt < vehicle.min_reserve_pct:
                continue  # would violate safety reserve
            dt = time_cache[(node_id, other_id)]  # real drive time from the routing engine
            new_time = cur_time + dt
            new_state = (other_id, floor_batt(new_batt))
            if new_state in visited:
                continue  # already finalized -- never re-open (avoids came_from cycles)
            if new_time < best_time.get(new_state, math.inf):
                best_time[new_state] = new_time
                came_from[new_state] = (state, Move(
                    "drive", node_id, other_id,
                    f"Drive {d:.1f} km to {nodes[other_id].name}",
                    dt, batt, new_batt))
                seq += 1
                heapq.heappush(pq, PQItem(new_time, seq, other_id, round_batt(new_batt)))

    return None  # no feasible route


def _charge_targets(from_pct, max_pct, step):
    t = from_pct + step
    while t <= max_pct + 1e-9:
        yield min(t, max_pct)
        t += step


def _reconstruct(came_from, end_state, total_time):
    path = []
    seen = set()
    state = end_state
    while state in came_from:
        if state in seen:
            # Should be unreachable given the visited-guard in plan_route, but
            # fail loudly rather than hang if the discretized graph ever
            # produces a cycle again.
            raise RuntimeError(f"Cycle detected while reconstructing route at state {state}")
        seen.add(state)
        prev_state, move = came_from[state]
        path.append(move)
        state = prev_state
    path.reverse()
    return {"total_time_min": total_time, "moves": path}


# Mock data + demo run
def build_mock_world():
    # Mumbai -> Hyderabad, roughly following the NH65 corridor
    # (Mumbai -> Pune -> Solapur -> Zaheerabad -> Hyderabad).
    start = Location("start", "Mumbai", 19.0760, 72.8777)
    end = Location("end", "Hyderabad", 17.3850, 78.4867)

    stations = [
        ChargingStation("cs1", "Pune Charging Hub", 18.5204, 73.8567, power_kw=120),
        ChargingStation("cs2", "Solapur Fast Charge", 17.6599, 75.9064, power_kw=90),
        ChargingStation("cs3", "Zaheerabad EV Point", 17.7231, 77.6047, power_kw=60),
    ]
    return start, end, stations


def print_plan(plan):
    if plan is None:
        print("No feasible route found with the given battery/reserve constraints.")
        return

    print(f"\n=== Optimal Route Plan ===")
    print(f"Total trip time: {plan['total_time_min']:.1f} min "
          f"({plan['total_time_min']/60:.2f} hrs)\n")
    
    collapsed = []
    for mv in plan["moves"]:
        if (collapsed and mv.action == "charge" and collapsed[-1].action == "charge"
                and collapsed[-1].to_node == mv.to_node):
            prev = collapsed[-1]
            collapsed[-1] = Move(prev.action, prev.from_node, prev.to_node, prev.detail,
                                  prev.duration_min + mv.duration_min,
                                  prev.battery_before, mv.battery_after)
        else:
            collapsed.append(mv)

    step = 1
    for mv in collapsed:
        if mv.action == "drive":
            print(f"{step}. DRIVE  -> {mv.detail}")
        else:
            print(f"{step}. CHARGE @ {mv.from_node} -> {mv.detail}")
        print(f"    battery: {mv.battery_before:.1f}% -> {mv.battery_after:.1f}%  "
              f"(time: {mv.duration_min:.1f} min)")
        step += 1
    print()


if __name__ == "__main__":
    vehicle = Vehicle(
        battery_capacity_kwh=60.0,
        consumption_kwh_per_km=0.18,
        min_reserve_pct=10.0,
        max_charge_pct=100.0,
    )
    start, end, stations = build_mock_world()

    # OSRM_BASE_URL env var can point this at a self-hosted instance;
    # otherwise it falls back to the public demo server.
    provider = OSRMProvider()

    plan = plan_route(
        vehicle=vehicle,
        start=start,
        end=end,
        stations=stations,
        start_battery_pct=90.0,   # try changing this to see route change
        battery_step=2.0,
        provider=provider,
    )

    print_plan(plan)