# EV Route Planner with Charging-Stop Optimization

## Problem Statement

Build a route planner for EV drivers that factors in battery range, charging
station locations, and charging speed to suggest the optimal route with
minimal total trip time.

- Computes a feasible route + charging stops given a start point, end point,
  and current battery %.
- Uses real-world routing data (via OSRM) for distances and drive times.
- Never plans a route that would drop the battery below a safety reserve.
- Optimizes for minimum **total trip time** (drive time + charging time),
  not just distance.

## Solution We're Targeting

The core idea: plain shortest-path (Dijkstra on locations alone) doesn't
work for EVs, because whether you can even reach the next point depends on
how much charge you have left. So the search runs over an expanded state
space:

```
state = (location, battery_percent)
```

From any state, there are two possible moves:

| Move  | Effect                          | Cost               |
|-------|----------------------------------|---------------------|
| Drive | Move to another node, battery drops | Real drive time (from OSRM) |
| Charge | Stay at a station, battery rises | Charging time (based on station power) |

Dijkstra's algorithm runs over this state graph and finds the path to the
destination with the lowest total time, while every state along the way
respects the minimum battery reserve.

Battery is discretized into fixed steps (e.g. 2%) to keep the number of
states finite and the search fast.

## Parameters Considered

**Vehicle**
- `battery_capacity_kwh` — total battery size
- `consumption_kwh_per_km` — energy used per km driven
- `min_reserve_pct` — battery % the plan should never go below
- `max_charge_pct` — cap on how full the plan will ever charge to

**Charging stations**
- Location (lat/lon)
- `power_kw` — charging speed
- Charging is modeled with a two-stage curve: full speed up to 80%, slower
  above 80% (mirrors real CC-CV charging behavior)

**Route/distance**
- Distance and drive time come from OSRM's `/table` matrix API (real
  road-network routing, not straight-line estimates)
- `battery_step` — how finely battery % is discretized during search
  (smaller = more precise, slower search)

**Trip inputs**
- Start location, end location, starting battery %

## How to Run

```bash
pip install requests
python3 evrouteplan.py
```

By default it uses OSRM's public demo server. To use a self-hosted OSRM
instance instead:

```bash
export OSRM_BASE_URL=http://localhost:5000
python3 evrouteplan.py
```

## Output

The planner prints the full trip plan: each drive leg (distance, time,
battery before/after) and each charging stop (station, time, battery
before/after), plus the total trip time. If no feasible route exists given
the battery/reserve constraints, it says so instead of crashing.

## Known Limitations / Stretch Goals Not Yet Covered

- No elevation or weather impact on range
- No real-time charger occupancy or downtime
- Single objective (time) — no cost or multi-objective optimization
- Charging curve is a simplified two-stage model, not a full per-vehicle curve
