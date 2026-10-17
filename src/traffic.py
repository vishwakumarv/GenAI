"""Core of the project: signal phases, the controller, ambulance priority, and one simulation run.

Signal plan (junction A0, 16 links: N=0-3, E=4-7, S=8-11, W=12-15):
    NS green 17 s -> yellow 3 s -> all-red 2 s -> EW green 18 s -> yellow 3 s -> all-red 2 s  = 45 s
Ambulance priority:
    ambulance within 200 m of the junction
      - its movement is already green  -> HOLD that green until it has passed
      - otherwise                      -> PREEMPT: yellow, all-red, then green for its whole approach
    released once the ambulance has left the junction; the normal 45 s clock then simply carries on.
Safety: the controller can only show the states listed in ALLOWED, every green-to-red goes through
yellow and all-red, and check_sequence() verifies that on every run.
"""
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SUMO = ROOT / "sumo"

if "SUMO_HOME" not in os.environ:
    try:
        import sumo as _s  # pip install eclipse-sumo (fallback)
        os.environ["SUMO_HOME"] = _s.SUMO_HOME
    except ImportError:
        sys.exit("Set SUMO_HOME first.")
sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import sumolib  # noqa: E402
import traci  # noqa: E402

TLS = "A0"
CYCLE, YELLOW, ALL_RED, MIN_GREEN, MAX_GREEN = 45, 3, 2, 5, 50
NS_END, EW_END = 17, 40                     # clock positions where the NS / EW greens end
DETECT_M = 200                              # ambulance detection distance

PHASES = {
    "NS": "GGGgrrrrGGGgrrrr",   # north + south (left turns yield = g)
    "EW": "rrrrGGGgrrrrGGGg",   # east + west
    "N":  "GGGGrrrrrrrrrrrr",   # emergency: one approach, every movement protected
    "E":  "rrrrGGGGrrrrrrrr",
    "S":  "rrrrrrrrGGGGrrrr",
    "W":  "rrrrrrrrrrrrGGGG",
}
APPROACH = {"top0A0": "N", "right0A0": "E", "bottom0A0": "S", "left0A0": "W"}
IN_LANES = [f"{e}_{i}" for e in APPROACH for i in (0, 1)]
ALL_RED_STATE = "r" * 16
ALLOWED = set(PHASES.values()) | {p.replace("G", "y").replace("g", "y") for p in PHASES.values()} | {ALL_RED_STATE}


def yellow_of(state):
    return state.replace("G", "y").replace("g", "y")


def clock_stage(t):
    """Which normal stage the fixed 45 s plan wants at time t."""
    tc = t % CYCLE
    return "EW" if NS_END <= tc < EW_END else "NS"


# ------------------------------------------------------------------ controller
class Controller:
    """The only thing that writes to the traffic light."""

    def __init__(self, conn):
        self.conn = conn
        self.stage = "NS"          # current green stage (or the one being left)
        self.green_since = 0
        self.plan = []             # pending transition: list of (state, seconds); empty = in green
        self.target = None
        self.state = PHASES["NS"]
        self.log = []              # every state change: (time, state)

    def in_green(self):
        return not self.plan

    def step(self, t, desired):
        """Call once per simulated second, before simulationStep()."""
        if self.plan:                                    # in yellow / all-red
            if t >= self.plan[0][1]:
                self.plan.pop(0)
            if self.plan:
                self.state = self.plan[0][0]
            else:                                        # transition done -> new green
                self.stage, self.green_since = self.target, t
                self.state = PHASES[self.stage]
        elif desired != self.stage and t - self.green_since >= MIN_GREEN:
            self.target = desired                        # start: yellow, then all-red, then new green
            self.plan = [(yellow_of(PHASES[self.stage]), t + YELLOW), (ALL_RED_STATE, t + YELLOW + ALL_RED)]
            self.state = self.plan[0][0]
        assert self.state in ALLOWED, self.state         # never send anything outside the whitelist
        if not self.log or self.log[-1][1] != self.state:
            self.log.append((t, self.state))
        self.conn.trafficlight.setRedYellowGreenState(TLS, self.state)


def check_sequence(log):
    """Independent check of what was actually shown. Returns a list of problems (should be empty)."""
    bad = []
    for (t0, a), (t1, b) in zip(log, log[1:]):
        if any(x in "Gg" and y == "r" for x, y in zip(a, b)):
            bad.append(f"t={t1}: green -> red without yellow")
        if "y" in a and (b != ALL_RED_STATE or t1 - t0 < YELLOW):
            bad.append(f"t={t1}: yellow too short or not followed by all-red")
        if a == ALL_RED_STATE and t1 - t0 < ALL_RED:
            bad.append(f"t={t1}: all-red only {t1 - t0}s")
    return bad


# ------------------------------------------------------------------ ambulance priority
class EmergencyPriority:
    def __init__(self, conn, coordinator=None):
        self.conn = conn
        self.coordinator = coordinator          # None -> plain rules; else llm_coordinator object
        self.active = None                      # (vehicle id, stage)
        self.events = []                        # (time, vehicle, what, detail)
        self.links = {}                         # (from_edge, to_edge) -> link indices
        for i, group in enumerate(conn.trafficlight.getControlledLinks(TLS)):
            if group:
                fr, to = group[0][0].rsplit("_", 1)[0], group[0][1].rsplit("_", 1)[0]
                self.links.setdefault((fr, to), []).append(i)

    def ambulance_links(self, v):
        r, i = self.conn.vehicle.getRoute(v), self.conn.vehicle.getRouteIndex(v)
        return self.links.get((r[i], r[i + 1]), []) if i + 1 < len(r) else []

    def desired(self, t, ctl, normal_desired):
        c = self.conn
        if self.active:                                   # keep priority until it has left the junction
            v, stage = self.active
            gone = v not in c.vehicle.getIDList() or (
                c.vehicle.getRoadID(v) not in APPROACH and not c.vehicle.getRoadID(v).startswith(":"))
            if not gone:
                return stage
            self.events.append((t, v, "released", stage))
            self.active = None
        if not ctl.in_green():
            return normal_desired
        # closest ambulance on an approach
        best = None
        for v in c.vehicle.getIDList():
            if c.vehicle.getVehicleClass(v) != "emergency":
                continue
            edge = c.vehicle.getRoadID(v)
            if edge not in APPROACH:
                continue
            dist = c.lane.getLength(c.vehicle.getLaneID(v)) - c.vehicle.getLanePosition(v)
            if dist <= DETECT_M and (best is None or dist < best[1]):
                best = (v, dist, edge)
        if best is None:
            return normal_desired
        v, dist, edge = best
        links = self.ambulance_links(v)
        already_green = all(ctl.state[i] == "G" for i in links)
        action = "HOLD" if already_green else "PREEMPT"
        source = "rule"
        if self.coordinator is not None:
            situation = {"approach": APPROACH[edge], "distance_m": round(dist), "speed_mps":
                         round(c.vehicle.getSpeed(v), 1), "movement_is_green": already_green,
                         "current_stage": ctl.stage}
            action, source = self.coordinator.decide(situation)
        stage = ctl.stage if action == "HOLD" else APPROACH[edge]
        self.active = (v, stage)
        self.events.append((t, v, action, f"{stage} ({source}, {dist:.0f} m)"))
        return stage


# ------------------------------------------------------------------ one simulation run
def run(seed=1, controller="fixed", emergency=False, amb_file="emergency.rou.xml", agent=None,
        coordinator=None, gui=False, end=3600, out_dir=None):
    """controller: 'original' (your old net, SUMO's own plan) | 'fixed' (45 s) | 'rl' (needs agent).
    Returns a dict of metrics."""
    out_dir = Path(out_dir or ROOT / "results" / "runs" / f"{controller}{'_em' if emergency else ''}_s{seed}")
    out_dir.mkdir(parents=True, exist_ok=True)
    tripinfo = out_dir / "tripinfo.xml"
    net = SUMO / ("net.original.net.xml" if controller == "original" else "net.net.xml")
    routes = [str(SUMO / "demand" / f"routes_{seed}.rou.xml")] + ([str(SUMO / amb_file)] if amb_file else [])
    cmd = [sumolib.checkBinary("sumo-gui" if gui else "sumo"), "-n", str(net), "-r", ",".join(routes),
           "--seed", str(seed), "--end", str(end), "--no-step-log", "--no-warnings",
           "--tripinfo-output", str(tripinfo), "--tripinfo-output.write-unfinished"]
    if gui:
        cmd += ["--start", "--quit-on-end", "--delay", "50"]
    label = f"run{seed}{controller}{emergency}{id(out_dir)}"
    traci.start(cmd, label=label)
    c = traci.getConnection(label)

    ctl = Controller(c) if controller != "original" else None
    em = EmergencyPriority(c, coordinator) if (emergency and ctl) else None
    rl_choice, queue_sum, amb_bad_entries, seen_in_junction = "NS", 0, [], set()
    t = 0
    while t < end and c.simulation.getMinExpectedNumber() > 0:
        if ctl:
            if controller == "rl":
                if t % 5 == 0:
                    rl_choice = ["NS", "EW"][agent(observe(c, ctl, t))]
                    if ctl.in_green() and rl_choice == ctl.stage and t - ctl.green_since >= MAX_GREEN:
                        rl_choice = "EW" if ctl.stage == "NS" else "NS"
                normal = rl_choice
            else:
                normal = clock_stage(t)
            ctl.step(t, em.desired(t, ctl, normal) if em else normal)
        c.simulationStep()
        t = int(c.simulation.getTime())
        queue_sum += sum(c.lane.getLastStepHaltingNumber(l) for l in IN_LANES)
        if em:   # did any ambulance cross the stop line on anything but protected green?
            for v in c.vehicle.getIDList():
                if v.startswith("ambulance") and v not in seen_in_junction and c.vehicle.getLaneID(v).startswith(":A0"):
                    seen_in_junction.add(v)
                    links = em.ambulance_links(v)
                    if any(ctl.state[i] != "G" for i in links):
                        amb_bad_entries.append((t, v, "".join(ctl.state[i] for i in links)))
    c.close()

    trips = list(ET.parse(tripinfo).getroot().iter("tripinfo"))
    cars = [x for x in trips if x.get("vType") != "emergency"]
    done = [x for x in cars if float(x.get("arrival")) >= 0]
    amb = [x for x in trips if x.get("vType") == "emergency" and float(x.get("arrival")) >= 0]
    avg = lambda xs, k: round(sum(float(x.get(k)) for x in xs) / len(xs), 2) if xs else None  # noqa: E731
    res = {
        "controller": controller, "emergency": emergency, "seed": seed,
        "finished": len(done), "unfinished": len(cars) - len(done),
        "avg_wait_s": avg(done, "waitingTime"), "avg_travel_s": avg(done, "duration"),
        "time_lost_h": round(sum(float(x.get("timeLoss")) + float(x.get("departDelay")) for x in cars) / 3600, 2),
        "avg_queue": round(queue_sum / max(t, 1), 2),
        "amb_travel_s": avg(amb, "duration"), "amb_wait_s": avg(amb, "waitingTime"),
        "amb_stops": sum(int(x.get("waitingCount")) for x in amb) if amb else None,
        "safety_problems": len(check_sequence(ctl.log)) if ctl else None,
        "amb_entered_not_green": len(amb_bad_entries) if em else None,
        "priority_actions": sum(1 for e in em.events if e[2] in ("HOLD", "PREEMPT")) if em else None,
    }
    if em:
        with open(out_dir / "emergency_events.csv", "w") as f:
            f.write("time,vehicle,event,detail\n")
            f.writelines(f"{a},{b},{c_},{d}\n" for a, b, c_, d in em.events)
    os.remove(tripinfo)       # large; metrics are already extracted
    return res


def observe(c, ctl, t):
    """RL observation (19 numbers, 0..1): halted + vehicles per incoming lane, current stage, time in green."""
    import numpy as np
    h = [min(c.lane.getLastStepHaltingNumber(l) / 26, 1) for l in IN_LANES]
    n = [min(c.lane.getLastStepVehicleNumber(l) / 26, 1) for l in IN_LANES]
    return np.array(h + n + [ctl.stage == "NS", ctl.stage == "EW", min((t - ctl.green_since) / MAX_GREEN, 1)],
                    dtype=np.float32)


if __name__ == "__main__":
    print(run(1, "fixed", emergency=True, amb_file="emergency_single.rou.xml"))
