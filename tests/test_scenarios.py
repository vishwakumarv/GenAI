"""Test A and Test B.

A: ambulance is the first vehicle in its lane and arrives at a RED light (no other traffic).
B: ambulance enters the junction and is then blocked inside it for 25 s, long past the point
   where its normal green would have ended.

Both must show: ambulance enters on protected green, conflicting movements stay red while it is
inside, priority ends only after it has left, the light sequence is safe, and the normal 45 s plan
is back in sync within one cycle.

    python -m pytest -q tests/test_scenarios.py      (or: python tests/test_scenarios.py  for a printout)
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from traffic import (ALL_RED_STATE, CYCLE, PHASES, SUMO, Controller, EmergencyPriority,  # noqa: E402
                     check_sequence, clock_stage, sumolib, traci, yellow_of)


def plan_state(t):
    """What the undisturbed 45 s plan shows at time t."""
    tc = t % CYCLE
    if tc < 17: return PHASES["NS"]
    if tc < 20: return yellow_of(PHASES["NS"])
    if tc < 22: return ALL_RED_STATE
    if tc < 40: return PHASES["EW"]
    if tc < 43: return yellow_of(PHASES["EW"])
    return ALL_RED_STATE


def scenario(depart, block_s=0, end=None, tmp=Path(".")):
    amb = tmp / "amb.rou.xml"
    amb.write_text('<routes><vType id="emergency" vClass="emergency" length="7" maxSpeed="20" speedFactor="1.3" '
                   'speedDev="0"/><trip id="ambulance_0" type="emergency" depart="%d" departLane="best" '
                   'departSpeed="max" from="top0A0" to="A0bottom0"/></routes>' % depart)
    traci.start([sumolib.checkBinary("sumo"), "-n", str(SUMO / "net.net.xml"), "-r", str(amb),
                 "--no-step-log", "--no-warnings"], label="scen")
    c = traci.getConnection("scen")
    ctl, em = Controller(c), EmergencyPriority(c)
    rec = {"entered": None, "entry_signal": None, "left_junction": None, "released": None,
           "bad_while_inside": [], "blocked_from": None}
    t = 0
    while t < (end or depart + 200):
        ctl.step(t, em.desired(t, ctl, clock_stage(t)))
        c.simulationStep()
        t = int(c.simulation.getTime())
        if "ambulance_0" in c.vehicle.getIDList():
            lane = c.vehicle.getLaneID("ambulance_0")
            links = em.ambulance_links("ambulance_0") if not lane.startswith(":") else links
            inside = lane.startswith(":A0")
            if inside and rec["entered"] is None:
                rec["entered"], rec["entry_signal"] = t, "".join(ctl.state[i] for i in links)
                if block_s:                                    # Test B: freeze it inside the junction
                    c.vehicle.setSpeedMode("ambulance_0", 0)    # allow an instant stop (breakdown)
                    c.vehicle.setSpeed("ambulance_0", 0)
                    rec["blocked_from"] = t
            if block_s and rec["blocked_from"] and t == rec["blocked_from"] + block_s:
                c.vehicle.setSpeedMode("ambulance_0", 31)
                c.vehicle.setSpeed("ambulance_0", -1)          # let it go again
            if inside:
                # while inside (N->S straight): its movement green, the crossing E and W approaches red
                others = [ctl.state[i] for i in list(range(4, 8)) + list(range(12, 16))]
                if any(ctl.state[i] != "G" for i in links) or any(ch != "r" for ch in others):
                    rec["bad_while_inside"].append((t, ctl.state))
            elif rec["entered"] and rec["left_junction"] is None:
                rec["left_junction"] = t
        if rec["released"] is None and any(e[2] == "released" for e in em.events):
            rec["released"] = t
    c.close()
    # back in sync: after release + one cycle, the display equals the undisturbed plan for a full cycle
    r = rec["released"]
    shown = dict()
    cur = None
    log = iter(ctl.log)
    entries = list(ctl.log)
    for k, (tt, st) in enumerate(entries):
        nxt = entries[k + 1][0] if k + 1 < len(entries) else t + 1
        for x in range(tt, nxt):
            shown[x] = st
    window = range(r + CYCLE, r + 2 * CYCLE)
    rec["in_sync"] = all(shown.get(x) == plan_state(x) for x in window if x in shown)
    rec["safety"] = check_sequence(ctl.log)
    rec["events"] = em.events
    return rec


def _check(rec):
    assert rec["entry_signal"] == "GG" or set(rec["entry_signal"]) == {"G"}, rec["entry_signal"]
    assert rec["bad_while_inside"] == []
    assert rec["released"] >= rec["left_junction"]
    assert rec["safety"] == []
    assert rec["in_sync"]


def test_A_first_in_lane_arrives_at_red(tmp_path):
    rec = scenario(depart=1825, tmp=tmp_path)   # 1825 % 45 = 25 -> EW green, so N is red on arrival
    assert any(e[2] == "PREEMPT" for e in rec["events"])
    _check(rec)


def test_B_blocked_inside_junction(tmp_path):
    rec = scenario(depart=1800, block_s=25, tmp=tmp_path)   # arrives on NS green, then stuck 25 s
    assert rec["left_junction"] - rec["entered"] >= 25
    _check(rec)


if __name__ == "__main__":
    import tempfile
    for name, kw in [("A: first in lane, arrives at red", dict(depart=1825)),
                     ("B: blocked inside junction for 25 s", dict(depart=1800, block_s=25))]:
        rec = scenario(tmp=Path(tempfile.mkdtemp()), **kw)
        print(f"\n== Test {name}")
        for e in rec["events"]:
            print(f"   t={e[0]}  {e[2]:8s} {e[3]}")
        print(f"   entered junction t={rec['entered']} on '{rec['entry_signal']}', left t={rec['left_junction']}, "
              f"priority released t={rec['released']}")
        print(f"   problems while inside: {len(rec['bad_while_inside'])} | sequence problems: {len(rec['safety'])} "
              f"| back in 45 s sync within one cycle: {rec['in_sync']}")
