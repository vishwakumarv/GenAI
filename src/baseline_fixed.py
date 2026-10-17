"""Fixed-time baseline: run SUMO with its default signal programs and record metrics.

Usage:
    python src/baseline_fixed.py --cfg sumo/config.sumocfg --seed 7 --out results/baseline_fixed.csv
Add --gui to watch it in sumo-gui.

Metrics written (one row per run, appended so you can run several seeds):
    avg_waiting_s, avg_queue_len, avg_travel_s, emergency_travel_s
"""
import argparse
import csv
import os
import sys
import xml.etree.ElementTree as ET

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
else:
    sys.exit("Set the SUMO_HOME environment variable first.")

import traci  # noqa: E402

try:
    from emergency_controller import EmergencySignalController
except ImportError:  # supports importing as src.baseline_fixed
    from src.emergency_controller import EmergencySignalController

EMERGENCY_TYPE = "emergency"  # vType id in routes.rou.xml


def run(cfg: str, seed: int, gui: bool, max_steps: int, tripinfo: str, ambulance_priority: bool = False) -> dict:
    binary = "sumo-gui" if gui else "sumo"
    traci.start([
        binary, "-c", cfg,
        "--seed", str(seed),
        "--tripinfo-output", tripinfo,
        "--no-step-log", "true",
        "--no-warnings", "true",
        *(["--start", "--quit-on-end"] if gui else []),
    ])

    # All lanes that belong to traffic-light-controlled junctions
    lanes = sorted({l for tl in traci.trafficlight.getIDList()
                    for l in traci.trafficlight.getControlledLanes(tl)})

    queue_samples = []
    priority_controller = EmergencySignalController(traci) if ambulance_priority else None
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0 and step < max_steps:
        traci.simulationStep()
        if priority_controller is not None:
            priority_controller.step()
        # queue length = vehicles halted (speed < 0.1 m/s) on controlled lanes
        queue_samples.append(sum(traci.lane.getLastStepHaltingNumber(l) for l in lanes))
        step += 1
    traci.close()

    # Per-vehicle stats come from SUMO's tripinfo file
    waits, durations, emerg = [], [], []
    for trip in ET.parse(tripinfo).getroot().iter("tripinfo"):
        w, d = float(trip.get("waitingTime")), float(trip.get("duration"))
        waits.append(w)
        durations.append(d)
        if trip.get("vType") == EMERGENCY_TYPE:
            emerg.append(d)

    mean = lambda xs: sum(xs) / len(xs) if xs else float("nan")  # noqa: E731
    return {
        "seed": seed,
        "vehicles_finished": len(waits),
        "avg_waiting_s": round(mean(waits), 2),
        "avg_queue_len": round(mean(queue_samples), 2),
        "avg_travel_s": round(mean(durations), 2),
        "emergency_travel_s": round(mean(emerg), 2),
        "ambulance_priority_enabled": ambulance_priority,
        "ambulance_preemptions": priority_controller.preemptions if priority_controller else 0,
        "ambulance_priority_s": round(priority_controller.emergency_priority_seconds, 2) if priority_controller else 0.0,
    }


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", default="sumo/config.sumocfg")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--ambulance-priority", action="store_true",
                    help="Enable predictive ambulance pre-emption and blocked-ambulance clearance.")
    ap.add_argument("--max-steps", type=int, default=3600)
    ap.add_argument("--out", default="results/baseline_fixed.csv")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    row = run(args.cfg, args.seed, args.gui, args.max_steps,
              tripinfo=args.out.replace(".csv", f"_tripinfo_{args.seed}.xml"),
              ambulance_priority=args.ambulance_priority)
    print(row)

    new_file = not os.path.exists(args.out)
    with open(args.out, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=row.keys())
        if new_file:
            w.writeheader()
        w.writerow(row)
