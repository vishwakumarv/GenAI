# Ambulance-aware traffic signal control

## What changed

- The central `A0` signal's normal cycle is now **45 seconds**: four 8.25-second green phases plus four 3-second yellow phases.
- The four peripheral single-link signal programs are also 45 seconds (37 seconds green, 3 seconds yellow, 5 seconds red).
- `src/emergency_controller.py` adds predictive ambulance detection using distance-to-junction and estimated arrival time. It includes yellow/all-red clearance before pre-emption, holds the ambulance's movement green while it approaches or is inside the intersection, and keeps priority if it is blocked mid-intersection.
- After the ambulance exits the intersection, the controller performs yellow/all-red clearance and resumes the phase implied by the **absolute simulation time modulo 45 seconds**. Emergency handling does not restart the global timer.
- `src/baseline_fixed.py` supports the controller through `--ambulance-priority` and records pre-emption metrics in the output CSV.

This is a deterministic predictive controller, not a trained RL/agentic-AI model. It can act as the safety/pre-emption layer alongside an RL policy. The current network's ambulance type is `emergency` and the controller is configured for the `A0` central junction and the existing four approach edge IDs.

## Run

From the project root, with SUMO installed and `SUMO_HOME` set:

```bash
python src/baseline_fixed.py --cfg sumo/config.sumocfg --seed 7 --max-steps 3600 --ambulance-priority --out results/ambulance_priority.csv
```

Add `--gui` to watch the simulation. For the original fixed-time comparison, omit `--ambulance-priority`.

## What to validate

1. An ambulance approaching from `top0A0` is detected before the junction, gets a protected priority phase, and crosses even when the normal phase would be red.
2. An ambulance on an internal `:A0_...` lane retains its priority phase until it reaches an outgoing edge.
3. The CSV shows `ambulance_preemptions` and `ambulance_priority_s`.
4. The global cycle position after recovery follows `simulation_time % 45`, not the time at which the ambulance cleared.

The controller uses SUMO vehicle type metadata as its emergency detection input. For a real deployment, connect this input to a validated ambulance/V2X detection source and retain a certified signal-safety interlock; simulation logic alone is not suitable for direct road deployment.
