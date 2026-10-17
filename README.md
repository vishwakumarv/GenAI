# Smart Traffic Management (basic version)

One 4-way junction in SUMO. Normal traffic runs on a **fixed 45 s plan** or a **PPO (RL) agent**.
Ambulances get **priority** (hold the green or switch to their approach), with an optional
**LLM** making that call. A comparison script shows what each part does.

## Files
```
sumo/net.net.xml            new network: one traffic light (A0), 45 s cycle
sumo/net.original.net.xml   your old network (5 traffic lights), kept for comparison
sumo/demand/routes_N.rou.xml  traffic per seed (1-5 = evaluation, 100-119 = training)
sumo/emergency*.rou.xml     ambulances: single / 30 N->S / 30 mixed directions
src/traffic.py              phases, controller, ambulance priority, run() + metrics   <- the core
src/llm_coordinator.py      optional LLM (Ollama) that picks HOLD / PREEMPT, with validation
src/train_rl.py             PPO training -> models/ppo.zip
src/run.py                  run one simulation (optionally in sumo-gui)
src/evaluate.py             compare everything -> results/summary.csv, comparison.png
src/make_files.py           regenerate network + traffic files (not needed, already included)
tests/test_basic.py         6 checks
```

## Run it (PowerShell)
```powershell
cd "$HOME\smart-traffic-agentic"
.\.venv\Scripts\Activate.ps1
pip install pytest

python -m pytest -q tests                                    # 6 passed
python src/run.py --gui --amb emergency_single.rou.xml       # watch an ambulance (~t=600 s)
python src/run.py --controller rl                            # RL + ambulance priority
python src/evaluate.py                                       # full comparison (~5 min)
python src/train_rl.py                                       # retrain RL (optional)
```

## How it works
**Signal plan:** NS green 17 s → yellow 3 s → all-red 2 s → EW green 18 s → yellow 3 s → all-red 2 s = **45 s**.
Left turns go on "permissive" green (they give way to oncoming traffic).

**Controller (`Controller` in traffic.py):** the only code that changes the light. You ask it for a stage
(`NS`, `EW`, or an emergency stage `N`/`E`/`S`/`W`). It keeps the current green for at least 5 s, then
shows yellow 3 s and all-red 2 s before the new green. It can only show states from a fixed whitelist,
and `check_sequence()` re-checks every run afterwards (`safety_problems` should be 0).

**Ambulance priority (`EmergencyPriority`):** when an ambulance is within 200 m of the junction:
- its movement is already green → **HOLD** that green until it has passed
- otherwise → **PREEMPT**: yellow, all-red, then green for its whole approach

Priority ends when the ambulance has left the junction; the normal 45 s clock then carries on where it
would have been. Events are saved in `results/runs/<run>/emergency_events.csv`.

**RL:** PPO chooses NS or EW every 5 s, but the controller still enforces yellow, all-red and min green,
so the agent can't do anything unsafe. Reward is minus the number of queued vehicles.

**LLM (optional):** `--llm ollama` sends the ambulance situation to a local model (free; install Ollama,
then `ollama pull llama3.2:3b`). It may only answer HOLD or PREEMPT. Anything else, or HOLD on a red,
is rejected and the simple rule is used instead. `--llm mock` is a fake LLM that sometimes answers
nonsense, to show that the validation works.

## What changed from your setup
- The old network had 5 traffic lights; the 4 edge ones only controlled U-turns. Now only A0 has one.
- The old ambulance type had `has.bluelight.device`, which lets SUMO ambulances **run red lights**
  (7 of 30 did). They now obey signals, so the priority system is what gets them through.
- About 24 % of the old `routes.rou.xml` were U-turns at the junction. The new routes have none, and
  every comparison (including the old network) uses the same new routes. Your originals are kept as
  `*.original.*`.

## Results
See `results/summary.csv` and `results/comparison.png`. These are real runs, 5 unseen seeds each.

Measured (mean of seeds 1-5, 30 ambulances/hour; `results/summary.csv`):

| scenario | car wait (s) | ambulance travel (s) | ambulance wait (s) |
|---|---:|---:|---:|
| old 5-controller network | 35.3 | 81.3 | 45.7 |
| fixed 45 s | 17.6 | 42.8 | 10.5 |
| fixed 45 s + ambulance priority | 19.2 | 27.9 | 0.0 |
| RL | 17.0 | 51.1 | 14.3 |
| RL + ambulance priority | 19.9 | 30.4 | 0.6 |

0 safety problems and 0 ambulances entering on anything but protected green, in every run.
The RL agent here is only lightly trained (30k steps, ~5 min): it's about level with the fixed plan.
Train longer with `python src/train_rl.py 150000`.
