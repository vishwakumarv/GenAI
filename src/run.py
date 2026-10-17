"""Run one simulation.

  python src/run.py                                   # fixed 45 s + ambulance priority, seed 1
  python src/run.py --gui --amb emergency_single.rou.xml   # watch one ambulance (arrives ~t=600)
  python src/run.py --controller rl --emergency        # RL + ambulance priority
  python src/run.py --emergency --llm ollama           # let a local LLM make the HOLD/PREEMPT call
  python src/run.py --controller original --no-emergency   # your old network for comparison
"""
import argparse

from traffic import ROOT, run

ap = argparse.ArgumentParser()
ap.add_argument("--controller", choices=["original", "fixed", "rl"], default="fixed")
ap.add_argument("--emergency", action=argparse.BooleanOptionalAction, default=True)
ap.add_argument("--amb", default="emergency.rou.xml", help="ambulance file in sumo/, or 'none'")
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--llm", choices=["none", "ollama", "mock"], default="none")
ap.add_argument("--gui", action="store_true")
a = ap.parse_args()

agent = None
if a.controller == "rl":
    from stable_baselines3 import PPO
    model = PPO.load(ROOT / "models" / "ppo.zip", device="cpu")
    agent = lambda obs: int(model.predict(obs, deterministic=True)[0])  # noqa: E731
coord = None
if a.llm != "none":
    import llm_coordinator
    coord = llm_coordinator.OllamaCoordinator() if a.llm == "ollama" else llm_coordinator.MockCoordinator()

res = run(a.seed, a.controller, a.emergency and a.controller != "original",
          None if a.amb == "none" else a.amb, agent=agent, coordinator=coord, gui=a.gui)
for k, v in res.items():
    print(f"{k:24s} {v}")
if coord:
    print(f"LLM calls {coord.calls}, rejected/fallback {coord.rejected}")
