"""Train a PPO agent that picks NS or EW green every 5 s (no ambulances during training).

    python src/train_rl.py              -> models/ppo.zip  (~5 min; pass a bigger number, e.g. 150000, to train longer)

Training uses demand seeds 100-119; evaluation uses seeds 1-5 (never seen in training).
Reward = -(halted vehicles on the approaches + vehicles waiting to enter) / 50, averaged over 5 s.
The Controller still enforces min green, yellow and all-red, so the agent cannot do anything unsafe.
"""
import random
import sys

import gymnasium as gym
import numpy as np

from traffic import (IN_LANES, MAX_GREEN, ROOT, SUMO, Controller, observe, sumolib, traci)

TRAIN_SEEDS = list(range(100, 120))


class SignalEnv(gym.Env):
    def __init__(self, episode_s=3600):
        self.action_space = gym.spaces.Discrete(2)
        self.observation_space = gym.spaces.Box(0, 1, (19,), np.float32)
        self.episode_s, self.c = episode_s, None

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if self.c:
            self.c.close()
        s = random.choice(TRAIN_SEEDS)
        traci.start([sumolib.checkBinary("sumo"), "-n", str(SUMO / "net.net.xml"),
                     "-r", str(SUMO / "demand" / f"routes_{s}.rou.xml"), "--seed", str(s),
                     "--no-step-log", "--no-warnings"], label="train")
        self.c = traci.getConnection("train")
        self.ctl, self.t = Controller(self.c), 0
        return observe(self.c, self.ctl, 0), {}

    def step(self, action):
        want = ["NS", "EW"][int(action)]
        if self.ctl.in_green() and want == self.ctl.stage and self.t - self.ctl.green_since >= MAX_GREEN:
            want = "EW" if want == "NS" else "NS"
        cost = 0
        for _ in range(5):
            self.ctl.step(self.t, want)
            self.c.simulationStep()
            self.t += 1
            cost += sum(self.c.lane.getLastStepHaltingNumber(l) for l in IN_LANES)
            cost += len(self.c.simulation.getPendingVehicles())
        done = self.t >= self.episode_s
        obs = observe(self.c, self.ctl, self.t)
        if done:
            self.c.close()
            self.c = None
        return obs, -cost / 5 / 50, done, False, {}


if __name__ == "__main__":
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor
    steps = int(sys.argv[1]) if len(sys.argv) > 1 else 30_000
    (ROOT / "models").mkdir(exist_ok=True)
    (ROOT / "results").mkdir(exist_ok=True)
    env = Monitor(SignalEnv(), str(ROOT / "results" / "training"))
    model = PPO("MlpPolicy", env, n_steps=720, batch_size=120, learning_rate=3e-4, ent_coef=0.01,
                seed=0, verbose=0, device="cpu")
    for done in range(0, steps, 7200):          # save every 10 episodes, so an interruption loses little
        model.learn(total_timesteps=7200, reset_num_timesteps=False)
        model.save(ROOT / "models" / "ppo.zip")
        print("saved after", done + 7200, "steps")
    print("saved models/ppo.zip")
