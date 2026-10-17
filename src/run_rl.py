
"""Run the trained PPO traffic-signal agent in SUMO GUI."""

from pathlib import Path
from stable_baselines3 import PPO
import traci

from env import make_env


def main():
    project_dir = Path(__file__).resolve().parent.parent
    model_path = project_dir / "results" / "ppo_traffic.zip"

    if not model_path.exists():
        raise FileNotFoundError(f"Model not found: {model_path}")

    env = make_env(
        single_agent=True,
        seed=42,
        num_seconds=600,
        gui=True,
        reward_fn="diff-waiting-time",
    )

    model = PPO.load(str(model_path))

    try:
        obs, _ = env.reset()

        # Slow the SUMO GUI down so vehicles are easy to observe.
        traci.gui.setSchema  # placeholder - not used
        traci.simulationStep()

        total_reward = 0.0
        done = False

        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action)
            total_reward += float(reward)
            done = terminated or truncated

        print(f"Simulation finished. Total reward: {total_reward:.2f}")

    finally:
        env.close()


if __name__ == "__main__":
    main()
