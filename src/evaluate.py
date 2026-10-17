"""Compare the controllers on 5 evaluation seeds (never used in training).

    python src/evaluate.py

Writes results/comparison.csv (every run), results/summary.csv (averages), results/comparison.png,
and prints the summary table.
"""
import pandas as pd

from traffic import ROOT, run

SEEDS = [1, 2, 3, 4, 5]
AMB = "emergency.rou.xml"        # 30 ambulances N->S, one every 2 minutes


def main():
    from stable_baselines3 import PPO
    model = PPO.load(ROOT / "models" / "ppo.zip", device="cpu")
    agent = lambda obs: int(model.predict(obs, deterministic=True)[0])  # noqa: E731

    scenarios = [  # name, controller, emergency priority on?
        ("1 original (old 5-controller net)", "original", False),
        ("2 fixed 45 s", "fixed", False),
        ("3 fixed 45 s + ambulance priority", "fixed", True),
        ("4 RL", "rl", False),
        ("5 RL + ambulance priority", "rl", True),
    ]
    rows = []
    for name, ctl, em in scenarios:
        for s in SEEDS:
            r = run(s, ctl, em, AMB, agent=agent if ctl == "rl" else None)
            r["scenario"] = name
            rows.append(r)
            print(name, "seed", s, "wait", r["avg_wait_s"], "amb wait", r["amb_wait_s"])
    df = pd.DataFrame(rows)
    df.to_csv(ROOT / "results" / "comparison.csv", index=False)
    cols = ["avg_wait_s", "avg_travel_s", "time_lost_h", "avg_queue", "finished",
            "amb_travel_s", "amb_wait_s", "amb_stops", "safety_problems", "amb_entered_not_green"]
    summary = df.groupby("scenario")[cols].mean().round(2)
    summary.to_csv(ROOT / "results" / "summary.csv")
    print(summary.to_string())

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    for ax, (col, title) in zip(axes, [("avg_wait_s", "Normal cars: avg waiting (s)"),
                                       ("amb_wait_s", "Ambulances: avg waiting (s)"),
                                       ("amb_travel_s", "Ambulances: avg travel time (s)")]):
        m = df.groupby("scenario")[col].mean()
        sd = df.groupby("scenario")[col].std()
        ax.bar([n.split(" ", 1)[1] for n in m.index], m.values, yerr=sd.values, capsize=3,
               color=["#9e9e9e", "#4c78a8", "#4c78a8", "#f58518", "#f58518"])
        ax.set_title(title, fontsize=10)
        ax.tick_params(axis="x", labelrotation=30, labelsize=8)
        for lab in ax.get_xticklabels():
            lab.set_ha("right")
        ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle(f"Mean ± sd over seeds {SEEDS}, 30 ambulances/hour", fontsize=10)
    fig.tight_layout()
    fig.savefig(ROOT / "results" / "comparison.png", dpi=150)
    print("saved results/comparison.png")


if __name__ == "__main__":
    main()
