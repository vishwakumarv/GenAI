"""Rebuild the network and traffic files (already included - only run this if you want to regenerate).

    python src/make_files.py

1. Network: same layout as before, but ONLY the centre junction A0 has a traffic light, with a
   45 s plan (NS 17 s, EW 18 s, 3 s yellow, 2 s all-red) and no U-turns at the light.
2. Traffic: randomTrips, one car every 2 s for 3600 s, seeds 1-5 (evaluation) and 100-119 (training).
3. Ambulances: obey traffic lights (no 'bluelight' device - with it SUMO lets them run red lights).
"""
import random
import subprocess
import sys

from traffic import SUMO, sumolib
import os

TOOLS = os.path.join(os.environ["SUMO_HOME"], "tools")
AMB_TYPE = ('    <vType id="emergency" vClass="emergency" guiShape="emergency" color="1,0,0" '
            'length="7" width="2.5" maxSpeed="20" speedFactor="1.3" speedDev="0"/>\n')
IN = {"N": "top0A0", "E": "right0A0", "S": "bottom0A0", "W": "left0A0"}
OUT = {"N": "A0top0", "E": "A0right0", "S": "A0bottom0", "W": "A0left0"}


def ambulances(name, trips):
    lines = ["<routes>\n", AMB_TYPE] + [
        f'    <trip id="ambulance_{k}" type="emergency" depart="{d}" departLane="best" departSpeed="max" '
        f'from="{a}" to="{b}"/>\n' for k, (d, a, b) in enumerate(trips)] + ["</routes>\n"]
    (SUMO / name).write_text("".join(lines))


if __name__ == "__main__":
    subprocess.run([sumolib.checkBinary("netgenerate"), "--grid", "--grid.number=1", "--grid.length=300",
                    "--grid.attach-length=200", "--default-junction-type", "priority", "--tls.set", "A0",
                    "--default.lanenumber", "2", "--default.lanewidth", "4.0", "--tls.layout", "opposites",
                    "--tls.cycle.time", "45", "--tls.yellow.time", "3", "--tls.allred.time", "2",
                    "--no-turnarounds.tls", "true", "-o", str(SUMO / "net.net.xml")], check=True)
    (SUMO / "demand").mkdir(exist_ok=True)
    for s in [1, 2, 3, 4, 5, 42] + list(range(100, 120)):
        subprocess.run([sys.executable, os.path.join(TOOLS, "randomTrips.py"), "-n", str(SUMO / "net.net.xml"),
                        "-r", str(SUMO / "demand" / f"routes_{s}.rou.xml"),
                        "-o", str(SUMO / "demand" / f"trips_{s}.trips.xml"), "-e", "3600", "-p", "2",
                        "--seed", str(s), "--validate", "--prefix", "veh"], check=True, stdout=subprocess.DEVNULL)
    (SUMO / "routes.rou.xml").write_text((SUMO / "demand" / "routes_42.rou.xml").read_text())
    ambulances("emergency_single.rou.xml", [(600, IN["N"], OUT["S"])])
    ambulances("emergency.rou.xml", [(10 + 120 * k, IN["N"], OUT["S"]) for k in range(30)])
    rng = random.Random(2024)
    mixed = []
    for k in range(30):
        a = rng.choice("NESW")
        b = rng.choice([x for x in "NESW" if x != a])
        mixed.append((10 + 120 * k, IN[a], OUT[b]))
    ambulances("emergency_mixed.rou.xml", mixed)
    print("done")
