
import os
import sys
import traci

# SUMO configuration
SUMO_CFG = r"sumo\config.sumocfg"

# UPDATE these to match your SUMO network
TLS_ID = "J1"

GREEN_PHASES = {
    "north": 0,
    "east": 2,
    "south": 4,
    "west": 6,
}

APPROACHES = {
    "north": ["north_in"],
    "east": ["east_in"],
    "south": ["south_in"],
    "west": ["west_in"],
}


def find_ambulance():
    for vid in traci.vehicle.getIDList():
        vehicle_type = traci.vehicle.getTypeID(vid).lower()

        if any(word in vid.lower() or word in vehicle_type
               for word in ("ambulance", "emergency")):
            return vid

    return None


def get_approach(lane_id):
    for approach, lanes in APPROACHES.items():
        if lane_id in lanes:
            return approach

    return None


def main():
    if not os.path.isfile(SUMO_CFG):
        print(f"SUMO config not found: {SUMO_CFG}")
        sys.exit(1)

    sumo_binary = "sumo-gui"

    try:
        traci.start([
            sumo_binary,
            "-c", SUMO_CFG,
            "--start"
        ])

        print("SUMO connected. Ambulance priority enabled.")

        previous_phase = None
        emergency_active = False

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

            ambulance = find_ambulance()

            if ambulance is None:
                if emergency_active and previous_phase is not None:
                    traci.trafficlight.setPhase(
                        TLS_ID, previous_phase
                    )
                    emergency_active = False
                    previous_phase = None
                    print("Ambulance cleared. Restoring signal.")

                continue

            lane_id = traci.vehicle.getLaneID(ambulance)

            # Ambulance is inside the junction.
            if lane_id.startswith(":"):
                continue

            approach = get_approach(lane_id)

            if approach is None:
                continue

            target_phase = GREEN_PHASES[approach]
            current_phase = traci.trafficlight.getPhase(TLS_ID)

            if not emergency_active:
                previous_phase = current_phase
                emergency_active = True
                print("Ambulance detected:", ambulance)

            if current_phase != target_phase:
                traci.trafficlight.setPhase(
                    TLS_ID, target_phase
                )

    except traci.exceptions.TraCIException as error:
        print("SUMO/TraCI error:", error)

    finally:
        if traci.isLoaded():
            traci.close()


if __name__ == "__main__":
    main()
