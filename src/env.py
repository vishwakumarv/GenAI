
import traci

AMBULANCE_KEYWORDS = ("ambulance", "emergency")

def find_ambulance():
    """Return the first active ambulance vehicle ID."""
    for vehicle_id in traci.vehicle.getIDList():
        vehicle_type = traci.vehicle.getTypeID(vehicle_id).lower()
        if any(word in vehicle_id.lower() or word in vehicle_type
               for word in AMBULANCE_KEYWORDS):
            return vehicle_id
    return None


def ambulance_priority():
    """
    Return the ambulance's incoming lane, if present.
    """
    ambulance = find_ambulance()
    if ambulance is None:
        return None

    lane_id = traci.vehicle.getLaneID(ambulance)
    if lane_id.startswith(":"):
        return None

    return lane_id


def force_ambulance_green(tls_id, green_phase):
    """
    Force the ambulance approach to green.
    green_phase must be the existing SUMO phase for that approach.
    """
    traci.trafficlight.setPhase(tls_id, green_phase)
