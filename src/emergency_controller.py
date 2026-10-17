"""Predictive ambulance signal pre-emption for the SUMO A0 intersection.

The normal four-approach signal cycle is exactly 45 seconds. Emergency pre-emption
is temporary and does not reset that clock: after the ambulance clears, the
controller returns to the phase implied by simulation_time % 45.

This is a deterministic safety controller around the SUMO/TraCI simulation. It
uses vehicle position and speed to estimate arrival time; it does not claim to
be a trained RL predictor. The RL policy can continue to be trained/evaluated
separately and should not bypass this safety layer in a live deployment.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

# Main junction movement groups in net.net.xml's 20-character A0 signal state.
APPROACH_PHASE = {
    "top0A0": 0,
    "right0A0": 1,
    "bottom0A0": 2,
    "left0A0": 3,
}
OUTGOING_EDGES = {"A0top0", "A0right0", "A0bottom0", "A0left0"}
TLS_ID = "A0"
GLOBAL_CYCLE_SECONDS = 45.0
GREEN_SECONDS = 8.25
YELLOW_SECONDS = 3.0
ALL_RED_SECONDS = 1.0
DETECTION_DISTANCE_M = 110.0
PREDICTION_HORIZON_S = 12.0
EMERGENCY_TYPE = "emergency"


def green_state(phase: int) -> str:
    """Return the safe green state for one of the four 5-link movement groups."""
    chars = ["r"] * 20
    start = phase * 5
    chars[start:start + 5] = ["G"] * 5
    return "".join(chars)


def yellow_state(phase: int) -> str:
    chars = ["r"] * 20
    start = phase * 5
    chars[start:start + 5] = ["y"] * 5
    return "".join(chars)


ALL_RED_STATE = "r" * 20
GREEN_STATES = tuple(green_state(i) for i in range(4))
YELLOW_STATES = tuple(yellow_state(i) for i in range(4))


def normal_signal_at(sim_time: float) -> tuple[int, str, float]:
    """Return (phase index, state, seconds remaining) on the global 45s clock."""
    cycle_pos = sim_time % GLOBAL_CYCLE_SECONDS
    phase_block = GREEN_SECONDS + YELLOW_SECONDS
    phase = min(int(cycle_pos // phase_block), 3)
    within = cycle_pos - phase * phase_block
    if within < GREEN_SECONDS:
        return phase, GREEN_STATES[phase], GREEN_SECONDS - within
    return phase, YELLOW_STATES[phase], phase_block - within


@dataclass
class EmergencyRequest:
    vehicle_id: str
    phase: int
    eta_seconds: float
    distance_m: float
    inside_intersection: bool


class EmergencySignalController:
    """TraCI signal override with approach prediction and blocked-ambulance support."""

    def __init__(self, traci_module, tls_id: str = TLS_ID):
        self.traci = traci_module
        self.tls_id = tls_id
        self.stage = "normal"  # normal, approach_yellow, preemption_clearance, priority, release_yellow, release_clearance
        self.stage_until = 0.0
        self.active_vehicle: Optional[str] = None
        self.active_phase: Optional[int] = None
        self.last_phase_by_vehicle: dict[str, int] = {}
        self.preemptions = 0
        self.emergency_priority_seconds = 0.0

    def _request_for_vehicle(self, vehicle_id: str, now: float) -> Optional[EmergencyRequest]:
        try:
            road_id = self.traci.vehicle.getRoadID(vehicle_id)
            route = self.traci.vehicle.getRoute(vehicle_id)
            route_index = self.traci.vehicle.getRouteIndex(vehicle_id)
            speed = max(0.0, float(self.traci.vehicle.getSpeed(vehicle_id)))
            position = float(self.traci.vehicle.getLanePosition(vehicle_id))
            lane_id = self.traci.vehicle.getLaneID(vehicle_id)
            lane_length = float(self.traci.lane.getLength(lane_id))
        except Exception:
            return None

        if road_id in APPROACH_PHASE:
            phase = APPROACH_PHASE[road_id]
            self.last_phase_by_vehicle[vehicle_id] = phase
            distance = max(0.0, lane_length - position)
            # Use a conservative speed floor only for ETA estimation; it does not
            # force the simulated ambulance to move.
            eta = distance / max(speed, 2.0)
            if distance <= DETECTION_DISTANCE_M or eta <= PREDICTION_HORIZON_S:
                return EmergencyRequest(vehicle_id, phase, eta, distance, False)
            return None

        if road_id.startswith(":"):
            # SUMO internal lanes represent a vehicle physically inside the junction.
            phase = self.last_phase_by_vehicle.get(vehicle_id)
            if phase is not None:
                return EmergencyRequest(vehicle_id, phase, 0.0, 0.0, True)
            return None

        # Once the ambulance is on an outgoing edge, it has cleared the junction.
        if road_id in OUTGOING_EDGES or (route_index > 0 and road_id == route[route_index]):
            return None
        return None

    def _find_request(self, now: float) -> Optional[EmergencyRequest]:
        requests = []
        for vehicle_id in self.traci.vehicle.getIDList():
            try:
                if self.traci.vehicle.getTypeID(vehicle_id) != EMERGENCY_TYPE:
                    continue
            except Exception:
                continue
            request = self._request_for_vehicle(vehicle_id, now)
            if request is not None:
                requests.append(request)
        return min(requests, key=lambda item: item.eta_seconds) if requests else None

    def _set_state(self, state: str) -> None:
        self.traci.trafficlight.setRedYellowGreenState(self.tls_id, state)

    def _active_vehicle_cleared(self) -> bool:
        if not self.active_vehicle:
            return True
        try:
            if self.active_vehicle not in self.traci.vehicle.getIDList():
                return True
            road_id = self.traci.vehicle.getRoadID(self.active_vehicle)
            return road_id in OUTGOING_EDGES
        except Exception:
            return True

    def step(self, now: Optional[float] = None) -> dict:
        """Run one control update after each traci.simulationStep()."""
        if now is None:
            now = float(self.traci.simulation.getTime())
        request = self._find_request(now)

        if self.stage == "normal":
            if request is None:
                phase, state, _ = normal_signal_at(now)
                self._set_state(state)
                return {"mode": "normal", "phase": phase, "global_cycle_s": now % GLOBAL_CYCLE_SECONDS}

            self.active_vehicle = request.vehicle_id
            self.active_phase = request.phase
            normal_phase, normal_state, _ = normal_signal_at(now)
            self.preemptions += 1
            # If the normal phase is green, honour a yellow clearance before all-red.
            if "G" in normal_state:
                self._set_state(YELLOW_STATES[normal_phase])
                self.stage = "approach_yellow"
                self.stage_until = now + YELLOW_SECONDS
            else:
                self._set_state(ALL_RED_STATE)
                self.stage = "preemption_clearance"
                self.stage_until = now + ALL_RED_SECONDS
            return {"mode": "preempting", "ambulance": self.active_vehicle, "phase": self.active_phase}

        if self.stage == "approach_yellow":
            if now < self.stage_until:
                return {"mode": self.stage, "ambulance": self.active_vehicle}
            self._set_state(ALL_RED_STATE)
            self.stage = "preemption_clearance"
            self.stage_until = now + ALL_RED_SECONDS
            return {"mode": self.stage, "ambulance": self.active_vehicle}

        if self.stage == "preemption_clearance":
            if now < self.stage_until:
                self._set_state(ALL_RED_STATE)
                return {"mode": self.stage, "ambulance": self.active_vehicle}
            # Refresh phase from current ambulance location in case it changed lanes.
            if request and request.vehicle_id == self.active_vehicle:
                self.active_phase = request.phase
            self._set_state(GREEN_STATES[self.active_phase or 0])
            self.stage = "priority"
            self.stage_until = now
            return {"mode": "ambulance_priority", "ambulance": self.active_vehicle, "phase": self.active_phase}

        if self.stage == "priority":
            self.emergency_priority_seconds += 1.0
            if not self._active_vehicle_cleared():
                # Keep priority active while approaching, blocked, or physically in
                # the junction. This covers an ambulance stuck mid-intersection.
                latest = self._request_for_vehicle(self.active_vehicle, now)
                if latest is not None:
                    self.active_phase = latest.phase
                    self._set_state(GREEN_STATES[self.active_phase])
                return {"mode": "ambulance_priority", "ambulance": self.active_vehicle, "phase": self.active_phase}
            self._set_state(YELLOW_STATES[self.active_phase or 0])
            self.stage = "release_yellow"
            self.stage_until = now + YELLOW_SECONDS
            return {"mode": self.stage, "ambulance": self.active_vehicle}

        if self.stage == "release_yellow":
            if now < self.stage_until:
                return {"mode": self.stage, "ambulance": self.active_vehicle}
            self._set_state(ALL_RED_STATE)
            self.stage = "release_clearance"
            self.stage_until = now + ALL_RED_SECONDS
            return {"mode": self.stage}

        if self.stage == "release_clearance":
            if now < self.stage_until:
                self._set_state(ALL_RED_STATE)
                return {"mode": self.stage}
            # Restore by absolute simulation time, not by restarting the cycle.
            self.active_vehicle = None
            self.active_phase = None
            self.stage = "normal"
            phase, state, _ = normal_signal_at(now)
            self._set_state(state)
            return {"mode": "normal_resumed", "phase": phase, "global_cycle_s": now % GLOBAL_CYCLE_SECONDS}

        # Defensive fallback: never leave the signal in an unknown state.
        self.stage = "normal"
        phase, state, _ = normal_signal_at(now)
        self._set_state(state)
        return {"mode": "normal", "phase": phase, "global_cycle_s": now % GLOBAL_CYCLE_SECONDS}
