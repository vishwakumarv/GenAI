"""Basic checks.  Run:  python -m pytest -q tests"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from llm_coordinator import MockCoordinator, validate  # noqa: E402
from traffic import ALL_RED_STATE, PHASES, SUMO, TLS, check_sequence, run, sumolib  # noqa: E402


def test_network_has_one_controller_and_45s_cycle():
    net = sumolib.net.readNet(str(SUMO / "net.net.xml"), withPrograms=True)
    assert [t.getID() for t in net.getTrafficLights()] == [TLS]
    phases = net.getTLS(TLS).getPrograms()["0"].getPhases()
    assert sum(p.duration for p in phases) == 45
    assert [p.state for p in phases if "y" not in p.state and "G" in p.state] == [PHASES["NS"], PHASES["EW"]]


def test_safety_check_catches_bad_sequence():
    assert check_sequence([(0, PHASES["NS"]), (1, PHASES["EW"])])          # no yellow
    assert check_sequence([(0, PHASES["NS"]), (17, PHASES["NS"].replace("G", "y").replace("g", "y")),
                           (20, ALL_RED_STATE), (21, PHASES["EW"])])         # all-red too short
    assert not check_sequence([(0, PHASES["NS"]), (17, PHASES["NS"].replace("G", "y").replace("g", "y")),
                               (20, ALL_RED_STATE), (22, PHASES["EW"])])


def test_fixed_controller_is_safe():
    r = run(1, "fixed", False, None, end=900)
    assert r["safety_problems"] == 0


def test_ambulances_get_priority_safely():
    for amb in ["emergency.rou.xml", "emergency_mixed.rou.xml"]:
        off = run(2, "fixed", False, amb)
        on = run(2, "fixed", True, amb)
        assert on["safety_problems"] == 0
        assert on["amb_entered_not_green"] == 0
        assert on["amb_wait_s"] < off["amb_wait_s"]


def test_llm_answers_are_validated():
    sit = {"movement_is_green": False}
    assert validate('{"action": "PREEMPT"}', sit) == "PREEMPT"
    assert validate('{"action": "HOLD"}', sit) is None              # can't hold a red
    assert validate('{"action": "SET_ALL_GREEN"}', sit) is None
    assert validate("just preempt", sit) is None
    m = MockCoordinator()
    for _ in range(8):
        action, _src = m.decide(sit)
        assert action == "PREEMPT"
    assert m.rejected > 0


def test_mock_llm_full_run_is_safe():
    r = run(3, "fixed", True, "emergency_mixed.rou.xml", coordinator=MockCoordinator())
    assert r["safety_problems"] == 0 and r["amb_entered_not_green"] == 0
