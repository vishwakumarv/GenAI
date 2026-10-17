"""Optional LLM coordinator for ambulance priority.

The LLM only chooses between two actions: HOLD (keep the current green) or PREEMPT (give the
ambulance's approach green). Its answer is validated; anything invalid falls back to the simple rule.
It never writes signal states - the Controller does that.

Backends:
  OllamaCoordinator - free, local (install Ollama, then: ollama pull llama3.2:3b)
  MockCoordinator   - fake LLM with some deliberately bad answers, used to test the validation
"""
import json
import urllib.request

PROMPT = """You coordinate an ambulance at a traffic signal. Situation: {situation}
Reply ONLY with JSON: {{"action": "HOLD"}} if the ambulance's movement is already green,
otherwise {{"action": "PREEMPT"}}."""


def rule(situation):
    return "HOLD" if situation["movement_is_green"] else "PREEMPT"


def validate(answer, situation):
    """Return the action if it is allowed, else None."""
    try:
        action = json.loads(answer[answer.index("{"):answer.rindex("}") + 1])["action"]
    except (ValueError, KeyError, TypeError):
        return None
    if action not in ("HOLD", "PREEMPT"):
        return None
    if action == "HOLD" and not situation["movement_is_green"]:
        return None                       # holding a red would leave the ambulance waiting
    return action


class _Base:
    def __init__(self):
        self.calls = self.rejected = 0

    def decide(self, situation):
        self.calls += 1
        try:
            action = validate(self.ask(PROMPT.format(situation=json.dumps(situation))), situation)
        except Exception:
            action = None
        if action is None:
            self.rejected += 1
            return rule(situation), "rule-fallback"
        return action, "llm"


class OllamaCoordinator(_Base):
    def __init__(self, model="llama3.2:3b"):
        super().__init__()
        self.model = model

    def ask(self, prompt):
        body = json.dumps({"model": self.model, "prompt": prompt, "stream": False, "format": "json"}).encode()
        req = urllib.request.Request("http://localhost:11434/api/generate", body,
                                     {"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.loads(r.read())["response"]


class MockCoordinator(_Base):
    ANSWERS = ['{"action": "PREEMPT"}', "sure, preempt it!", '{"action": "SET_ALL_GREEN"}',
               '{"action": "HOLD"}']

    def __init__(self):
        super().__init__()
        self.i = 0

    def ask(self, prompt):
        self.i += 1
        return self.ANSWERS[self.i % len(self.ANSWERS)]
