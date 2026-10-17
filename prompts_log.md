# Prompts log

One entry per prompt that mattered. Paste real errors. Failures are the evidence.

## Entry template

### [#] Module: <file>  |  Date: <date>  |  Model: <Claude / ChatGPT / Gemini + version>
**Prompt (verbatim or trimmed):**
> ...

**What I got:** (works / partly works / broken)

**What went wrong:** (full error text, or wrong behaviour, e.g. "LLM returned prose instead of JSON")

**My follow-up prompt / fix:**
> ...

**Outcome + lesson:** (one line you can reuse in the report)

---

## Entries

### 1. Module: baseline_fixed.py
**Prompt:**
> Write a TraCI script that runs SUMO with default fixed-time signals and records avg waiting time, queue length and emergency vehicle travel time to CSV.

**What I got:**

**What went wrong:**

**Outcome + lesson:**

### 2. Module: whole system (network, safety, controller, emergency, RL, LLM, evaluation) | Date: 2026-10-14 | Model: Claude (configured claude-opus-5-5)
**Prompt (trimmed):**
> Project master prompt (single 4-way junction, SUMO + TraCI + RL + LLM emergency coordinator) + "BUILD ME EVERYTHING AND GIVE ME".

**What I got:** works – full pipeline, 17 automated tests passing, evaluation over 5 unseen seeds.

**What went wrong (real findings during the build):**
- Original net had 5 traffic-light controllers; the 4 fringe ones only guarded edge U-turns (90 s cycle). A0 used a 110 s one-approach-at-a-time plan with no all-red.
- `has.bluelight.device` let ambulances run red lights: measured **7/30** entering on red (0/30 without the device). The original `emergency_travel_s = 23.0` therefore did not measure any priority system.
- About 24 % of the original `routes.rou.xml` trips were U-turns at the junction (435/1800); they became invalid on the rebuilt network.
- Bug in my own controller, caught by the independent SafetyMonitor: when an emergency request arrived during an all-red interval, the "retarget" code recomputed the all-red state and re-lit East links 1 s after conflicting West links turned red (28 `no_clearance` violations in one run). Fixed: a retarget keeps the current all-red display and its full clearance timer. Re-tested: 0 violations.
- "Resync" metric first fired on a momentary match. Changed to "matches the undisturbed plan for a full 45 s cycle".
- Training was lost once when the workspace restarted → added checkpoint + auto-resume to train_rl.py.

**Outcome + lesson:** Never trust a visually plausible run. An independent runtime safety monitor found a clearance bug that a normal-looking simulation hid, and a default SUMO device (bluelight) silently invalidated the original ambulance metric.
