# Boost-Phase Stability Issues

Open issues identified while investigating SIH boost instability (~3 s after launch). Each item is a **hypothesis**, not a confirmed root cause. Use the discrimination experiments at the end to narrow the stack.

## Evidence baseline

Primary log: `logs/sim/2026-06-27/boost-upright-v2-20260627T020707Z/02_07_14.ulg`  
Profile: `config/flight_profiles/lander_boost_upright.json` (`RK_GD_BOOST_FULL=1`, zero lateral guidance)

| Time | Observation |
|------|-------------|
| t ≈ 2.73 s | `tv3_allocator_status` first published — LM path activates |
| t ≈ 2.73 s | Rail exit (~3.5 m rail; boost begins ~1 s after launch) |
| t ≈ 3 s | Torque demand ~16 Nm; LM **0% converged**, always **12/12** iterations |
| t ≈ 3 s | Residual torque **~15 Nm** (tol **0.15 Nm**); residual thrust **~40 N** |
| t ≈ 3 s | ~16° collective yaw splay on all engines; pitch TVC ~2.6° |
| t ≈ 5 s | Attitude quaternion `q[0]` → ~0.1 (large departure from launch reference) |

Related runs: `boost-att-v3-*` (attitude-only boost), `boundary-handling-*` (hover window).

## Likely causal stack (working theory)

```text
Rail exit (~2.7 s)
  → LM + collective splay path activates
  → Attitude integrator windup (physics frozen on rail) + torque spike
  → required_thrust vs chamber mismatch → collective splay on yaw axis (~16°)
  → Attitude demands yaw torque; collective splay cannot produce yaw moment
  → LM fails every cycle (~15 Nm torque residual) → fallback holds stale gimbals
  → Attitude departs → rates grow → gyro clip / failsafe
```

Hover-window thrust lowering amplifies the same failure mode later in flight; it is not required to trigger the initial ~3 s onset.

---

## A. Rail exit / mode handoff

### BS-001 — Hard discontinuity at `rail_exit`

**Confidence:** high  
**Status:** open

LM mixing and collective splay run only when `sequence_complete && rail_exit`. Before rail exit, gimbal commands stay at zero; immediately after, the full LM + splay path activates. Allocator logging begins at t ≈ 2.73 s, coincident with instability onset.

**Code:** `tv3_control_mixer.cpp` (`can_apply_splay`), `tv3_sih.cpp` (rail constraint), `tv3_mode_manager.cpp` (`_rail_exit`).

---

### BS-002 — Attitude integrator windup on rail

**Confidence:** high  
**Status:** open

SIH freezes attitude integration on rail (`skip_attitude_integration = true`) while the attitude controller continues to integrate rate error. When rail releases, stored integrator state and zero initial body rates can produce a large torque spike.

**Code:** `tv3_sih.cpp`, `tv3_attitude.cpp` (integrator update has no rail-aware anti-windup).

---

### BS-003 — Rail vs free-flight gain scheduling

**Confidence:** medium  
**Status:** open

During powered flight, boost gains (`RK_ATT_P_BOOST`, `RK_RATE_P_BOOST`) apply even while still on rail. There is no integrator reset or gain reduction tied to `rail_exit` itself (only `rail_mode` affects pre-boost non-powered paths, which are overridden in boost).

**Params:** `RK_ATT_P_RAIL`, `RK_ATT_P_BOOST`, `RK_RATE_P_RAIL`, `RK_RATE_P_BOOST`.

---

## B. Thrust guidance / scheduling

### BS-004 — `RK_GD_BOOST_FULL` still throttles via collective splay

**Confidence:** high  
**Status:** open

Boost-full sets `required_thrust_n = available` from the motor reference catalog, not measured chamber thrust. When `Σ chamber > available`, collective splay activates to reduce net axial thrust. At t ≈ 3 s: chamber ≈ 35 N, required ≈ 34 N → splay ≈ 16° (`acos(34/35)`).

**Code:** `tv3_guidance.cpp` (`update_commanded_thrust`), `tv3_control_mixer.cpp` (`collective_throttle_yaw_deg`).

---

### BS-005 — Expected vs measured thrust divergence

**Confidence:** high  
**Status:** open

Guidance `available_thrust_n` comes from `tv3_motor_reference.expected_thrust_n`. The mixer LM uses per-engine `filtered_thrust_n`. Filtered thrust peaks ~17.5 N per engine at t ≈ 2 s then falls to ~12 N by t ≈ 3 s during boost. Any catalog vs filtered vs measured mismatch drives splay and LM axial error.

**Code:** `tv3_guidance.cpp` (`update_thrust_envelope`), `tv3_control_mixer.cpp` (`engine_chamber_thrust_n`).

---

### BS-006 — Hover-window thrust lowering (amplifier)

**Confidence:** high (as amplifier, not sole trigger)  
**Status:** open

Hover-window altitude scheduling reduces `required_thrust_n` as the vehicle approaches `hold_alt_m`. SIH comparison: hover profile rates run away (1000+ °/s) while boost-full plateaus ~70 °/s. Same LM fallback signature; thrust modulation makes it worse.

**Profile:** `config/flight_profiles/lander_hover_window.json`.

---

### BS-007 — Axial thrust as a hard LM constraint

**Confidence:** high  
**Status:** open

LM treats axial thrust as a weighted residual (`RK_ALLOC_THR_W`, 5% thrust tolerance). With large collective splay, achieving commanded axial thrust and torque simultaneously may be infeasible. Log: ~40 N axial residual at t ≈ 3 s while torque residual ~15 Nm.

**Code:** `tv3_gimbal_lm.cpp` (`evaluate_residual`, `converged`).

---

## C. Torque allocation / LM solver

### BS-008 — LM never converges (100% fallback)

**Confidence:** high  
**Status:** open

Every logged cycle in boost-upright-v2: 12 iterations, `converged=false`, `used_fallback_solution=true`. Torque residual ~15 Nm vs demand ~16 Nm — allocator delivers almost no useful torque relative to demand.

**Topic:** `tv3_allocator_status`.

---

### BS-009 — Collective splay cannot produce yaw torque

**Confidence:** high  
**Status:** open

Attitude demands yaw torque up to ±16 Nm (`RK_TQ_Y_MAX`). Log shows identical yaw on all three engines (~16.7°) — pure collective splay. Symmetric secondary-axis deflection reduces net thrust but does not generate net yaw moment; differential yaw is needed for yaw torque.

**Code:** `tv3_control_mixer.cpp` (splay applied equally), `tv3_gimbal_plant.cpp`.

---

### BS-010 — Conflicting objectives in one LM solve

**Confidence:** high  
**Status:** open

Single LM solve simultaneously targets: 3-axis torque, axial thrust, symmetric-splay regularization, and gimbal limits — with 6 DOFs (3 engines × pitch + yaw) where yaw is partially consumed for thrust throttling. Problem may be inconsistent or poorly conditioned in boost.

**Code:** `tv3_gimbal_lm.cpp`, `tools/tv3_control_allocator.py` (host mirror).

---

### BS-011 — Fallback holds stale gimbals

**Confidence:** high  
**Status:** open

On LM failure, previous primary/yaw solution is held (`used_fallback_solution=true`). Creates lag between torque demand and gimbal response, enabling limit cycles and growing attitude error.

**Code:** `tv3_control_mixer.cpp` (post-`solve_gimbal_lm` fallback branch).

---

### BS-012 — LM tuning inadequate for flight regime

**Confidence:** medium  
**Status:** open

Bench tuning (`RK_ALLOC_TOL=0.15 Nm`, `RK_ALLOC_THR_W=1.0`) may suit hover oracle cases but flight demands ~16 Nm. FD step `RK_ALLOC_FD_EPS=0.01` rad may be poor at large yaw deflections (~16°). Convergence gate in CI mostly exercises zero-torque thrust ladder.

**Params:** `RK_ALLOC_*`. **Tests:** `tests/test_gimbal_lm_convergence.py`.

---

### BS-013 — Convergence criterion mismatch

**Confidence:** medium  
**Status:** open

Convergence requires **both** torque and thrust residuals below tolerance. During boost, torque-only convergence (deferring axial thrust tracking) might behave better but is not supported.

**Code:** `tv3_gimbal_lm.cpp` (`converged`).

---

## D. Attitude control

### BS-014 — Launch reference ≠ “straight up” in SIH

**Confidence:** medium-high  
**Status:** open

SIH initializes at 90° pitch (`AxisAngle(Y, π/2)`). Attitude captures that as `_launch_reference_q`. “Upright” profiles hold **rail attitude**, not world-vertical. After rail exit, q₀ falls from ~0.71 to ~0.1 by t ≈ 5 s — large departure from reference.

**Code:** `tv3_sih.cpp` (initial `_att_q`), `tv3_attitude.cpp` (`_launch_reference_q`).

---

### BS-015 — Boost gains may be too aggressive for TVC-limited plant

**Confidence:** medium  
**Status:** open

`RK_ATT_P_BOOST=8`, `RK_RATE_P_BOOST=2`, integrator limit 15 Nm. Controller demands can exceed what gimbals physically deliver given BS-008/BS-009 → saturation → oscillation.

**Params:** lander manifest / `RK_ATT_*`, `RK_RATE_*`, `RK_INT_LIM_BOOST`.

---

### BS-016 — Guidance envelope gating zeros torque

**Confidence:** medium  
**Status:** open

When `control_solution_valid=false`, attitude scales torque to zero. Hover profile intermittently clips control. Boost-att mode (`RK_GD_BOOST_ATT=1`) removes gating and performed **worse** (~250 °/s vs ~70 °/s for boost-full) — suggests ungated torque + LM failure is more violent, not that gating is the root fix.

**Code:** `tv3_attitude.cpp` (`control_envelope_valid`).

---

### BS-017 — Roll/yaw coupling on triangular engine ring

**Confidence:** medium  
**Status:** open

Three engines on a 120° ring: pitch attitude error may project into roll/yaw torque demands via geometry. Log shows small roll torque, large saturated yaw torque — may reflect ring layout and SIH initial attitude.

**Geometry:** `CA_RK_G*` params, `config/vehicles/tv3_lander_v1.json`.

---

## E. Dual control path / architecture

### BS-018 — PX4 allocator vs TV3 LM handoff

**Confidence:** medium  
**Status:** open

Pre-rail: engine commands come from `actuator_servos` (PX4 control allocator). Post-rail: TV3 LM overwrites commands. `actuator_servos` logged as NaN in boost-upright-v2 — PX4 allocator may be inactive or unwired for lander SIH, making pre-rail behavior unclear.

**Topics:** `actuator_servos`, `control_allocator_status`, `tv3_engine_command`.

---

### BS-019 — LM init seeded from PX4 servos

**Confidence:** medium  
**Status:** open

First LM cycle seeds from `actuator_servos` pitch/yaw plus splay offset. If servos are zero or non-finite, initialization is poor. Warm-start only helps after a converged step (never observed in boost logs).

**Code:** `tv3_control_mixer.cpp` (`init_p`, `init_y`, `use_warm_start`).

---

### BS-020 — Control rate mismatch

**Confidence:** medium-low  
**Status:** open

Attitude updates on `vehicle_attitude` (~100 Hz effective), mixer at 100 Hz (`ScheduleOnInterval(10_ms)`), SIH at 400 Hz. LM failure + hold-previous at 100 Hz imposes ~10 ms actuation delay.

---

## F. Plant / simulation fidelity

### BS-021 — SIH rail model is kinematic, not dynamic

**Confidence:** medium  
**Status:** open

Rail constraint instantly zeros lateral motion and freezes attitude integration; no gradual rail clearance, compliance, or side loads. Real rail exit would be smoother.

**Code:** `tv3_sih.cpp` (on-rail branch).

---

### BS-022 — Thrust model transients

**Confidence:** medium  
**Status:** open

Filtered thrust overshoots then decays early in boost (per-engine ~17.5 N → ~12 N). Allocator uses instantaneous filtered thrust in the wrench Jacobian while attitude assumes quasi-steady thrust.

**Topic:** `tv3_engine_state.filtered_thrust_n`.

---

### BS-023 — Inertia / COM / gimbal geometry uncertainty

**Confidence:** medium  
**Status:** open

Preliminary manifest values flagged in vehicle JSON. Wrong `CA_RK_G*` geometry vs SIH plant → Jacobian predicts incorrect torque-per-degree, LM steps fail to converge.

**Config:** `config/vehicles/tv3_lander_v1.json`, generated `CA_RK_G*`.

---

### BS-024 — No rate damping in nominal SIH

**Confidence:** low-medium  
**Status:** open

`RK_SIH_RATE_DAMP=0` by default — no dissipative term once angular energy is injected. Optional param exists for numerical stability only.

**Code:** `tv3_sih.cpp`, param `RK_SIH_RATE_DAMP`.

---

## G. Propulsion / sequencing

### BS-025 — Multi-engine ignition timing

**Confidence:** medium-low  
**Status:** open

LM requires `sequence_complete`. Three-engine stagger may align with the ~2–3 s timeline. Thrust asymmetry during stagger could seed attitude error before full control authority.

**Code:** `tv3_mode_manager.cpp` (engine sequence), `tv3_motor_model`.

---

### BS-026 — Engine count / ignition mask mismatch

**Confidence:** low-medium  
**Status:** open

Engine index 3 commands stay at 0° in logs (3 of 4 slots active?). Verify ignition mask, `RK_ENG_COUNT`, and geometry cover the same active set.

**Topics:** `tv3_engine_command`, `tv3_status.ignition_mask`.

---

### BS-027 — Motor burn fraction resets thrust state ordering

**Confidence:** low  
**Status:** open

`update_thrust_envelope()` sets `_last_required_thrust_n = 0` every cycle before phase/thrust scheduling. Phase logic runs after in the same tick, but any future reorder or subscriber reading mid-cycle could see transient zero required thrust.

**Code:** `tv3_guidance.cpp` (`Run()` call order).

---

## H. Sensing / failsafe

### BS-028 — Gyro clipping / STALE (symptom)

**Confidence:** high (as consequence)  
**Status:** open

Gyro clipping and `angular velocity no longer valid` appear after rates diverge — likely **termination**, not initial cause. Still blocks recovery and obscures later-cycle logs.

**Log symptom:** `WARN [vehicle_imu] Gyro 0 clipping`, failsafe activation.

---

### BS-029 — Ground-truth vs EKF position (unlikely contributor)

**Confidence:** low  
**Status:** open

Sim profiles use `sim_groundtruth_fallback=1` for guidance position. Unlikely to cause boost attitude instability for current upright profiles (zero lateral setpoints), but could affect waypoint/hover scenarios.

**Param:** `RK_GD_SIM_GT`.

---

## Discrimination experiments

Run in SIH with `lander_boost_upright.json` unless noted. Archive ULog under `logs/sim/` and compare against boost-upright-v2 baseline.

| ID | Experiment | Primary issues tested | Pass/fail signal |
|----|------------|----------------------|------------------|
| **EXP-01** | Log rail-exit boundary: add or post-process `rail_exit`, integrator (`tv3_attitude` print_status), first LM cycle inputs/outputs, `required_thrust_n`, `Σ chamber`, splay deg | BS-001, BS-002, BS-004 | Timestamp alignment: allocator start ≡ rail_exit ± one cycle |
| **EXP-02** | **Disable collective splay during boost** (force splay = 0; axial target = chamber) | BS-004, BS-007, BS-009, BS-010 | Rates stay bounded past 5 s; residual thrust drops; yaw splay ≈ 0 |
| **EXP-03** | **Reset attitude integrator on `rail_exit`** | BS-002, BS-003 | Torque spike at rail exit absent or much smaller |
| **EXP-04** | **LM torque-only mode during boost** (drop axial thrust from residual / convergence) | BS-007, BS-010, BS-013 | LM convergence rate > 0%; lower torque residual |
| **EXP-05** | **`required = active_chamber_thrust_n()` in boost-full** instead of catalog `available` | BS-004, BS-005 | Collective splay ≈ 0 when chamber is authoritative |
| **EXP-06** | **Reduce boost gains** (e.g. halve `RK_ATT_P_BOOST`, `RK_RATE_P_BOOST`) | BS-015 | Lower peak rates; may mask vs fix — compare with EXP-02 |
| **EXP-07** | **SIH initial attitude vertical** (identity or rail-aligned vertical) if “straight up” is intent | BS-014 | Smaller attitude error for upright profile; isolates reference frame |
| **EXP-08** | **Delay LM until N ms after rail_exit** (soft handoff) | BS-001, BS-018, BS-019 | Onset shifts by delay; implicates handoff timing |
| **EXP-09** | **Differential yaw only for torque; reserve collective splay for thrust axis** (arch change) | BS-009, BS-010 | Yaw torque track improves; splay decoupled from yaw TVC |
| **EXP-10** | **Hover window with `RK_GD_BOOST_FULL=1` through 8 m** | BS-006 vs BS-004 | Hover instability reduced if thrust lowering was dominant amplifier |
| **EXP-11** | Sweep `RK_ALLOC_THR_W` down (0.1, 0.01) and `RK_ALLOC_TOL` up during boost | BS-012, BS-013 | Any convergence; trade thrust vs torque tracking |
| **EXP-12** | Enable `RK_SIH_RATE_DAMP` > 0 | BS-024 | Energy dissipation; distinguishes plant damping from control fix |

### Recommended order

1. **EXP-05** + **EXP-02** — fastest firmware toggles; directly test highest-confidence thrust/splay hypotheses.  
2. **EXP-03** — cheap attitude fix for rail windup.  
3. **EXP-04** — tests LM feasibility separate from thrust scheduling.  
4. **EXP-01** — logging to confirm alignment before larger arch changes (EXP-09).

---

## Experiment results (2026-06-27)

Firmware changes shipped in this pass:

| Change | Maps to |
|--------|---------|
| Conditional collective splay off when `required ≥ chamber` during boost | EXP-02 |
| Attitude integrator frozen on rail + reset on `rail_exit` | EXP-03 |
| `RK_GD_BOOST_FULL` sets `required_thrust_n` from live chamber sum | EXP-05 |
| Cap servo/LM limits at `RK_TVC_MAX_DEG` / `RK_SPLAY_MAX_DEG` (not raw 90° CA params) | BS-023 |
| Symmetric ±`RK_TVC_MAX` LM yaw limits during boost | BS-009 (partial) |
| LM active from `sequence_complete` during boost (not only after `rail_exit`) | BS-001 (partial) |
| LM `converged()` skips thrust check when `thrust_weight ≤ 0` | EXP-04 (helper, not used in flight) |

Reference logs:

| Run | Profile | Peak rate | t≈4–6 s | LM conv |
|-----|---------|-----------|---------|---------|
| `boost-upright-v2` (baseline) | boost_upright | 92 °/s | ~70 °/s | 0% |
| `stability-final-hover-030545` | hover_window | 136 °/s | **12–30 °/s** | 0% |
| `stability-final2-boost-030821` | boost_upright | 311 °/s | ~310 °/s | 0% |

### Per-experiment verdict

| ID | Result | Notes |
|----|--------|-------|
| **EXP-01** | Partial | LM now starts ~2.1–2.8 s (sequence complete), still 0% converged |
| **EXP-02** | Partial | Boost-upright: splay ≈ 0 ° as intended; hover still uses splay when `required < chamber` |
| **EXP-03** | Shipped | Integrator freeze + rail-exit reset in `tv3_attitude` |
| **EXP-04** | Failed in isolation | `thrust_weight=0` alone increased divergence; reverted for flight |
| **EXP-05** | Shipped | Guidance uses chamber sum for boost-full |
| **EXP-06** | Not run | Deferred |
| **EXP-07** | Not run | SIH 90° pitch rail attitude retained |
| **EXP-08** | Not run | Early LM on boost tried instead |
| **EXP-09** | **Still required** | LM still commands identical pitch/yaw on all engines; yaw torque unsatisfied |
| **EXP-10** | Improved hover | Hover early phase much calmer without profile change |
| **EXP-11** | Not run | LM still 0% converged |
| **EXP-12** | Not run | Deferred |

### Issue status updates

| ID | Status | Notes |
|----|--------|-------|
| BS-004 | **mitigated** | Boost-full uses chamber; splay gated when `required ≥ chamber` |
| BS-005 | **mitigated** | Same |
| BS-006 | **mitigated** | Hover t≈4–6 s: 12–30 °/s vs 2624 °/s baseline |
| BS-002 | **mitigated** | Integrator freeze on rail |
| BS-003 | **mitigated** | Same |
| BS-023 | **mitigated** | TVC cap in control mixer |
| BS-008 | **open** | LM convergence still 0% |
| BS-009 | **open** | Yaw remains symmetric; needs EXP-09 |
| BS-001 | **open** | Boost-upright still diverges ~3 s |

**Next priority:** EXP-09 — decouple collective splay from differential yaw TVC in the LM (or route boost torque through PX4 CA primary axes only).

---

## 2026-06-27 continue pass (torque gating, direct/LM policy, yaw authority, small-angle cap)

Firmware changes:
- Torque output no longer gated by guidance `control_solution_valid` during ignition/boost (un-gate for "point straight up").
- Guidance `boost_full` forces thrust+control valid.
- LM re-solve is active for all powered+sequence+ready (broad), not only when splay > 0.2° (preserves accurate wrench vs. actual chamber for hover early phase).
- Hard 8° TVC clamp on commanded pitch/yaw and on LM limits while `powered_boost_active()` (prevents 30-50° collective cant).
- During pure boost hold (`boost_full` or `boost_att`), yaw torque demand and yaw integrator are zeroed (the 3-engine ring has negligible authority on roll-around-thrust; asking for it saturated the allocator and starved transverse).
- (Temporarily) SIH rate damp default exercised at 0.8; no material effect.

Reference logs from this pass:
- `continue-boost_upright-153240`, `continue-hover_window-153303`, `zero-yaw-tq-154120`, `damp08-boost-154249`

Observations (boost_upright):
- wz (roll-around-thrust rate) begins growing immediately around rail exit (t~2.6 already ~45 °/s, 195 °/s by 2.8) even with commanded yaw torque = 0 and very small TVC (~0.8° primary at t~2.8).
- Yaw torque demand removed → residual torque dropped (from ~16 Nm to ~3-9 Nm), but wz growth continued.
- With the 8° cap, commanded primary stayed in range; still insufficient counter-torque to arrest the rates that appear at release.
- Peak rates remained ~250-290 °/s; same family as prior "bad" boost runs.

Observations (hover_window with current stack):
- Early hover phase was previously calmed by the integrator + chamber + no-unwanted-splay + LM-while-powered changes.
- The broad LM policy + un-gate (not triggered for hover) kept the "best prev" behavior; splay still engages when guidance lowers required in the hover window.

### Updated verdicts

| ID | Result | Notes |
|----|--------|-------|
| EXP-06 (gains / limits) | Partial | 8° hard cap shipped for boost; does not prevent onset but bounds the cant. Gains still the root driver for the post-release transient. |
| EXP-09 (differential yaw) | Still open | Zeroing the yaw demand component is a workaround; the plant/CA effectiveness for the lander ring produces little to no roll-around-thrust moment from the primary/secondary deflections at these angles. |
| EXP-12 (SIH damp) | No effect | 0.8 Nm/(rad/s) changed nothing; the growth is not viscous-energy driven. |

### Issue status deltas

| ID | Status | Notes |
|----|--------|-------|
| BS-009 | Confirmed core for yaw | Even with demand removed, wz appears at rail exit → authority gap or disturbance at release. |
| BS-016 | Mitigated for torque | Un-gate + guidance force-valid; torque no longer drops to 0 at the critical handoff. |
| BS-014 / rail state | Elevated | wz growth with near-zero TVC commands right at release points to either (a) kinematic unlock / thrust line vs CG moment or (b) launch reference not matching the free-flight state the instant the rail constraint is removed. |

**Current diagnosis for pure boost upright:** The ~3 s instability is not (primarily) thrust scheduling or collective splay or yaw torque demand. A rate (mainly wz) appears at rail exit; the available TVC (even before the 8° cap) does not generate enough opposing moment in time. The fixes that tamed the hover window (integrator hygiene, chamber-true thrust, no spurious splay, LM with real thrust) are necessary but not sufficient for a "straight-up boost hold" after a kinematic rail.

**Next actions to consider:**
1. Instrument rail exit precisely: correlate `rail_exit`, `vehicle_angular_velocity`, `vehicle_attitude`, `launch_reference`, and engine chamber at the exact transition (add a one-off print or dedicated ulog topic).
2. At rail exit in SIH (or attitude), explicitly zero body rates for the isolation "boost upright" profile to test "clean release" hypothesis.
3. Try a longer rail or different initial orientation for the lander in SIH to separate "release transient" from "boost control".
4. Revisit the lander geometry / CA effectiveness matrix for roll-around-thrust authority (or accept bounded wz for this vehicle).
5. If the disturbance is repeatable, feedforward a counter torque at release or schedule a brief "rate capture" mode.

---

## Related docs and profiles

- [Control mixer](control_mixer.md) — LM solver, boost thrust modes (`RK_GD_BOOST_FULL`, `RK_GD_BOOST_ATT`)
- [Simulation](simulation.md) — SIH workflow, known limitations
- Profiles: `lander_boost_upright.json`, `lander_boost_attitude_only.json`, `lander_hover_window.json`
