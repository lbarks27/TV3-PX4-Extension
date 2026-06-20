#!/usr/bin/env python3
"""Report TV3 completion-roadmap progress from gate scripts and repo evidence."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
STATUS_PATH = REPO_ROOT / "config/completion_status.json"
STATUS_DOC_PATH = REPO_ROOT / "docs/completion_status.md"
VEHICLE_DIR = REPO_ROOT / "config/vehicles"
GROUND_LOG_DIR = REPO_ROOT / "logs/ground"
SIM_LOG_DIR = REPO_ROOT / "logs/sim"
PX4_SITL_BIN = REPO_ROOT.parent / ".work/px4-tv3/build/px4_sitl_default/bin/px4"

STATUS_VALUES = ("not_started", "in_progress", "structural", "verified", "blocked")
GATE_RESULTS = ("unknown", "pass", "fail", "skipped")

PHASE_DEFINITIONS: tuple[dict[str, Any], ...] = (
    {
        "id": "0",
        "title": "Stabilize SIH Baseline",
        "goal": "Make the active simulator boring to build and easy to reproduce.",
        "gate_script": "scripts/check_barebones.sh",
        "gate_profile": "fast",
    },
    {
        "id": "1",
        "title": "Prove Lander Hover Window",
        "goal": "Validate the first required lander scenario in SIH before expanding scope.",
        "gate_script": "scripts/check_hover_window.sh",
        "gate_profile": "slow",
        "profile_hint": "lander_hover_window",
    },
    {
        "id": "2",
        "title": "Replace Provisional Physical Data",
        "goal": "Manifests represent the real vehicles closely enough for simulation and control design.",
        "gate_script": "scripts/check_physical_manifests.sh",
        "gate_profile": "fast",
    },
    {
        "id": "3",
        "title": "Propulsion And Load-Cell Semantics",
        "goal": "Flight software sees motor state the same way in SIH, bench tests, and flight.",
        "gate_script": "scripts/check_propulsion_semantics.sh",
        "gate_profile": "fast",
    },
    {
        "id": "4",
        "title": "Control Mixer",
        "goal": "Requested torque and net thrust become reachable, bounded engine commands.",
        "gate_script": "scripts/check_control_mixer.sh",
        "gate_profile": "fast",
    },
    {
        "id": "5",
        "title": "Guidance And Monte Carlo",
        "goal": "Guidance only claims a solution when the remaining vehicle envelope can execute it.",
        "gate_script": "scripts/check_guidance_monte_carlo.sh",
        "gate_profile": "fast",
    },
    {
        "id": "6",
        "title": "Bench And Hardware Gates",
        "goal": "Real sensors and outputs behave like the simulated interfaces.",
        "gate_script": None,
        "gate_profile": "manual",
    },
)

FLIGHT_GATE_DEFINITIONS: tuple[dict[str, str], ...] = (
    {"id": "A", "title": "tv3_v1 single-engine ascent free flight", "vehicle": "tv3_v1"},
    {"id": "B", "title": "tv3_lander_v1 restrained/tethered lander tests", "vehicle": "tv3_lander_v1"},
    {"id": "C", "title": "tv3_lander_v1 waypoint and landing flight", "vehicle": "tv3_lander_v1"},
)


@dataclass
class GateRun:
    result: str
    duration_s: float
    output_tail: str


@dataclass
class ManifestProgress:
    name: str
    flight_ready: bool
    summary: str
    counts: dict[str, int]
    measured_ratio: float


def utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")


def default_status() -> dict[str, Any]:
    phases = {}
    for phase in PHASE_DEFINITIONS:
        phases[phase["id"]] = {
            "title": phase["title"],
            "gate_script": phase.get("gate_script"),
            "gate_profile": phase["gate_profile"],
            "status": "not_started",
            "status_override": None,
            "notes": "",
            "evidence": [],
            "last_gate_run_utc": None,
            "last_gate_result": "unknown",
            "last_gate_output_tail": "",
        }
    flight_gates = {
        gate["id"]: {
            "title": gate["title"],
            "vehicle": gate["vehicle"],
            "status": "not_started",
            "status_override": None,
            "notes": "",
            "evidence": [],
        }
        for gate in FLIGHT_GATE_DEFINITIONS
    }
    return {
        "schema": "tv3_completion_status_v1",
        "updated_utc": utc_now(),
        "phases": phases,
        "flight_gates": flight_gates,
    }


def merge_status(existing: dict[str, Any] | None) -> dict[str, Any]:
    status = default_status()
    if not existing:
        return status

    status["updated_utc"] = existing.get("updated_utc", status["updated_utc"])
    for phase_id, defaults in status["phases"].items():
        saved = existing.get("phases", {}).get(phase_id, {})
        for key in (
            "status_override",
            "notes",
            "evidence",
            "last_gate_run_utc",
            "last_gate_result",
            "last_gate_output_tail",
        ):
            if key in saved:
                defaults[key] = saved[key]
    for gate_id, defaults in status["flight_gates"].items():
        saved = existing.get("flight_gates", {}).get(gate_id, {})
        for key in ("status_override", "notes", "evidence"):
            if key in saved:
                defaults[key] = saved[key]
    return status


def run_gate(script_rel: str, timeout_s: int) -> GateRun:
    script = REPO_ROOT / script_rel
    started = datetime.now(timezone.utc)
    try:
        completed = subprocess.run(
            [str(script)],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        output = (exc.stdout or "") + (exc.stderr or "")
        return GateRun("fail", float(timeout_s), tail(output, 12))

    duration = (datetime.now(timezone.utc) - started).total_seconds()
    output = completed.stdout + completed.stderr
    result = "pass" if completed.returncode == 0 else "fail"
    return GateRun(result, duration, tail(output, 12))


def tail(text: str, max_lines: int) -> str:
    lines = [line.rstrip() for line in text.splitlines() if line.strip()]
    return "\n".join(lines[-max_lines:])


def manifest_progress(path: Path) -> ManifestProgress:
    manifest = load_json(path)
    data_status = manifest.get("data_status", {})
    fields = data_status.get("fields", {})
    counts: dict[str, int] = {}
    for value in fields.values():
        counts[value] = counts.get(value, 0) + 1
    total = sum(counts.values())
    measured = counts.get("measured", 0)
    ratio = measured / total if total else 0.0
    return ManifestProgress(
        name=manifest.get("name", path.stem),
        flight_ready=bool(data_status.get("flight_ready", False)),
        summary=str(data_status.get("summary", "")),
        counts=counts,
        measured_ratio=ratio,
    )


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def parse_manifest_text(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text().splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    return values


def latest_sim_runs(profile_hint: str | None = None, limit: int = 3) -> list[str]:
    runs: list[tuple[str, Path]] = []
    if not SIM_LOG_DIR.is_dir():
        return []

    for manifest_path in SIM_LOG_DIR.glob("*/*/manifest.txt"):
        meta = parse_manifest_text(manifest_path)
        profile = meta.get("flight_profile", "")
        if profile_hint and profile_hint not in profile:
            continue
        archived_at = meta.get("archived_at_utc", "")
        runs.append((archived_at, manifest_path.parent))

    runs.sort(key=lambda item: item[0], reverse=True)
    return [relative(path) for _, path in runs[:limit]]


def latest_ground_captures(limit: int = 3) -> list[str]:
    captures = sorted(GROUND_LOG_DIR.glob("bench_capture_*.json"), reverse=True)
    return [relative(path) for path in captures[:limit]]


def derive_phase2_evidence(manifests: list[ManifestProgress], bench_captures: list[str]) -> list[str]:
    evidence = list(bench_captures)
    for manifest in manifests:
        if manifest.counts.get("measured", 0) > 0 and manifest.summary:
            evidence.append(f"config/vehicles/{manifest.name}.json")
    return dedupe(evidence)


def dedupe(items: list[str]) -> list[str]:
    seen: set[str] = set()
    ordered: list[str] = []
    for item in items:
        if item in seen:
            continue
        seen.add(item)
        ordered.append(item)
    return ordered


def merge_evidence(existing: list[str], derived: list[str]) -> list[str]:
    return dedupe([*existing, *derived])


def derive_status(
    phase_id: str,
    phase: dict[str, Any],
    gate_result: str,
    manifests: list[ManifestProgress],
    sim_runs: list[str],
    bench_captures: list[str],
    px4_built: bool,
) -> tuple[str, list[str]]:
    override = phase.get("status_override")
    if override:
        return override, list(phase.get("evidence", []))

    evidence = list(phase.get("evidence", []))

    if phase_id == "0":
        evidence = merge_evidence(evidence, sim_runs[:1])
        if gate_result == "pass" and px4_built and sim_runs:
            return "verified", evidence
        if gate_result == "pass":
            return "structural", evidence
        if gate_result == "fail":
            return "in_progress", evidence
        if sim_runs or px4_built:
            return "in_progress", evidence
        return "not_started", evidence

    if phase_id == "1":
        hover_runs = latest_sim_runs("lander_hover_window", limit=3)
        evidence = merge_evidence(evidence, hover_runs)
        if gate_result == "pass" and hover_runs:
            return "verified", evidence
        if gate_result == "pass":
            return "structural", evidence
        if hover_runs:
            return "in_progress", evidence
        if gate_result == "fail":
            return "in_progress", evidence
        return "not_started", evidence

    if phase_id == "2":
        evidence = merge_evidence(evidence, derive_phase2_evidence(manifests, bench_captures))
        any_measured = any(item.counts.get("measured", 0) > 0 for item in manifests)
        all_ready = manifests and all(item.flight_ready for item in manifests)
        if gate_result == "pass" and all_ready:
            return "verified", evidence
        if gate_result == "pass" and any_measured:
            return "in_progress", evidence
        if gate_result == "pass":
            return "structural", evidence
        if any_measured or bench_captures:
            return "in_progress", evidence
        return "not_started", evidence

    if phase_id in {"3", "4", "5"}:
        if gate_result == "pass":
            return "structural", evidence
        if gate_result == "fail":
            return "in_progress", evidence
        return "not_started", evidence

    if phase_id == "6":
        if bench_captures:
            evidence = merge_evidence(evidence, bench_captures[:1])
            return "in_progress", evidence
        return "not_started", evidence

    return "not_started", evidence


def format_counts(counts: dict[str, int]) -> str:
    if not counts:
        return "no tracked fields"
    total = sum(counts.values())
    measured = counts.get("measured", 0)
    preliminary = counts.get("preliminary", 0)
    placeholder = counts.get("placeholder", 0)
    return f"{measured}/{total} measured, {preliminary} preliminary, {placeholder} placeholder"


def build_next_block(status: dict[str, Any], manifests: list[ManifestProgress]) -> list[str]:
    lines: list[str] = []
    failing_ids = [
        phase_id
        for phase_id, phase in status["phases"].items()
        if phase.get("last_gate_result") == "fail" and phase.get("gate_script")
    ]
    if failing_ids:
        names = ", ".join(f"Phase {phase_id}" for phase_id in failing_ids)
        lines.append(
            f"Gate scripts failing: {names}. Run `./scripts/check_completion_status.sh --run-gates fast` after fixing tests."
        )

    for manifest in manifests:
        if not manifest.flight_ready:
            lines.append(
                f"`{manifest.name}`: {format_counts(manifest.counts)}; `flight_ready=false`."
            )

    phase6 = status["phases"]["6"]
    if phase6.get("status") in {"not_started", "in_progress"}:
        lines.append("Phase 6 hardware gates still require restrained ignition, actuator measurement, and flight-controller validation.")

    if not lines:
        lines.append("All automated fast gates pass. Run slow Phase 1 SIH gate and promote measured manifest fields.")
    return lines


def render_markdown(
    status: dict[str, Any],
    manifests: list[ManifestProgress],
    px4_built: bool,
    generated_utc: str,
) -> str:
    lines = [
        "# TV3 Completion Status",
        "",
        f"Generated by `tools/report_completion_status.py` at `{generated_utc}`.",
        "Plan and exit criteria live in [completion_roadmap.md](completion_roadmap.md).",
        "Edit manual notes and overrides in [config/completion_status.json](../config/completion_status.json), then regenerate.",
        "",
        "## Phase Dashboard",
        "",
        "| Phase | Status | Gate | Last gate | Evidence |",
        "| --- | --- | --- | --- | --- |",
    ]

    for phase_def in PHASE_DEFINITIONS:
        phase = status["phases"][phase_def["id"]]
        gate = phase.get("gate_script") or "manual review"
        gate_result = phase.get("last_gate_result", "unknown")
        last_run = phase.get("last_gate_run_utc") or "—"
        evidence_items = phase.get("evidence", [])
        evidence = "<br>".join(f"`{item}`" for item in evidence_items[:2]) if evidence_items else "—"
        if len(evidence_items) > 2:
            evidence += f"<br>+{len(evidence_items) - 2} more"
        lines.append(
            f"| {phase_def['id']}: {phase_def['title']} | `{phase.get('status', 'unknown')}` | `{gate}` | `{gate_result}` @ {last_run} | {evidence} |"
        )

    lines.extend(
        [
            "",
            "## Manifest Physical Progress",
            "",
            "| Vehicle | flight_ready | Field provenance | Summary |",
            "| --- | --- | --- | --- |",
        ]
    )
    for manifest in manifests:
        summary = manifest.summary
        if len(summary) > 96:
            summary = summary[:93] + "..."
        lines.append(
            f"| `{manifest.name}` | `{manifest.flight_ready}` | {format_counts(manifest.counts)} | {summary} |"
        )

    lines.extend(
        [
            "",
            "## Flight Gates",
            "",
            "| Gate | Vehicle | Status | Notes |",
            "| --- | --- | --- | --- |",
        ]
    )
    for gate_def in FLIGHT_GATE_DEFINITIONS:
        gate = status["flight_gates"][gate_def["id"]]
        notes = gate.get("notes") or "—"
        lines.append(
            f"| {gate_def['id']}: {gate_def['title']} | `{gate_def['vehicle']}` | `{gate.get('status', 'unknown')}` | {notes} |"
        )

    lines.extend(["", "## Infrastructure Signals", "", f"- PX4 SIH binary present: `{px4_built}` (`{relative(PX4_SITL_BIN)}`)"])
    latest_sim = latest_sim_runs(limit=1)
    if latest_sim:
        lines.append(f"- Latest archived SIH run: `{latest_sim[0]}`")
    latest_bench = latest_ground_captures(limit=1)
    if latest_bench:
        lines.append(f"- Latest bench capture: `{latest_bench[0]}`")

    phase_notes = [
        (phase_def["id"], phase_def["title"], status["phases"][phase_def["id"]].get("notes", ""))
        for phase_def in PHASE_DEFINITIONS
        if status["phases"][phase_def["id"]].get("notes")
    ]
    if phase_notes:
        lines.extend(["", "## Phase Notes", ""])
        for phase_id, title, note in phase_notes:
            lines.append(f"- **Phase {phase_id} ({title})**: {note}")

    lines.extend(["", "## Next Blockers", ""])
    for item in build_next_block(status, manifests):
        lines.append(f"- {item}")

    lines.extend(
        [
            "",
            "## Refresh",
            "",
            "```bash",
            "./scripts/check_completion_status.sh                  # update dashboard only",
            "./scripts/check_completion_status.sh --run-gates fast # rerun fast gate scripts",
            "./scripts/check_completion_status.sh --run-gates all  # include slow SIH hover gate",
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def update_status(
    run_gates: str,
    gate_timeout_s: int,
    write_doc: bool,
) -> dict[str, Any]:
    existing = load_json(STATUS_PATH) if STATUS_PATH.is_file() else None
    status = merge_status(existing)
    generated_utc = utc_now()

    manifests = [manifest_progress(path) for path in sorted(VEHICLE_DIR.glob("*.json"))]
    bench_captures = latest_ground_captures()
    px4_built = PX4_SITL_BIN.is_file() and PX4_SITL_BIN.stat().st_size > 0

    for phase_def in PHASE_DEFINITIONS:
        phase_id = phase_def["id"]
        phase = status["phases"][phase_id]
        gate_script = phase_def.get("gate_script")
        gate_profile = phase_def["gate_profile"]

        should_run = False
        if run_gates == "all" and gate_script:
            should_run = True
        elif run_gates == "fast" and gate_script and gate_profile == "fast":
            should_run = True

        if should_run and gate_script:
            gate_run = run_gate(gate_script, gate_timeout_s)
            phase["last_gate_run_utc"] = generated_utc
            phase["last_gate_result"] = gate_run.result
            phase["last_gate_output_tail"] = gate_run.output_tail
        elif gate_script and phase.get("last_gate_result") not in GATE_RESULTS:
            phase["last_gate_result"] = "unknown"

        gate_result = phase.get("last_gate_result", "unknown")
        sim_runs = latest_sim_runs()
        derived_status, derived_evidence = derive_status(
            phase_id,
            phase,
            gate_result,
            manifests,
            sim_runs,
            bench_captures,
            px4_built,
        )
        if not phase.get("status_override"):
            phase["status"] = derived_status
        phase["evidence"] = merge_evidence(phase.get("evidence", []), derived_evidence)

        if phase_def["id"] == "6" and not phase.get("status_override"):
            if bench_captures:
                phase["status"] = "in_progress"
            elif phase.get("notes"):
                phase["status"] = "in_progress"

    for gate_def in FLIGHT_GATE_DEFINITIONS:
        gate = status["flight_gates"][gate_def["id"]]
        if gate.get("status_override"):
            gate["status"] = gate["status_override"]
        elif gate.get("notes") or gate.get("evidence"):
            gate["status"] = "in_progress"
        else:
            gate["status"] = "not_started"

    status["updated_utc"] = generated_utc
    write_json(STATUS_PATH, status)

    if write_doc:
        STATUS_DOC_PATH.write_text(render_markdown(status, manifests, px4_built, generated_utc))

    return status


def main() -> int:
    parser = argparse.ArgumentParser(description="Report TV3 completion-roadmap progress.")
    parser.add_argument(
        "--run-gates",
        choices=("none", "fast", "all"),
        default="none",
        help="Run gate scripts before updating status (fast skips the SIH hover-window gate).",
    )
    parser.add_argument(
        "--gate-timeout-s",
        type=int,
        default=1800,
        help="Timeout per gate script when --run-gates is set.",
    )
    parser.add_argument(
        "--no-doc",
        action="store_true",
        help="Update config/completion_status.json without rewriting docs/completion_status.md.",
    )
    args = parser.parse_args()

    status = update_status(args.run_gates, args.gate_timeout_s, write_doc=not args.no_doc)
    print(f"Updated {relative(STATUS_PATH)}")
    if not args.no_doc:
        print(f"Wrote {relative(STATUS_DOC_PATH)}")

    failing = [
        phase_id
        for phase_id, phase in status["phases"].items()
        if phase.get("last_gate_result") == "fail" and phase.get("gate_script")
    ]
    if failing and args.run_gates != "none":
        print(f"Gate failures recorded for phase(s): {', '.join(failing)}", file=sys.stderr)
        return 1
    if failing and args.run_gates == "none":
        print(
            f"Recorded gate failures for phase(s): {', '.join(failing)} (rerun with --run-gates fast to refresh)",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())