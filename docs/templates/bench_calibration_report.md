# Bench Load-Cell Calibration Report

Use this template when promoting `hardware.load_cell.calibration` fields in a
vehicle manifest from `placeholder` to `measured`. Archive the completed report
with ground-test logs under `logs/ground/`.

## Test Metadata

| Field | Value |
|-------|-------|
| Report ID | |
| Date (UTC) | |
| Operator | |
| Vehicle manifest | `config/vehicles/` |
| Bench / fixture ID | |
| Load-cell channel (`RK_LC_CH`) | |
| ADC instance / mode | |
| Firmware build / git SHA | |
| Related ULog run ID | |

## Hardware Under Test

| Item | Value |
|------|-------|
| Load-cell part number | |
| Amplifier / ADC board | |
| Mounting orientation | |
| Cable length / routing notes | |
| Igniter isolated (Y/N) | |

## Calibration Procedure

1. Power the flight stack with motors unarmed and no thrust applied.
2. Record at least 30 s of tare samples with the vehicle restrained.
3. Apply known masses or reference loads in ascending order.
4. Record steady-state ADC counts for each load step.
5. Verify no-thrust false positives are rejected before updating manifest values.

## Measurements

### Tare

| Sample window | Mean raw count | Std dev counts | Notes |
|---------------|----------------|----------------|-------|
| | | | |

### Scale Points

| Step | Known load (N) | Mean raw count | Derived N per count | Notes |
|------|----------------|----------------|---------------------|-------|
| 1 | | | | |
| 2 | | | | |
| 3 | | | | |

### Derived Calibration

| Manifest field | Measured value | Units | Method |
|----------------|----------------|-------|--------|
| `hardware.load_cell.calibration.tare` | | counts | mean no-load ADC |
| `hardware.load_cell.calibration.scale` | | N/count | linear fit |
| `hardware.load_cell.calibration.kg_per_count` | | kg/count | optional secondary |
| `hardware.load_cell.alpha` | | | filter setting retained or updated |
| `hardware.load_cell.timeout_ms` | | ms | max observed gap + margin |

## Validation Checks

| Check | Pass/Fail | Evidence |
|-------|-----------|----------|
| Tare stable over 30 s | | |
| Scale monotonic across load steps | | |
| No ignition confirmation below `RK_LAUNCH_THR_N` | | |
| Stale timeout matches observed ADC gaps | | |
| SIH / bench ADC replay agrees with expected `tv3_engine_state` | | |

## Manifest Update

After this report is approved, update the selected vehicle manifest (`config/vehicles/<vehicle>.json`):

```json
{
  "data_status": {
    "fields": {
      "hardware.load_cell.calibration": "measured"
    }
  },
  "hardware": {
    "load_cell": {
      "calibration": {
        "tare": 0.0,
        "scale": 0.0,
        "kg_per_count": 0.0
      }
    }
  }
}
```

Replace the placeholder numbers with measured values from the tables above.

Re-run:

```bash
./scripts/stage_microsd.sh
./scripts/complete_phase2_bench.sh --body-mass-kg <weighed_body_kg>
./scripts/check_physical_manifests.sh
./scripts/check_propulsion_semantics.sh
```

## Sign-Off

| Role | Name | Date | Notes |
|------|------|------|-------|
| Performed by | | | |
| Reviewed by | | | |
