"""Load checked-in thrust-curve motors for vehicle manifests and visualization."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Literal

from tools.generate_motor_catalog import DEFAULT_SOURCE, load_inventory, validate_motor

REPO_ROOT = Path(__file__).resolve().parents[1]
ThrustBasis = Literal["max_certified", "average_certified", "curve_peak"]


@dataclass(frozen=True)
class MotorCatalogEntry:
    motor_index: int
    motor_id: str
    manufacturer: str
    designation: str
    loaded_mass_kg: float
    dry_mass_kg: float
    average_thrust_n: float
    max_thrust_n: float
    curve_peak_thrust_n: float
    total_impulse_ns: float
    burn_duration_s: float

    def thrust_n(self, basis: ThrustBasis = "max_certified") -> float:
        if basis == "average_certified":
            return self.average_thrust_n
        if basis == "curve_peak":
            return self.curve_peak_thrust_n
        return self.max_thrust_n

    @property
    def label(self) -> str:
        return f"{self.manufacturer} {self.designation}"


def _catalog_source_root(source: Path | str | None) -> Path:
    if source is None:
        return DEFAULT_SOURCE
    path = Path(source)
    if not path.is_absolute():
        path = REPO_ROOT / path
    return path


@lru_cache(maxsize=4)
def load_motor_catalog(source: str | None = None) -> dict[str, MotorCatalogEntry]:
    source_root = _catalog_source_root(source)
    inventory_path = source_root / "motor_inventory.csv"
    if not inventory_path.exists():
        return {}

    entries: dict[str, MotorCatalogEntry] = {}
    for index, row in enumerate(load_inventory(inventory_path)):
        result, specs, curve = validate_motor(row, source_root, index)
        if not result.active or specs is None or curve is None:
            continue

        specs_path = source_root / row["specs_file"]
        with specs_path.open(newline="") as stream:
            specs_row = next(csv.DictReader(stream), None)
        if specs_row is None:
            continue

        entries[result.motor_id] = MotorCatalogEntry(
            motor_index=result.motor_index,
            motor_id=result.motor_id,
            manufacturer=result.manufacturer,
            designation=result.designation,
            loaded_mass_kg=specs.loaded_mass_kg,
            dry_mass_kg=specs.dry_mass_kg,
            average_thrust_n=float(specs_row["average_thrust_certified_N"]),
            max_thrust_n=float(specs_row["max_thrust_certified_N"]),
            curve_peak_thrust_n=max(point.thrust_n for point in curve),
            total_impulse_ns=specs.total_impulse_ns,
            burn_duration_s=specs.burn_duration_s,
        )
    return entries


def resolve_motor_id(engine: dict, motor_selection: dict) -> str:
    return str(
        engine.get("motor_id")
        or motor_selection.get("default_motor_id")
        or motor_selection.get("motor_id")
        or ""
    ).strip()


def thrust_basis_from_manifest(motor_selection: dict) -> ThrustBasis:
    basis = str(motor_selection.get("thrust_basis", "max_certified"))
    if basis in ("max_certified", "average_certified", "curve_peak"):
        return basis  # type: ignore[return-value]
    return "max_certified"


def allocator_thrust_fields_from_catalog(vehicle: dict) -> dict[str, float] | None:
    """Derive allocator reference thrust from checked-in motor catalog totals."""
    motor_selection = vehicle.get("motor_selection", {})
    catalog_source = motor_selection.get("catalog_source")
    if not catalog_source:
        return None

    catalog = load_motor_catalog(str(catalog_source))
    if not catalog:
        return None

    engines = vehicle.get("propulsion", {}).get("engines") or []
    if not engines:
        return None

    max_total = 0.0
    avg_total = 0.0
    for engine in engines:
        motor_id = resolve_motor_id(engine, motor_selection)
        entry = catalog.get(motor_id)
        if entry is None:
            return None
        max_total += entry.max_thrust_n
        avg_total += entry.average_thrust_n

    body = vehicle["vehicle"]
    minimum_floor = float(body.get("ca_minimum_thrust_n", 4.0))
    fallback_floor = float(body.get("ca_fallback_thrust_n", 10.0))
    return {
        "ca_reference_thrust_n": max_total,
        "ca_minimum_thrust_n": max(avg_total * 0.5, minimum_floor),
        "ca_fallback_thrust_n": max(avg_total, fallback_floor),
    }


def engine_thrust_n(
    vehicle: dict,
    engine: dict,
    *,
    catalog: dict[str, MotorCatalogEntry] | None = None,
) -> float:
    body = vehicle["vehicle"]
    engines = vehicle.get("propulsion", {}).get("engines") or [engine]
    motor_selection = vehicle.get("motor_selection", {})
    motor_id = resolve_motor_id(engine, motor_selection)
    basis = thrust_basis_from_manifest(motor_selection)

    if catalog is None and motor_selection.get("catalog_source"):
        catalog = load_motor_catalog(str(motor_selection["catalog_source"]))

    if catalog and motor_id in catalog:
        return catalog[motor_id].thrust_n(basis)

    return body["ca_reference_thrust_n"] * float(
        engine.get("thrust_fraction", 1.0 / max(len(engines), 1))
    )