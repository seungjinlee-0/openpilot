"""Custom external HUD screen for a 2022 Kia K5 Hybrid (ClusterHudScreenMode 7, "my-hud").

Everything for this screen lives in this file so rebasing onto upstream carrot-wip only
touches a handful of one-line hooks (screen-mode alias, one state field, render dispatch).

Layout: 1920x480, driving band on top (turn signals, gear, ENGINE/EV, RPM, set speed,
lead distance, accel/brake bars) and a status row below (TPMS, HV battery, 12V, device).

Values openpilot does not publish for this legacy-CAN hybrid are decoded here from raw
bus-0 CAN using hyundai_kia_generic.dbc (verified against a real K5 HEV rlog):
  0x371 E_EMS11  N (engine rpm), Engine_Run (0 = engine off -> EV driving)
  0x220 ESP12    CYL_PRES (brake pressure, bar)
  0x50B EV_Info  OPKR_EV_Charge_Level (HV battery SOC, %)
"""

from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pyray as rl

from cluster_config import DESIGN_HEIGHT, DESIGN_WIDTH

MY_HUD_SCREEN_MODE = 7  # ClusterHudScreenMode value / --screen-mode my-hud
DBC_NAME = "hyundai_kia_generic.dbc"
CAN_BUS = 0
CAN_SIGNALS: dict[int, tuple[str, ...]] = {
    0x371: ("N", "Engine_Run"),
    0x220: ("CYL_PRES",),
    0x50B: ("OPKR_EV_Charge_Level",),
}
CAN_STALE_S = 1.0
LIVE_CAN_IDLE_S = 2.0  # stop draining raw CAN this long after the my-hud screen was last drawn
SERVICE_STALE_S = 5.0
RPM_BAR_MAX = 6000.0
BRAKE_BAR_MAX = 10.0
TPMS_LOW_PSI = 31.0
CPU_TEMP_WARN_C = 75.0
CPU_TEMP_CRIT_C = 90.0
REGEN_MIN_SPEED_MPS = 1.5  # regen fades out below ~5 km/h
REGEN_MIN_DECEL_MPS2 = -0.25
REGEN_HOLD_S = 0.6  # keep the charge mark steady between samples
REGEN_BAR_MAX_MPS2 = 3.0  # deceleration that fills the regen bar
CRUISE_ACCEL_MAX_MPS2 = 2.0  # requested acceleration that fills the accel bar under cruise

BG = (4, 7, 10)
INK = (233, 241, 239)
DIM = (124, 140, 136)
FAINT = (36, 48, 45)
LINE = (23, 32, 30)
SET_COLOR = (90, 214, 190)
EV_COLOR = (91, 217, 140)
ENGINE_COLOR = (240, 164, 58)
GAS_COLOR = (90, 214, 190)
BRAKE_COLOR = (255, 91, 79)
WARN_COLOR = (240, 164, 58)
BLINK_COLOR = (57, 226, 106)
REGEN_COLOR = (60, 255, 120)
BAR_OUTLINE = (54, 70, 66)

# Updated by draw_my_hud so live telemetry only drains raw CAN while this screen is visible.
_screen_state = {"last_draw": 0.0}


@dataclass(frozen=True, slots=True)
class MyHudSnapshot:
    engine_rpm: float | None = None
    engine_running: bool | None = None
    brake_pressed: bool = False
    brake_pressure_bar: float | None = None
    gas_percent: float | None = None
    lead_distance_m: float | None = None
    decel_mps2: float | None = None
    hv_soc_percent: float | None = None
    regen_charging: bool = False
    battery_voltage_v: float | None = None
    fan_rpm: float | None = None
    cpu_temp_c: float | None = None
    drive_seconds: float | None = None
    cpu_usage_percent: float | None = None
    memory_used_percent: float | None = None
    disk_used_percent: float | None = None


# ---------------------------------------------------------------------------
# DBC decoding (only the handful of signals above)

@dataclass(frozen=True, slots=True)
class _SignalDef:
    name: str
    start: int
    length: int
    little_endian: bool
    signed: bool
    factor: float
    offset: float


_BO_RE = re.compile(r"^BO_\s+(\d+)\s+\w+\s*:")
_SG_RE = re.compile(r"^\s*SG_\s+(\w+)\s*(?:M|m\d+)?\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*\(([^,]+),([^)]+)\)")


def _find_dbc() -> Path | None:
    for root in Path(__file__).resolve().parents:
        for candidate in (root / "opendbc_repo" / "opendbc" / "dbc" / DBC_NAME, root / "opendbc" / "dbc" / DBC_NAME):
            if candidate.exists():
                return candidate
    return None


def load_can_signals(path: Path | None = None) -> dict[int, tuple[_SignalDef, ...]]:
    path = path or _find_dbc()
    if path is None:
        print(f"[my-hud] {DBC_NAME} not found; CAN metrics disabled", flush=True)
        return {}
    wanted = {addr: set(names) for addr, names in CAN_SIGNALS.items()}
    found: dict[int, list[_SignalDef]] = {}
    current: int | None = None
    with open(path, encoding="latin-1") as f:
        for line in f:
            bo = _BO_RE.match(line)
            if bo:
                current = int(bo.group(1))
                continue
            if current not in wanted:
                continue
            sg = _SG_RE.match(line)
            if sg and sg.group(1) in wanted[current]:
                found.setdefault(current, []).append(_SignalDef(
                    name=sg.group(1),
                    start=int(sg.group(2)),
                    length=int(sg.group(3)),
                    little_endian=sg.group(4) == "1",
                    signed=sg.group(5) == "-",
                    factor=float(sg.group(6)),
                    offset=float(sg.group(7)),
                ))
    return {addr: tuple(defs) for addr, defs in found.items()}


def decode_signal(sig: _SignalDef, dat: bytes) -> float | None:
    if sig.little_endian:
        if (sig.start + sig.length + 7) // 8 > len(dat):
            return None
        raw = (int.from_bytes(dat, "little") >> sig.start) & ((1 << sig.length) - 1)
    else:
        raw = 0
        bit = sig.start
        for _ in range(sig.length):
            byte = bit // 8
            if byte >= len(dat):
                return None
            raw = (raw << 1) | ((dat[byte] >> (bit % 8)) & 1)
            bit = bit + 15 if bit % 8 == 0 else bit - 1
    if sig.signed and raw & (1 << (sig.length - 1)):
        raw -= 1 << sig.length
    return raw * sig.factor + sig.offset


# ---------------------------------------------------------------------------
# Telemetry collection

def _finite(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _list_values(values: Any) -> list[float]:
    try:
        return [v for v in (_finite(x) for x in values) if v is not None]
    except TypeError:
        return []


class MyHudTelemetry:
    """Keeps the latest values the custom HUD needs; fed from live SubMaster or rlog replay."""

    LIVE_SERVICES = ("radarState", "deviceState", "peripheralState", "can")

    def __init__(self) -> None:
        self._signals = load_can_signals()
        self._can: dict[str, tuple[float, float]] = {}
        self._values: dict[str, tuple[float | None, float]] = {}
        self._messaging: Any = None
        self._can_sock: Any = None
        self._can_unavailable = False
        self._regen_until = 0.0

    def update_live(self, sm: Any) -> None:
        """Feed from the live SubMaster; drain raw CAN on a dedicated socket while my-hud is shown.

        `can` deliberately stays out of the SubMaster: it reads one message per update, which lags far
        behind 100 Hz `can`, and it would also run the CAN-FD corner-radar parser on this car's
        bus-1 radar frames (0x238-0x248 overlap its address range).
        """
        for service in ("radarState", "deviceState", "peripheralState"):
            try:
                if service in sm.services and sm.updated[service]:
                    self.observe(service, sm[service], sm.logMonoTime[service] / 1e9)
            except Exception:
                pass

        if self._can_unavailable:
            return
        if time.monotonic() - _screen_state["last_draw"] > LIVE_CAN_IDLE_S:
            self._can_sock = None  # another screen is active: stop paying for raw CAN
            return
        if self._messaging is None:
            try:
                import openpilot.cereal.messaging as messaging
            except Exception as exc:
                self._can_unavailable = True
                print(f"[my-hud] cereal messaging unavailable; CAN metrics disabled: {exc}", flush=True)
                return
            self._messaging = messaging
        try:
            if self._can_sock is None:
                self._can_sock = self._messaging.sub_sock("can", conflate=False, timeout=0)
            for event in self._messaging.drain_sock(self._can_sock):
                self._observe_can(event.can, event.logMonoTime / 1e9)
        except Exception as exc:
            self._can_sock = None  # recreate on the next frame
            print(f"[my-hud] raw CAN read failed: {exc}", flush=True)

    def observe(self, service: str, msg: Any, event_t: float) -> None:
        if msg is None:
            return
        try:
            if service == "can":
                self._observe_can(msg, event_t)
            elif service == "radarState":
                lead = msg.leadOne
                self._values["lead"] = (_finite(lead.dRel) if lead.status else None, event_t)
            elif service == "deviceState":
                cpu_temps = _list_values(msg.cpuTempC)
                cpu_usage = _list_values(msg.cpuUsagePercent)
                free = _finite(msg.freeSpacePercent)
                self._values["cpu_temp"] = (max(cpu_temps) if cpu_temps else None, event_t)
                started = bool(getattr(msg, "started", False))
                started_t = _finite(getattr(msg, "startedMonoTime", None))
                self._values["drive_start"] = (started_t / 1e9 if started and started_t else None, event_t)
                self._values["cpu_usage"] = (sum(cpu_usage) / len(cpu_usage) if cpu_usage else None, event_t)
                self._values["memory"] = (_finite(msg.memoryUsagePercent), event_t)
                self._values["disk"] = (100.0 - free if free is not None else None, event_t)
            elif service == "peripheralState":
                millivolts = _finite(msg.voltage)
                self._values["fan_rpm"] = (_finite(getattr(msg, "fanSpeedRpm", None)), event_t)
                self._values["voltage"] = (millivolts / 1000.0 if millivolts else None, event_t)
        except Exception:
            # A schema mismatch must never take the HUD down; the value just goes stale.
            pass

    def _observe_can(self, frames: Any, event_t: float) -> None:
        for frame in frames:
            if frame.src != CAN_BUS:
                continue
            defs = self._signals.get(frame.address)
            if not defs:
                continue
            dat = bytes(frame.dat)
            for sig in defs:
                value = decode_signal(sig, dat)
                if value is not None:
                    self._can[sig.name] = (value, event_t)

    def _fresh_can(self, name: str, now: float) -> float | None:
        entry = self._can.get(name)
        if entry is None or now - entry[1] > CAN_STALE_S:
            return None
        return entry[0]

    def _fresh(self, key: str, now: float) -> float | None:
        entry = self._values.get(key)
        if entry is None or now - entry[1] > SERVICE_STALE_S:
            return None
        return entry[0]

    def snapshot(self, car_state: Any, event_t: float) -> MyHudSnapshot:
        engine_run = self._fresh_can("Engine_Run", event_t)
        drive_start = self._fresh("drive_start", event_t)
        gas = _finite(getattr(car_state, "gas", None))
        # No battery current on this car's buses, so regen is inferred from the driving state:
        # pedal released and still decelerating above walking pace.
        v_ego = _finite(getattr(car_state, "vEgo", None))
        a_ego = _finite(getattr(car_state, "aEgo", None))
        if (not getattr(car_state, "gasPressed", False) and v_ego is not None and a_ego is not None
                and v_ego > REGEN_MIN_SPEED_MPS and a_ego < REGEN_MIN_DECEL_MPS2):
            self._regen_until = event_t + REGEN_HOLD_S
        return MyHudSnapshot(
            engine_rpm=self._fresh_can("N", event_t),
            engine_running=None if engine_run is None else engine_run >= 0.5,
            brake_pressed=bool(getattr(car_state, "brakePressed", False)),
            brake_pressure_bar=self._fresh_can("CYL_PRES", event_t),
            gas_percent=None if gas is None else max(0.0, min(100.0, gas * 100.0)),
            lead_distance_m=self._fresh("lead", event_t),
            decel_mps2=a_ego,
            hv_soc_percent=self._fresh_can("OPKR_EV_Charge_Level", event_t),
            regen_charging=event_t < self._regen_until,
            battery_voltage_v=self._fresh("voltage", event_t),
            fan_rpm=self._fresh("fan_rpm", event_t),
            cpu_temp_c=self._fresh("cpu_temp", event_t),
            drive_seconds=(event_t - drive_start) if drive_start is not None and event_t > drive_start else None,
            cpu_usage_percent=self._fresh("cpu_usage", event_t),
            memory_used_percent=self._fresh("memory", event_t),
            disk_used_percent=self._fresh("disk", event_t),
        )


# ---------------------------------------------------------------------------
# Drawing (coordinates are 1920x480 design pixels)

def _c(color: tuple[int, int, int], alpha: int = 255) -> rl.Color:
    return rl.Color(color[0], color[1], color[2], alpha)


def _rounded(x: float, y: float, w: float, h: float, radius: float, fill: tuple[int, int, int] | None,
             outline: tuple[int, int, int] | None = None, outline_width: float = 2.0) -> None:
    rect = rl.Rectangle(x, y, w, h)
    roundness = max(0.0, min(1.0, radius / max(1.0, min(w, h) * 0.5)))
    if fill is not None:
        rl.draw_rectangle_rounded(rect, roundness, 12, _c(fill))
    if outline is not None:
        rl.draw_rectangle_rounded_lines_ex(rect, roundness, 12, outline_width, _c(outline))


def _hbar(x: float, y: float, w: float, h: float, fraction: float, color: tuple[int, int, int]) -> None:
    _rounded(x, y, w, h, h * 0.25, FAINT)
    fraction = max(0.0, min(1.0, fraction))
    if fraction > 0.0:
        _rounded(x, y, max(h, w * fraction), h, h * 0.25, color)


def _vbar(x: float, y: float, w: float, h: float, fraction: float, color: tuple[int, int, int]) -> None:
    _rounded(x, y, w, h, 8, FAINT)
    fraction = max(0.0, min(1.0, fraction))
    if fraction > 0.0:
        fill_h = max(8.0, h * fraction)
        _rounded(x, y + h - fill_h, w, fill_h, 8, color)


def _value_with_unit(r: Any, value: str, unit: str, x: float, cy: float, size: float,
                     color: tuple[int, int, int], unit_size: float, anchor: str = "left") -> None:
    value_w, _ = r._measure_text(value, size)
    gap = unit_size * 0.25
    unit_w = r._measure_text(unit, unit_size)[0] + gap if unit else 0.0
    left = x - (value_w + unit_w) * 0.5 if anchor == "center" else x
    r._draw_text(value, left, cy, size, color, "left")
    if unit:
        r._draw_text(unit, left + value_w + gap, cy + (size - unit_size) * 0.3, unit_size, DIM, "left")


def _draw_signal(x: float, y: float, w: float, h: float, left: bool, lit: bool) -> None:
    """Turn signal: a plain triangle, kept small so the driving band stays uncluttered."""
    color = _c(BLINK_COLOR) if lit else _c(FAINT)
    tip = rl.Vector2(x if left else x + w, y + h * 0.5)
    top = rl.Vector2(x + w if left else x, y)
    bottom = rl.Vector2(x + w if left else x, y + h)
    if left:
        rl.draw_triangle(top, tip, bottom, color)
    else:
        rl.draw_triangle(top, bottom, tip, color)


def _draw_wifi(cx: float, cy: float, connected: bool) -> None:
    """Small status glyph; (cx, cy) is the dot, arcs rise about 20 px above it."""
    color = _c(INK if connected else FAINT)
    center = rl.Vector2(cx, cy)
    for radius in (9.0, 17.0):
        rl.draw_ring(center, radius, radius + 3.5, 225.0, 315.0, 16, color)
    rl.draw_circle_v(center, 3.5, color)  # disconnected simply stays grey, no slash


def _fmt(value: float | None, digits: int = 0) -> str:
    return "--" if value is None else f"{value:.{digits}f}"


def _cruise_active(state: Any) -> bool:
    name = str(getattr(getattr(state, "cruise_display_state", None), "name", getattr(state, "cruise_display_state", ""))).lower()
    return not any(word in name for word in ("off", "pause", "inactive", "unavailable"))


EV_PANEL_FILL = (10, 44, 27)
ENGINE_PANEL_FILL = (46, 31, 9)


def _draw_top_band(r: Any, state: Any, snap: MyHudSnapshot, signal_lights: tuple[bool, bool]) -> None:
    left_lit, right_lit = signal_lights
    _draw_signal(26, 96, 48, 110, True, left_lit)
    _draw_signal(1846, 96, 48, 110, False, right_lit)

    # The whole gear/RPM block is tinted by powertrain state so EV vs engine reads at a glance.
    engine_running = snap.engine_running
    if engine_running is None:
        accent = DIM
    else:
        accent = ENGINE_COLOR if engine_running else EV_COLOR
        _rounded(110, 30, 642, 254, 22, ENGINE_PANEL_FILL if engine_running else EV_PANEL_FILL, accent, 3)

    gear = str(getattr(state, "gear_text", "") or "").strip() or "--"
    _rounded(134, 52, 100, 100, 18, None, FAINT if engine_running is None else DIM, 3)
    r._draw_text(gear, 184, 102, 70, INK, "center")
    mode_text = "--" if engine_running is None else ("ENGINE" if engine_running else "EV")
    r._draw_text(mode_text, 184, 214, 30, accent, "center")

    if engine_running is False:
        soc = snap.hv_soc_percent
        if soc is None:
            r._draw_text("엔진 정지", 294, 58, 24, EV_COLOR, "left")
            r._draw_text("EV", 290, 150, 150, EV_COLOR, "left")
        else:
            # RPM is 0 while the engine is off; the HV battery is the number worth watching.
            r._draw_text("고전압 배터리", 294, 58, 24, EV_COLOR, "left")
            _value_with_unit(r, f"{soc:.0f}", "%", 294, 142, 124, EV_COLOR, 44)
            _hbar(294, 222, 436, 18, soc / 100.0, EV_COLOR)
            r._draw_text("0", 294, 257, 18, DIM, "left")
            r._draw_text("50", 512, 257, 18, DIM, "center")
            r._draw_text("100%", 730, 257, 18, DIM, "right")
    else:
        rpm = snap.engine_rpm
        r._draw_text("RPM", 294, 58, 24, accent, "left")
        r._draw_text("--" if rpm is None else f"{rpm:,.0f}", 294, 142, 124, FAINT if rpm is None else INK, "left")
        _hbar(294, 222, 436, 18, (rpm or 0.0) / RPM_BAR_MAX, ENGINE_COLOR)
        r._draw_text("0", 294, 257, 18, DIM, "left")
        r._draw_text("2k", 439, 257, 18, DIM, "center")
        r._draw_text("4k", 585, 257, 18, DIM, "center")
        r._draw_text("6k", 730, 257, 18, DIM, "right")

    for tile in TOP_DIVIDER_TILES:
        rl.draw_rectangle(STATUS_TILE_X0 + tile * STATUS_TILE_W, 36, 2, 238, _c(LINE))

    metric = bool(getattr(r, "is_metric", True))
    cruise = getattr(state, "cruise_kph", None)
    if cruise is None or cruise <= 0 or cruise >= 250:
        r._draw_text("--", SET_CENTER_X, 127, 210, FAINT, "center")
    else:
        shown = cruise if metric else cruise * 0.621371
        size = 210 if shown < 99.5 else 172  # three digits must stay inside the 390 px column
        r._draw_text(f"{shown:.0f}", SET_CENTER_X, 127, size, SET_COLOR if _cruise_active(state) else DIM, "center")
    r._draw_text("설정 km/h" if metric else "설정 mph", SET_CENTER_X, 258, 24, DIM, "center")

    lead = snap.lead_distance_m
    r._draw_text("차간거리", 1184, 58, 24, DIM, "left")
    if lead is None:
        r._draw_text("--", 1184, 159, 118, FAINT, "left")
    else:
        distance = lead if metric else lead * 3.28084
        _value_with_unit(r, f"{distance:.1f}" if distance < 100 else f"{distance:.0f}", "m" if metric else "ft",
                         1184, 159, 118, INK, 36)

    # Three fixed-colour bars instead of one brake box that changes colour: accel, regen, hydraulic.
    # While cruise drives, the pedal reads zero, so the accel bar shows the requested acceleration.
    gas = snap.gas_percent
    pressure = snap.brake_pressure_bar
    regen = abs(snap.decel_mps2 or 0.0) / REGEN_BAR_MAX_MPS2 if snap.regen_charging else 0.0
    cruise_mode = str(getattr(state, "cruise_display_state", "off") or "off")
    requested = _finite(getattr(state, "planned_accel_mps2", None))
    if cruise_mode == "engaged" and requested is not None:
        accel_fraction, accel_label = max(0.0, requested) / CRUISE_ACCEL_MAX_MPS2, f"요청 {max(0.0, requested):.1f}"
    else:
        accel_fraction, accel_label = (gas or 0.0) / 100.0, f"가속 {_fmt(gas)}%"
    bars = (
        (1544, accel_label, accel_fraction, GAS_COLOR),
        (1644, "회생", regen, REGEN_COLOR),
        (1744, f"유압 {_fmt(pressure, 1)}", (pressure or 0.0) / BRAKE_BAR_MAX, BRAKE_COLOR),
    )
    for x, label, fraction, color in bars:
        _vbar(x, 42, 64, 172, fraction, color)
        active = fraction > 0.0  # the active bar lights its own outline; idle bars stay grey
        _rounded(x - 6, 36, 76, 184, 11, None, color if active else BAR_OUTLINE, 3)  # sits outside the bar
        r._draw_text(label, x + 32, 238, 20, color if active else DIM, "center")

    # Same badge wording as ENGINE/EV above: who is working the pedals right now.
    # "paused" means a set speed exists but cruise is not driving, so the pedals are the driver's.
    cruise_text, cruise_color = ("CRUISE", SET_COLOR) if cruise_mode == "engaged" else ("MANUAL", DIM)
    r._draw_text(cruise_text, 1676, 274, 26, cruise_color, "center")


STATUS_TILE_X0 = 30
STATUS_TILE_W = 186
TOP_DIVIDER_TILES = (4, 6, 8)  # top-band column lines sit exactly on these status-row tile edges
SET_CENTER_X = STATUS_TILE_X0 + 5 * STATUS_TILE_W  # middle of the set-speed column (tiles 4-6)
STATUS_LABEL_Y = 336
STATUS_VALUE_Y = 422
STATUS_VALUE_SIZE = 66
STATUS_UNIT_SIZE = 28
STATUS_VALUE_MAX_W = STATUS_TILE_W - 18 - 12  # tile inner width, keeping air before the next divider
TPMS_VALUE_SIZE = 50


def _draw_status_row(r: Any, state: Any, snap: MyHudSnapshot) -> None:
    rl.draw_rectangle(STATUS_TILE_X0, 304, 10 * STATUS_TILE_W, 2, _c(LINE))

    def tile_x(index: int) -> float:
        x = STATUS_TILE_X0 + index * STATUS_TILE_W
        if index:
            rl.draw_rectangle(x, 326, 2, 136, _c(LINE))
            return x + 18
        return x

    # TPMS spans tiles 0-1: front pair on the first line, rear pair below.
    x = tile_x(0)
    r._draw_text("타이어 psi", x, STATUS_LABEL_Y, 21, DIM, "left")
    tpms = getattr(state, "tpms", None)
    for row_label, cy, pair in (
        ("앞", 390, (getattr(tpms, "fl", None), getattr(tpms, "fr", None))),
        ("뒤", 440, (getattr(tpms, "rl", None), getattr(tpms, "rr", None))),
    ):
        r._draw_text(row_label, x, cy, 21, DIM, "left")
        for value, dx in zip(pair, (60, 210)):
            low = value is not None and value < TPMS_LOW_PSI
            r._draw_text(_fmt(value), x + dx, cy, TPMS_VALUE_SIZE,
                         BRAKE_COLOR if low else (INK if value is not None else FAINT), "left")

    def metric_tile(index: int, label: str, value: float | None, unit: str, digits: int = 0,
                    color: tuple[int, int, int] = INK) -> None:
        tx = tile_x(index)
        r._draw_text(label, tx, STATUS_LABEL_Y, 21, DIM, "left")
        if value is None:
            r._draw_text("--", tx, STATUS_VALUE_Y, STATUS_VALUE_SIZE, FAINT, "left")
        else:
            text = _fmt(value, digits)
            size, unit_size = STATUS_VALUE_SIZE, STATUS_UNIT_SIZE
            width = r._measure_text(text, size)[0] + unit_size * 0.25 + r._measure_text(unit, unit_size)[0]
            if width > STATUS_VALUE_MAX_W:  # e.g. "12.9 V" shrinks slightly instead of touching the divider
                scale = STATUS_VALUE_MAX_W / width
                size, unit_size = size * scale, unit_size * scale
            _value_with_unit(r, text, unit, tx, STATUS_VALUE_Y, size, color, unit_size)

    cpu_temp = snap.cpu_temp_c
    cpu_temp_color = INK
    if cpu_temp is not None and cpu_temp >= CPU_TEMP_CRIT_C:
        cpu_temp_color = BRAKE_COLOR
    elif cpu_temp is not None and cpu_temp >= CPU_TEMP_WARN_C:
        cpu_temp_color = WARN_COLOR

    metric_tile(2, "고전압 배터리", snap.hv_soc_percent, "%")
    metric_tile(3, "12V 전압", snap.battery_voltage_v, "V", 1)
    # Fan rpm in thousands: 1986 -> "1.99" with a small "k" unit like the other tiles.
    metric_tile(4, "팬 rpm", None if snap.fan_rpm is None else snap.fan_rpm / 1000.0, "k", 2)
    metric_tile(5, "CPU 온도", cpu_temp, "°C", 0, cpu_temp_color)
    metric_tile(6, "CPU 사용", snap.cpu_usage_percent, "%")
    metric_tile(7, "메모리", snap.memory_used_percent, "%")
    metric_tile(8, "저장소", snap.disk_used_percent, "%")
    drive = snap.drive_seconds
    tx = tile_x(9)
    r._draw_text("주행 시간", tx, STATUS_LABEL_Y, 21, DIM, "left")
    if drive is None:
        r._draw_text("--", tx, STATUS_VALUE_Y, STATUS_VALUE_SIZE, FAINT, "left")
    else:
        r._draw_text(f"{int(drive // 3600)}:{int(drive // 60) % 60:02d}", tx, STATUS_VALUE_Y, STATUS_VALUE_SIZE, INK, "left")

    # Wi-Fi is only a small glyph at the right end of the label line.
    _draw_wifi(STATUS_TILE_X0 + 10 * STATUS_TILE_W - 22, 346, bool(getattr(state, "network_connected", False)))


def draw_my_hud(r: Any, state: Any, signal_lights: tuple[bool, bool]) -> None:
    _screen_state["last_draw"] = time.monotonic()
    snap = getattr(state, "my_hud", None) or MyHudSnapshot()
    rl.clear_background(_c(BG))
    rl.rl_push_matrix()
    rl.rl_scalef(r.width / DESIGN_WIDTH, r.height / DESIGN_HEIGHT, 1.0)
    try:
        _draw_top_band(r, state, snap, signal_lights)
        _draw_status_row(r, state, snap)
    finally:
        rl.rl_pop_matrix()
