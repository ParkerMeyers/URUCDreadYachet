"""Small shared helpers."""
from __future__ import annotations

import ipaddress
import re
import socket

from topside.constants import (
    AUX_TO_CSV,
    ARM_CSV_INDICES,
    ARM_PWM_MAX,
    ARM_PWM_MIN,
    CLAW_PWM_MAX,
    CLAW_PWM_MIN,
    CLAW_STOP_US_DEFAULT,
    JOINT_PWM_SPECS,
    THR_PWM_MAX,
    THR_PWM_MIN,
)


def clamp_arm_pwm(value) -> int:
    return int(max(ARM_PWM_MIN, min(ARM_PWM_MAX, int(round(float(value))))))


def clamp_claw_pwm(value) -> int:
    return int(max(CLAW_PWM_MIN, min(CLAW_PWM_MAX, int(round(float(value))))))


def clamp_joint_pwm_csv(csv_idx: int, value) -> int:
    spec = JOINT_PWM_SPECS.get(csv_idx)
    if spec is None:
        return clamp_arm_pwm(value)
    lo, hi = spec["min"], spec["max"]
    return int(max(lo, min(hi, int(round(float(value))))))


def clamp_joint_pwm_aux(aux: int, value) -> int:
    csv_idx = AUX_TO_CSV.get(int(aux))
    if csv_idx is None:
        return clamp_arm_pwm(value)
    return clamp_joint_pwm_csv(csv_idx, value)


def clamp_arm_pwm_list(pwms: list) -> list[int]:
    """Clamp 7 joint PWM values using per-joint limits at active CSV indices."""
    out = [clamp_arm_pwm(v) for v in pwms[:7]]
    while len(out) < 7:
        out.append(1500)
    for csv_idx in ARM_CSV_INDICES:
        out[csv_idx] = clamp_joint_pwm_csv(csv_idx, out[csv_idx])
    return out


def clamp_thr_pwm(value) -> int:
    return int(max(THR_PWM_MIN, min(THR_PWM_MAX, int(round(float(value))))))


_MAVPROXY_UDP_HOST_RE = re.compile(r"^udp(?:out)?:(?P<host>[^:]+):(?P<port>\d+)$", re.I)


def _parse_udp_endpoint(spec: str) -> tuple[str, int] | None:
    m = _MAVPROXY_UDP_HOST_RE.match(str(spec or "").strip())
    if not m:
        return None
    host = m.group("host").strip()
    if not host or host.startswith("127."):
        return None
    try:
        return host, int(m.group("port"))
    except (TypeError, ValueError):
        return None


def _local_ipv4_addresses() -> list[str]:
    """Best-effort local IPv4 list (hostname + route probes)."""
    addrs: set[str] = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            addrs.add(info[4][0])
    except OSError:
        pass
    for probe in ("8.8.8.8", "1.1.1.1"):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.connect((probe, 80))
                addrs.add(s.getsockname()[0])
            finally:
                s.close()
        except OSError:
            pass
    return sorted(a for a in addrs if a and not a.startswith("127."))


def local_ip_for_peer(peer_ip: str) -> str | None:
    """Local IPv4 on the same LAN as peer_ip (Pi→topside UDP return path)."""
    peer_ip = str(peer_ip or "").strip()
    if not peer_ip:
        return _local_ipv4_addresses()[0] if _local_ipv4_addresses() else None
    try:
        peer = ipaddress.ip_address(peer_ip)
        peer_net = ipaddress.ip_network(f"{peer_ip}/24", strict=False)
    except ValueError:
        peer = None
        peer_net = None

    if peer_net is not None:
        for addr in _local_ipv4_addresses():
            try:
                if ipaddress.ip_address(addr) in peer_net:
                    return addr
            except ValueError:
                continue

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((peer_ip, 9))
            ip = s.getsockname()[0]
        finally:
            s.close()
        if ip and not ip.startswith("127."):
            if peer is None or ipaddress.ip_address(ip) in peer_net:
                return ip
    except OSError:
        pass
    return None


def topside_return_ip(cfg: dict) -> str | None:
    """Best-guess IPv4 for diagnostics (Pi→topside UDP)."""
    explicit = str(cfg.get("topside_ip", "")).strip()
    if explicit:
        return explicit
    pi_ip = str(cfg.get("pi_ip", "")).strip()
    return local_ip_for_peer(pi_ip)
