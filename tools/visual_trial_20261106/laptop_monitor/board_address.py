"""Follow a board whose Wi-Fi address changed (2026-11-11).

The GUI keeps the address the operator chose ("configured"). When a board stops
answering there, recover() looks for the same board and remembers a route:

1. MAC on this laptop's network: our boards' Wi-Fi MACs are learned after each
   successful connection (seeded with the three boards verified on 11/11). Only
   the address whose MAC matches is contacted, and its SSH host key is read
   without logging in (ssh-keyscan) and compared with the key stored for the
   configured address. Other teams' boards are never logged into.
2. Known board host names (mDNS), checked the same way.
3. The fixed Jetson <-> Pi5 wired link: reach Pi5 at 10.77.0.2 through the
   Jetson, or the Jetson at 10.77.0.1 through Pi5 (ssh -W), with HostKeyAlias
   set to the configured address so the stored host key still applies.

Every SSH user of a board asks route() for the current target, so commands,
the live display, logs and the planner follow the same recovered address.
"""
from __future__ import annotations

import ipaddress
import json
import os
import shlex
import socket
import subprocess
import threading
import time
from pathlib import Path

ROLES = ("pi5", "jetson", "pi4")
# Wi-Fi MACs verified on 2026-11-11 (MAC + host key + device model).
SEED_MACS = {"pi5": "2c:cf:67:65:bc:bd", "pi4": "d8:3a:dd:0d:81:38", "jetson": "e8:4e:06:aa:5c:ea"}
MDNS = {"pi5": ("user-desktop.local",), "pi4": ("lpp.local",), "jetson": ("ubuntu.local", "tegra-ubuntu.local")}
WIRED = {"pi5": ("user@10.77.0.2", "jetson"), "jetson": ("nvidia@10.77.0.1", "pi5")}
ACCOUNTS = {"pi5": "user", "pi4": "user", "jetson": "nvidia"}
# Another team's board on the shared network: never contacted.
FORBIDDEN = frozenset({"192.168.2.158"})
RETRY_AFTER_S = 20.0
_lock = threading.RLock()


def state_dir() -> Path:
    return Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "nomad-trial"


def _path() -> Path:
    return state_dir() / "board_routes.json"


def _load() -> dict:
    try:
        data = json.loads(_path().read_text(encoding="utf-8"))
        if isinstance(data, dict):
            return data
    except (OSError, ValueError):
        pass
    return {}


def _save(data: dict) -> None:
    path = _path()
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    temporary.chmod(0o600)
    os.replace(temporary, path)


def _address(host: str) -> str:
    return host.split("@", 1)[1]


def register(**hosts) -> None:
    """Record which configured address belongs to which board (pi5, jetson, pi4)."""
    with _lock:
        data = _load()
        roles = data.setdefault("roles", {})
        changed = False
        for role, host in hosts.items():
            if role in ROLES and isinstance(host, str) and "@" in host and roles.get(role) != host.strip():
                roles[role] = host.strip()
                changed = True
        if changed:
            _save(data)


def role_of(host: str) -> str | None:
    data = _load()
    for role, configured in data.get("roles", {}).items():
        if configured == host:
            return role
    for configured, route in data.get("current", {}).items():
        if route.get("host") == host:
            return role_of(configured)
    return None  # Only boards the GUI registered are ever looked for.


def route(host: str) -> dict:
    """Current target for a configured host: {host, alias, via, method}."""
    current = _load().get("current", {}).get(host)
    if isinstance(current, dict) and isinstance(current.get("host"), str):
        return {"host": current["host"], "alias": current.get("alias"), "via": current.get("via"),
                "method": current.get("method")}
    return {"host": host, "alias": None, "via": None, "method": None}


def ssh_route_options(target: dict, common: list[str]) -> list[str]:
    """Extra ssh -o options for a route; common are the caller's own -o options."""
    options = []
    if target.get("via"):
        hop = " ".join(shlex.quote(part) for part in ["ssh", *common, "-W", "%h:%p", target["via"]])
        options += ["-o", "ProxyCommand=" + hop]
    if target.get("alias"):
        options += ["-o", "HostKeyAlias=" + target["alias"]]
    return options


def forget(host: str) -> None:
    with _lock:
        data = _load()
        if data.get("current", {}).pop(host, None) is not None:
            _save(data)


def learn(host: str, used: dict | None = None) -> None:
    """After a successful direct connection, remember the board's Wi-Fi MAC."""
    used = used or route(host)
    if used.get("via"):
        return
    role = role_of(host)
    if role is None:
        return
    mac = _neighbour_macs().get(_address(used["host"]))
    if mac:
        with _lock:
            data = _load()
            if data.setdefault("macs", {}).get(role) != mac:
                data["macs"][role] = mac
                _save(data)


def _run(args, timeout):
    try:
        return subprocess.run(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                              timeout=timeout, check=False).stdout.decode("utf-8", "replace")
    except Exception:  # missing tool, timeout, or a test double: no evidence
        return ""


def _neighbour_macs() -> dict:
    try:
        rows = json.loads(_run(["ip", "-j", "-4", "neigh", "show"], 3) or "[]")
    except ValueError:
        return {}
    return {row["dst"]: row["lladdr"].lower() for row in rows
            if isinstance(row, dict) and row.get("dst") and row.get("lladdr")
            and row.get("state") not in (["FAILED"], ["INCOMPLETE"])}


def _local_networks():
    try:
        rows = json.loads(_run(["ip", "-j", "-4", "address", "show"], 3) or "[]")
    except ValueError:
        return [], set()
    networks, local = [], set()
    for row in rows:
        if row.get("ifname") == "lo" or str(row.get("ifname", "")).startswith(("docker", "br-", "veth")):
            continue
        for info in row.get("addr_info", []):
            if info.get("family") != "inet":
                continue
            local.add(info["local"])
            network = ipaddress.ip_interface(f"{info['local']}/{info['prefixlen']}").network
            if 16 <= network.prefixlen < 31 and network.num_addresses <= 4096:
                networks.append(network)
    return networks, local


def _wake_neighbours(networks, local, found=None, batch=128) -> None:
    """One tiny UDP datagram per address makes the kernel resolve its MAC (ARP).
    Nothing is opened or logged into; the forbidden board is skipped. Batches keep
    the kernel neighbour table from overflowing; found() ends the sweep early."""
    addresses = [str(a) for network in networks for a in network.hosts()
                 if str(a) not in local and str(a) not in FORBIDDEN]
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setblocking(False)
        for start in range(0, len(addresses), batch):
            for text in addresses[start:start + batch]:
                try:
                    sock.sendto(b"", (text, 9))
                except OSError:
                    pass
            time.sleep(0.4)
            if found is not None and found():
                return
    time.sleep(1.0)


def _stored_keys(alias: str, known_hosts_files) -> set:
    keys = set()
    for path in known_hosts_files:
        if not Path(path).is_file():
            continue
        for line in _run(["ssh-keygen", "-F", alias, "-f", str(path)], 3).splitlines():
            parts = line.split()
            if len(parts) >= 3 and not line.startswith("#"):
                keys.add((parts[1], parts[2]))
    return keys


def _scanned_keys(address: str) -> set:
    keys = set()
    for line in _run(["ssh-keyscan", "-T", "3", address], 6).splitlines():
        parts = line.split()
        if len(parts) >= 3 and not line.startswith("#"):
            keys.add((parts[1], parts[2]))
    return keys


def _same_board(address: str, alias: str, known_hosts_files) -> bool:
    if address in FORBIDDEN:
        return False
    stored = _stored_keys(alias, known_hosts_files)
    if not stored:
        return True  # No stored key yet: the matching learned MAC is the evidence.
    return bool(stored & _scanned_keys(address))


def _ssh_open(address: str, timeout=1.5) -> bool:
    if address in FORBIDDEN:
        return False
    try:
        with socket.create_connection((address, 22), timeout=timeout):
            return True
    except OSError:
        return False


def default_known_hosts():
    return [state_dir() / "known_hosts"]


def recover(host: str, known_hosts_files=None, *, allow_scan=True) -> dict | None:
    """Find the board behind a configured host again. Returns the new route or None.
    If a remembered detour exists and the configured address itself answers, the
    detour is forgotten and the configured address is returned (method "configured")."""
    role = role_of(host)
    if role is None:
        return None
    alias = _address(host)
    known_hosts_files = list(known_hosts_files or default_known_hosts())
    # 2026-11-12: a remembered detour (e.g. hotspot 10.42.0.x) failed but the configured
    # address answers again (back on the classroom Wi-Fi): drop the detour and go direct.
    # SSH's own host-key check still verifies the board at that address.
    if host in _load().get("current", {}) and _ssh_open(alias):
        forget(host)
        return {"host": host, "alias": None, "via": None, "method": "configured"}
    with _lock:
        data = _load()
        last = data.setdefault("attempts", {}).get(host, 0)
        if time.time() - last < RETRY_AFTER_S:
            return None
        data["attempts"][host] = time.time()
        _save(data)
    account = ACCOUNTS[role]
    mac = _load().get("macs", {}).get(role) or SEED_MACS.get(role)
    found = None
    if mac and allow_scan:
        def by_mac():
            return [ip for ip, value in _neighbour_macs().items() if value == mac and ip != alias]
        candidates = by_mac()
        if not candidates:
            networks, local = _local_networks()
            if networks:
                _wake_neighbours(networks, local, found=lambda: bool(by_mac()))
                for _ in range(6):  # late replies
                    candidates = by_mac()
                    if candidates:
                        break
                    time.sleep(0.5)
        for address in candidates:
            if _ssh_open(address) and _same_board(address, alias, known_hosts_files):
                found = {"host": f"{account}@{address}", "alias": alias, "via": None, "method": "mac"}
                break
    if found is None:
        for name in MDNS.get(role, ()):
            for line in _run(["getent", "ahostsv4", name], 3).splitlines():
                address = line.split()[0] if line.split() else ""
                if address and address != alias and _ssh_open(address) and \
                        _same_board(address, alias, known_hosts_files):
                    found = {"host": f"{account}@{address}", "alias": alias, "via": None, "method": "mdns"}
                    break
            if found:
                break
    if found is None and role in WIRED:
        wired_host, peer_role = WIRED[role]
        peer_configured = _load().get("roles", {}).get(peer_role)
        if peer_configured:
            peer = route(peer_configured)
            if not peer.get("via") and _ssh_open(_address(peer["host"])):
                found = {"host": wired_host, "alias": alias, "via": peer["host"], "method": "wired"}
    if found is None:
        return None
    with _lock:
        data = _load()
        data.setdefault("current", {})[host] = {**found, "role": role, "t": time.time()}
        _save(data)
    return found
