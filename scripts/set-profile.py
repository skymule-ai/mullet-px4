#!/usr/bin/env python3
"""Apply, show or back up a mullet-px4 profile. Runs on the drone's Orin host, through mavlink-router.

  set-profile.py outdoor       write the profile, read it back, set the bridge mode, reboot if needed
  set-profile.py --show        active profile, bridge mode, and every parameter that drifted from baseline
  set-profile.py --backup      dump all parameters to backups/<drone>-<UTC time>.params
  set-profile.py --selftest    offline checks (CI)
"""
import argparse
import datetime
import re
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
ENV = Path.home() / "mullet-localization" / ".env"
INT32, REAL32 = 6, 9  # MAV_PARAM_TYPE, as in QGC files
# Profile parameters that need a Pixhawk reboot. check_params.py fails if a profile sets another
# parameter whose metadata says reboot_required.
REBOOT_PARAMS = {"EKF2_MAG_TYPE"}
# PX4 changes these by itself (hash, baro offset re-estimate, flight counters): never drift.
IGNORE = re.compile(r"_HASH_CHECK|CAL_BARO\d_OFF|LND_FLIGHT_T_(HI|LO)|COM_FLIGHT_UUID")


def read_params_file(path):
    params = {}
    for line in Path(path).read_text().splitlines():
        if line.strip() and not line.startswith("#"):
            _, _, name, value, ptype = line.split("\t")
            params[name] = (int(value) if int(ptype) == INT32 else float(value), int(ptype))
    return params


def write_params_file(path, params):
    lines = ["# Onboard parameters for Vehicle 1", "#", "# Vehicle-Id Component-Id Name Value Type"]
    for name, (value, ptype) in sorted(params.items()):
        lines.append(f"1\t1\t{name}\t{value if ptype == INT32 else format(value, '.18g')}\t{ptype}")
    Path(path).write_text("\n".join(lines) + "\n")


def load_profiles():
    cfg = yaml.safe_load((ROOT / "profiles" / "profiles.yaml").read_text())
    for p in cfg["profiles"].values():
        p["values"] = read_params_file(ROOT / "profiles" / p["params"])
    return cfg


def same(a, b):
    if a is None or b is None:
        return a is b
    return a == b if isinstance(b, int) else abs(a - b) <= 1e-6 * max(1.0, abs(b))


def matching(current, profiles):
    return [n for n, p in profiles.items()
            if all(same(current.get(k, (None,))[0], v) for k, (v, _) in p["values"].items())]


def drift(current, baseline, profile):
    expected = {**baseline, **profile["values"]}
    return [(k, expected.get(k, (None,))[0], current.get(k, (None,))[0])
            for k in sorted(expected.keys() | current.keys())
            if not IGNORE.fullmatch(k) and not same(current.get(k, (None,))[0], expected.get(k, (None,))[0])]


def set_env(text, key, value):
    line = f"{key}={value}"
    if re.search(rf"^{key}=", text, flags=re.M):
        return re.sub(rf"^{key}=.*$", line, text, flags=re.M)
    return text + ("" if text.endswith("\n") or not text else "\n") + line + "\n"


class Px4:
    def __init__(self, url):
        from pymavlink import mavutil
        self.M = mavutil.mavlink
        self.m = mavutil.mavlink_connection(url, source_system=254, source_component=190)

    def _recv(self, mtype, timeout):
        end = time.time() + timeout
        while time.time() < end:
            msg = self.m.recv_match(type=mtype, blocking=True, timeout=max(0.1, end - time.time()))
            if msg and msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1:
                return msg
        return None

    def armed(self):
        hb = self._recv("HEARTBEAT", 10)
        if hb is None:
            sys.exit("no PX4 heartbeat (is mavlink-router up?)")
        return bool(hb.base_mode & self.M.MAV_MODE_FLAG_SAFETY_ARMED)

    @staticmethod
    def _value(msg):
        if msg.param_type == REAL32:
            return msg.param_value, REAL32
        return struct.unpack("<i", struct.pack("<f", msg.param_value))[0], INT32  # int32 bits in a float

    def read(self, name):
        for _ in range(5):
            self.m.mav.param_request_read_send(1, 1, name.encode(), -1)
            end = time.time() + 2
            while (msg := self._recv("PARAM_VALUE", end - time.time())) is not None:
                if msg.param_id == name:
                    return self._value(msg)
        sys.exit(f"no reply reading {name}")

    def write(self, name, value, ptype):
        raw = value if ptype == REAL32 else struct.unpack("<f", struct.pack("<i", value))[0]
        self.m.mav.param_set_send(1, 1, name.encode(), raw, ptype)
        time.sleep(0.3)

    def read_all(self):
        self.m.mav.param_request_list_send(1, 1)
        params, total, last = {}, None, time.time()
        while time.time() - last < 5 and len(params) != total:
            msg = self._recv("PARAM_VALUE", 1)
            if msg:
                last, total = time.time(), msg.param_count
                params[msg.param_id] = self._value(msg)
        if len(params) != total:
            sys.exit(f"parameter dump incomplete: {len(params)} of {total}")
        return params

    def reboot(self):
        self.m.mav.command_long_send(1, 1, self.M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
        ack = self._recv("COMMAND_ACK", 5)
        print("reboot accepted" if ack and ack.result == 0 else f"reboot NOT accepted: {ack}")


def backup(px4, drone):
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    path = ROOT / "backups" / f"{drone}-{stamp}.params"
    path.parent.mkdir(exist_ok=True)
    params = px4.read_all()
    write_params_file(path, params)
    print(f"backup: {len(params)} parameters -> {path}")
    return params


def bridge_running():
    """publish_to_px4 of the running ev_bridge container, or None."""
    try:
        cid = subprocess.run(["docker", "compose", "ps", "-q", "ev_bridge"], cwd=ENV.parent,
                             capture_output=True, text=True).stdout.strip()
        cmd = subprocess.run(["docker", "inspect", "--format", "{{json .Config.Cmd}}", cid],
                             capture_output=True, text=True).stdout if cid else ""
    except FileNotFoundError:
        return None
    m = re.search(r"publish_to_px4:=(\w+)", cmd)
    return m and m.group(1) == "true"


def apply(px4, drone, name, cfg):
    profile = cfg["profiles"][name]
    if px4.armed():
        sys.exit("armed: refusing to change parameters")
    backup(px4, drone)
    changed = [k for k, (v, _) in profile["values"].items() if not same(px4.read(k)[0], v)]
    for k in changed:
        print(f"set {k} = {profile['values'][k][0]}")
        px4.write(k, *profile["values"][k])
    bad = [k for k, (v, _) in profile["values"].items() if not same(px4.read(k)[0], v)]
    if bad:
        sys.exit(f"read-back mismatch on {bad}: bridge and reboot left alone")
    print(f"read back {len(profile['values'])} parameters: OK")

    mode = "true" if profile["publish_to_px4"] else "false"
    ENV.write_text(set_env(ENV.read_text(), "PUBLISH_TO_PX4", mode))
    subprocess.run(["docker", "compose", "up", "-d", "ev_bridge"], cwd=ENV.parent, check=True)
    print(f"bridge: PUBLISH_TO_PX4={mode}, running {bridge_running()}")

    if REBOOT_PARAMS & set(changed):
        if input(f"{sorted(REBOOT_PARAMS & set(changed))} need a Pixhawk reboot. Reboot now? [y/N] ") != "y":
            print("NOT rebooted: the change takes effect at the next Pixhawk boot")
        elif px4.armed():
            sys.exit("armed: not rebooting")
        else:
            px4.reboot()


def show(px4, drone, cfg):
    current = px4.read_all()
    baseline = read_params_file(ROOT / "drones" / drone / "baseline.params")
    active = matching(current, cfg["profiles"])
    name = active[0] if active else cfg["default"]
    print(f"profile: {name}" if active else f"profile: NONE matches (drift shown against {name})")
    want = cfg["profiles"][name]["publish_to_px4"]
    running = bridge_running()
    print(f"bridge: running publish_to_px4={running}, profile wants {want}")
    diffs = drift(current, baseline, cfg["profiles"][name])
    for k, expected, got in diffs:
        print(f"drift: {k} expected {expected} got {got}")
    print("no drift" if not diffs else f"{len(diffs)} parameters drifted")
    return 0 if active and not diffs and running == want else 1


def selftest():
    cfg = load_profiles()
    profiles = cfg["profiles"]
    names = {frozenset(p["values"]) for p in profiles.values()}
    assert len(names) == 1, "every profile must set the same parameter names"
    assert cfg["default"] in profiles
    for drone in (ROOT / "drones").iterdir():
        baseline = read_params_file(drone / "baseline.params")
        assert matching(baseline, profiles) == [cfg["default"]], f"{drone.name} baseline is not on the default"
        for n, p in profiles.items():
            current = {**baseline, **p["values"]}
            assert matching(current, profiles) == [n], n
            assert drift(current, baseline, p) == [], n
        current = {**baseline, "_HASH_CHECK": (1, INT32), "CAL_BARO0_OFF": (1.0, REAL32)}
        assert drift(current, baseline, profiles[cfg["default"]]) == []
        current = {**baseline, "EKF2_EV_DELAY": (99.0, REAL32)}
        assert [d[0] for d in drift(current, baseline, profiles[cfg["default"]])] == ["EKF2_EV_DELAY"]
        assert drift({k: v for k, v in baseline.items() if k != "EKF2_EV_DELAY"}, baseline,
                     profiles[cfg["default"]])[0][0] == "EKF2_EV_DELAY"

    # README profiles table == profiles.yaml + profile files
    rows = re.findall(r"^\| `([\w-]+)`[^|]*\| (\d+) \| (\d+) \| (\w+) \|", (ROOT / "README.md").read_text(), re.M)
    assert {r[0] for r in rows} == set(profiles), rows
    for n, ev, mag, bridge in rows:
        v = profiles[n]["values"]
        assert (v["EKF2_EV_CTRL"][0], v["EKF2_MAG_TYPE"][0]) == (int(ev), int(mag)), n
        assert bridge == ("fused" if profiles[n]["publish_to_px4"] else "shadow"), n

    assert set_env("A=1\nPUBLISH_TO_PX4=false\n", "PUBLISH_TO_PX4", "true") == "A=1\nPUBLISH_TO_PX4=true\n"
    assert set_env("A=1", "PUBLISH_TO_PX4", "true") == "A=1\nPUBLISH_TO_PX4=true\n"
    with tempfile.TemporaryDirectory() as d:
        write_params_file(Path(d) / "x.params", baseline)
        assert read_params_file(Path(d) / "x.params") == baseline
    print("selftest OK")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("profile", nargs="?")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--backup", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--drone", default=socket.gethostname())
    ap.add_argument("--url", default="tcp:127.0.0.1:5760", help="mavlink-router endpoint")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    cfg = load_profiles()
    if args.profile and args.profile not in cfg["profiles"]:
        sys.exit(f"unknown profile {args.profile}; known: {', '.join(cfg['profiles'])}")
    if not (args.profile or args.show or args.backup):
        ap.error("give a profile, --show or --backup")
    px4 = Px4(args.url)
    if args.backup:
        backup(px4, args.drone)
    if args.profile:
        apply(px4, args.drone, args.profile, cfg)
    if args.show:
        return show(px4, args.drone, cfg)


if __name__ == "__main__":
    sys.exit(main())
