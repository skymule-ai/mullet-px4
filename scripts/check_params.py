#!/usr/bin/env python3
"""Checks every baseline and profile against the parameter metadata embedded in the pinned PX4 firmware:
known name, right type, value within min/max, a listed enum value, only defined bitmask bits. Also checks
that set-profile.py reboots for every reboot_required profile parameter.

  check_params.py [--firmware px4_fmu-v6c_default.px4] [files...]   (default: drones/*/baseline.params profiles/*.params)
"""
import argparse
import base64
import hashlib
import importlib.util
import json
import sys
import urllib.request
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
# The firmware every drone runs. Change together with the README.
FIRMWARE_URL = "https://github.com/PX4/PX4-Autopilot/releases/download/v1.17.0/px4_fmu-v6c_default.px4"
FIRMWARE_SHA256 = "0d9ac77b8cdaf30a7cb1127200e20f9d42e7586a6b47d6bf32444b5852e98387"
# mullet1's v1.17.0 build also has the CRSF RC driver (crsf_rc), which the release build leaves out.
EXTRA_PARAMS = {"RC_CRSF_PRT_CFG", "RC_CRSF_TEL_EN"}

spec = importlib.util.spec_from_file_location("set_profile", ROOT / "scripts" / "set-profile.py")
set_profile = importlib.util.module_from_spec(spec)
spec.loader.exec_module(set_profile)


def metadata(firmware):
    data = Path(firmware).read_bytes() if firmware else urllib.request.urlopen(FIRMWARE_URL).read()
    if hashlib.sha256(data).hexdigest() != FIRMWARE_SHA256:
        sys.exit(f"firmware sha256 is not the pinned {FIRMWARE_SHA256}")
    fw = json.loads(data)
    xml = zlib.decompress(base64.b64decode(fw["parameter_xml"]))
    image = zlib.decompress(base64.b64decode(fw["image"]))
    return {p.get("name"): p for p in ET.fromstring(xml).iter("parameter")}, image


def errors(path, meta, image):
    out = []
    for name, (value, ptype) in set_profile.read_params_file(path).items():
        p = meta.get(name)
        if p is None:
            # _HASH_CHECK is in the firmware's parameter table without metadata: the name is all we can check.
            if name.encode() + b"\0" not in image and name not in EXTRA_PARAMS:
                out.append(f"{name}: unknown parameter")
            continue
        want = set_profile.INT32 if p.get("type") == "INT32" else set_profile.REAL32
        if ptype != want:
            out.append(f"{name}: type {ptype}, firmware says {p.get('type')}")
            continue
        if set_profile.IGNORE.fullmatch(name):
            continue  # counters PX4 updates itself (LND_FLIGHT_T_LO is a uint32 half: negative is fine)
        tol = 1e-6 * max(1.0, abs(value))
        if p.findtext("min") is not None and value < float(p.findtext("min")) - tol:
            out.append(f"{name} = {value}: below min {p.findtext('min')}")
        if p.findtext("max") is not None and value > float(p.findtext("max")) + tol:
            out.append(f"{name} = {value}: above max {p.findtext('max')}")
        codes = [float(v.get("code")) for v in p.iter("value")]
        if codes and not any(set_profile.same(float(value), c) for c in codes):
            out.append(f"{name} = {value}: not one of {codes}")
        bits = sum(1 << int(b.get("index")) for b in p.iter("bit"))
        if bits and int(value) & ~bits:
            out.append(f"{name} = {value}: bits outside mask {bits:#x}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--firmware", help="local copy of the pinned .px4 (default: download it)")
    ap.add_argument("files", nargs="*", type=Path)
    args = ap.parse_args()
    meta, image = metadata(args.firmware)
    files = args.files or sorted(ROOT.glob("drones/*/baseline.params")) + sorted(ROOT.glob("profiles/*.params"))
    failed = False
    for path in files:
        errs = errors(path, meta, image)
        if path.parent.name == "profiles":
            errs += [f"{n}: reboot_required but not in set-profile.py REBOOT_PARAMS"
                     for n in set_profile.read_params_file(path)
                     if n in meta and (meta[n].findtext("reboot_required") or "").lower() == "true"
                     and n not in set_profile.REBOOT_PARAMS]
        print(f"{'FAIL' if errs else 'ok  '} {path}")
        for e in errs:
            print(f"     {e}")
        failed |= bool(errs)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
