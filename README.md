# mullet-px4

The one source for every mullet drone's PX4 setup: the firmware release, each drone's parameter baseline and
the localization profiles. Parameter changes on a drone go only through this repo.

## Firmware
Every drone runs stock **PX4 v1.17.0** (git `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`) on the Pixhawk 6C:

| File | sha256 |
| --- | --- |
| [`px4_fmu-v6c_default.px4`](https://github.com/PX4/PX4-Autopilot/releases/download/v1.17.0/px4_fmu-v6c_default.px4) | `0d9ac77b8cdaf30a7cb1127200e20f9d42e7586a6b47d6bf32444b5852e98387` |

mullet1 runs a v1.17.0 build (same git hash) that also has the CRSF RC driver: its baseline has
`RC_CRSF_PRT_CFG` and `RC_CRSF_TEL_EN`, which the release build lacks. `check_params.py` lists them in
`EXTRA_PARAMS`. A drone flashed with the release above will not have them.

Flash it with QGroundControl: *Vehicle Setup → Firmware*, unplug and replug the Pixhawk over USB, tick
*Advanced settings → Custom firmware file*, pick the downloaded `.px4` (check `sha256sum` first). Then load
the drone's `baseline.params` (*Parameters → Tools → Load from file*) and reboot. The version is pinned in
`scripts/check_params.py` too: change both together.

## Layout
- `drones/<name>/baseline.params`: the drone's full parameter dump (QGC format) in its known-good outdoor
  state. It is on the default profile.
- `profiles/<name>.params`: only the parameters a profile sets, in QGC format (QGC can load them too).
  Every profile sets the same names.
- `profiles/profiles.yaml`: profile → params file, bridge mode, note.
- `scripts/set-profile.py`: applies, shows and backs up profiles on the drone.
- `scripts/check_params.py`: CI check of every file against the firmware's parameter metadata.

## Profiles
All of them also set `EKF2_RNG_A_VMAX 2.0`, `EKF2_MIN_RNG 0.21` and `EKF2_MAG_CHECK` (1 = field strength
check, the baseline value; `outdoor-vio` sets 3 = strength + inclination).

**Temporary (issue [#4](https://github.com/skymule-ai/mullet-px4/issues/4)):** every profile also sets `SENS_FLOW_SCALE 1.95`, `EKF2_OF_GYR_SRC 1` and
`EKF2_OF_QMIN 140`. The hflow board labels each flow message as 15.872 ms when it covers ~8.1 ms, so flow reads at 0.51x. `drones/mullet1/baseline.params` carries the
same values. To revert after the firmware fix, set 1 and 0 back in the profiles and the baseline: don't delete the profile
lines, `set-profile.py` only writes what the profiles set.

| Profile | `EKF2_EV_CTRL` | `EKF2_MAG_TYPE` | Bridge | Use |
| --- | --- | --- | --- | --- |
| `outdoor` (default) | 0 | 0 | shadow | GPS + compass; vision recorded only |
| `outdoor-vio` | 5 | 0 | fused | vision position + velocity fused alongside GPS and compass; for outdoor ↔ indoor walks (see below) |
| `indoor` | 13 | 5 | fused | vision position + heading, compass off. **Bench/walk only**: PX4 does not yet align yaw on vision heading |

`outdoor-vio` across outdoor ↔ indoor: EKF2 learns the rotation from the vision frame to NED while GPS and
compass are fused, and keeps fusing vision position + velocity through it when they drop out, so the
heading holds without a reset. The compass stops on its own where the field fails the strength or
inclination check, or disagrees with the vision-aided heading. Vision yaw is left off on purpose: with an
FRD vision frame PX4 blocks it while GPS is fused and, when GPS drops, resets the heading to the raw vision
yaw (`ev_yaw_control.cpp`). Start where GPS and the compass are good.

Bridge = `px4_ev_bridge` in mullet-localization (`PUBLISH_TO_PX4` in its `.env`): shadow publishes the
odometry on `/localization/vehicle_odometry` only, fused also sends it to PX4.

## On the drone
Clone once to `~/mullet-px4` on the Orin host (pymavlink and PyYAML are installed there). The script talks to
PX4 through mavlink-router (`tcp:127.0.0.1:5760`) and finds the baseline by hostname (`--drone` overrides).

```bash
cd ~/mullet-px4 && git pull
scripts/set-profile.py --show          # start of day: profile, bridge mode, drift; exit 1 on any problem
scripts/set-profile.py outdoor-vio     # switch profile
scripts/set-profile.py --backup        # dump everything to backups/<drone>-<UTC>.params
```

Switching a profile:
1. refuses while armed;
2. backs up every parameter first;
3. writes the profile's parameters that differ, then reads every one back (stops on a mismatch);
4. sets `PUBLISH_TO_PX4` in `~/mullet-localization/.env` and runs `docker compose up -d ev_bridge`;
5. if `EKF2_MAG_TYPE` changed, asks before rebooting the Pixhawk (only while disarmed); it takes effect only
   after the reboot. It saves the parameters to flash first (PX4's own autosave is delayed, and a reboot
   can beat it) and reads the profile back once PX4 is up again.

`--show` reports drift: any parameter that differs from `baseline.params` + the active profile.
`_HASH_CHECK`, `CAL_BARO*_OFF` and the flight counters change by themselves and are ignored.

## Parameter-backup rule
- Before any parameter change, on any drone: `set-profile.py --backup` (switching a profile does it for you).
  Changing parameters in QGC is for recovery only; afterwards run `--show` and bring the repo in line.
- A wanted change to a drone's baseline (calibration, tuning) is a PR here: dump with `--backup`, copy it over
  `drones/<name>/baseline.params`, and say why in the PR. CI checks it against the firmware metadata.
- Keep the backups you took for a field day in the field notes (`field/params/`).

## CI
`scripts/check_params.py` downloads the pinned `.px4`, checks its sha256 and validates every baseline and
profile against the parameter metadata embedded in it (unknown name, wrong type, out of range, not a listed
value, undefined bit). `scripts/set-profile.py --selftest` checks profile matching, drift and this README's
table. Run both locally with `pip install pyyaml` and:

```bash
scripts/check_params.py && scripts/set-profile.py --selftest
```
