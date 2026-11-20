"""brain_slots.py -- what is in each slot of the brain on SHULIB_BRAIN_PORT (its SYSTEM port), for route_builder.py's
"What is on robot one" (RMP1-K D5). Run ONLY by `pros user-script tools/brain_slots.py` (pros-cli's own Python, so
pros-cli's own V5Device reads the brain), never by itself. Read-only: it lists the brain's files and reads each
slot_N.ini as pros-cli's own `used_slots()` does. `pros v5 read-file` cannot do this: pros-cli 3.5.6 declares an
`outfile` argument its function never takes, so it stops with a TypeError before it reads anything.

Prints ONE line, after whatever pros-cli prints itself ("Loading ...", "Downloading slot_1.ini"):
    SHULIB_SLOTS {"slots": {"1": {"name": ..., "description": ..., "date": ...} or {"unread": why}}} or {"error": why}
Python 3.9 (the PROS extension's pros-cli is a frozen Python 3.9).
"""
import json
import os
import re


def read():
    from pros.serial.devices.vex import V5Device
    from pros.serial.ports import DirectPort
    device = V5Device(DirectPort(os.environ["SHULIB_BRAIN_PORT"]))
    found = set()
    for i in range(device.get_dir_count(vid=1)):
        m = re.fullmatch(r"slot_(\d+)\.ini", str(device.get_file_metadata_by_idx(i).get("filename", "")))
        if m:
            found.add(int(m.group(1)))
    slots = {}
    for n in sorted(found):
        try:  # one slot that cannot be read never hides the others
            ini = device.read_ini("slot_%d.ini" % n)
            if ini is None or not ini.has_section("program"):
                slots[str(n)] = {"unread": "pros could not read slot_%d.ini" % n}
                continue
            # raw: pros's plain ConfigParser would read a '%' (another tool's "100% auton") as interpolation and raise
            slots[str(n)] = {k: ini["program"].get(k, "", raw=True) for k in ("name", "description", "date")}
        except Exception as e:  # noqa: BLE001
            slots[str(n)] = {"unread": "%s: %s" % (type(e).__name__, e)}
    return {"slots": slots}


try:
    out = read()
except Exception as e:  # noqa: BLE001 -- said, never a traceback the page cannot read
    out = {"error": "%s: %s" % (type(e).__name__, e)}
print("\nSHULIB_SLOTS " + json.dumps(out), flush=True)
