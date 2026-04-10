#!/usr/bin/env python3
"""
fix_mavlink_headers.py

Two fixes applied together before building ArduPlane:

Fix 1 — Remove spurious pre-generated MAVLink headers from the source tree.
  Cause: libraries/GCS_MAVLink/include/ exists in the source tree (not tracked
  by git, likely from a manual mavgen run or stale copy).  The waf build system
  also generates identical headers into build/<board>/libraries/GCS_MAVLink/
  include/.  Both paths land on the compiler's -I list, so mavlink_types.h is
  included twice → fatal redefinition of 'param_union' error.

Fix 2 — Apply TRACKING_MESSAGE patch to the mavlink submodule.
  Cause: upstream ardupilotmega.xml does not define TRACKING_MESSAGE (ID 11045).
  Without it, mavlink_get_msg_entry() silently drops the message before it
  reaches handle_tracking_message() in ArduPlane.  The patch adds the message
  definition so waf's mavgen step generates the correct C header and CRC table.

Usage:
  python3 fix_mavlink_headers.py           # apply all fixes
  python3 fix_mavlink_headers.py --check   # check status only, no changes
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def fix_spurious_headers(check_only: bool) -> bool:
    stale = ROOT / "libraries" / "GCS_MAVLink" / "include"
    if stale.is_dir():
        if check_only:
            print(f"[CHECK] FAIL — spurious headers exist: {stale}")
            print("         Run without --check to remove them.")
            return False
        print(f"[fix 1] Removing spurious headers: {stale}")
        shutil.rmtree(stale)
        print("[fix 1] Done.")
    else:
        print("[fix 1] OK — no spurious headers found.")
    return True


def fix_tracking_message(check_only: bool) -> bool:
    patch_file = ROOT / "fix_mavlink_tracking_message.patch"
    ardupilotmega = (
        ROOT / "modules" / "mavlink" / "message_definitions" / "v1.0" / "ardupilotmega.xml"
    )

    if not patch_file.is_file():
        print(f"[fix 2] ERROR — patch file not found: {patch_file}", file=sys.stderr)
        return False

    if "TRACKING_MESSAGE" in ardupilotmega.read_text():
        print("[fix 2] OK — TRACKING_MESSAGE already present in ardupilotmega.xml.")
        return True

    if check_only:
        print("[CHECK] FAIL — TRACKING_MESSAGE missing from ardupilotmega.xml")
        print("         Run without --check to apply the patch.")
        return False

    print("[fix 2] Applying TRACKING_MESSAGE patch...")
    result = subprocess.run(
        ["git", "apply", str(patch_file)],
        cwd=ROOT / "modules" / "mavlink",
    )
    if result.returncode != 0:
        print("[fix 2] ERROR — git apply failed.", file=sys.stderr)
        return False
    print("[fix 2] Done.")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="Check status only, make no changes")
    args = parser.parse_args()

    ok1 = fix_spurious_headers(args.check)
    ok2 = fix_tracking_message(args.check)

    if not args.check:
        print("\nAll fixes applied. Next steps:")
        print("  ./waf distclean")
        print("  ./waf configure --board <board>")
        print("  ./waf plane")

    return 0 if (ok1 and ok2) else 1


if __name__ == "__main__":
    sys.exit(main())
