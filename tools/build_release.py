#!/usr/bin/env python3
"""
Assemble the deployable release folder.

    python3 tools/build_release.py           # rebuild release/
    python3 tools/build_release.py --check   # verify release/ is up to date

The release is committed so anyone can clone and deploy without running
anything, which is the point of it - reflashing a spare board should not
require a toolchain. That does mean it can go stale, so `--check` exists to
catch that, and returns non-zero if release/ no longer matches the sources.

release/README.md is hand-written and is left alone.
"""

import filecmp
import hashlib
import pathlib
import shutil
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
RELEASE = ROOT / "release"
LIB_SOURCE = ROOT / "install bundle" / "lib"

# (source, destination-relative-to-release)
FILES = [
    (ROOT / "code.py", "CIRCUITPY/code.py"),
    (ROOT / "config.py", "CIRCUITPY/config.py"),
    (ROOT / "tools" / "osc_tuner.py", "tuner/osc_tuner.py"),
    (ROOT / "tools" / "Encoder Tuner.command", "tuner/Encoder Tuner.command"),
    (ROOT / "zero_reset.py", "utilities/zero_reset.py"),
    (ROOT / "address_config.py", "utilities/address_config.py"),
]
TREES = [(LIB_SOURCE, "CIRCUITPY/lib")]
GENERATED = ["CIRCUITPY", "tuner", "utilities", "MANIFEST.txt"]


def iter_pairs():
    """Yield (source_path, dest_path) for every file in the release."""
    for src, rel in FILES:
        yield src, RELEASE / rel
    for src_root, rel_root in TREES:
        for src in sorted(src_root.rglob("*")):
            if src.is_file():
                yield src, RELEASE / rel_root / src.relative_to(src_root)


def build():
    missing = [str(s) for s, _ in FILES if not s.exists()]
    if not LIB_SOURCE.is_dir():
        missing.append(str(LIB_SOURCE))
    if missing:
        print("Cannot build, missing:\n  " + "\n  ".join(missing))
        return 1

    for name in GENERATED:
        target = RELEASE / name
        if target.is_dir():
            shutil.rmtree(target)
        elif target.exists():
            target.unlink()

    count = 0
    lines = []
    for src, dst in iter_pairs():
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        digest = hashlib.sha256(dst.read_bytes()).hexdigest()
        lines.append("%s  %s" % (digest, dst.relative_to(RELEASE).as_posix()))
        count += 1

    # A manifest so a copy onto a board can be verified - a truncated file on
    # a FAT drive is otherwise easy to miss.
    (RELEASE / "MANIFEST.txt").write_text(
        "# sha256  path (relative to release/)\n" + "\n".join(sorted(lines)) + "\n"
    )
    print("Built release/ with %d files." % count)
    return 0


def check():
    stale = []
    for src, dst in iter_pairs():
        if not dst.exists():
            stale.append("missing: %s" % dst.relative_to(RELEASE).as_posix())
        elif not filecmp.cmp(src, dst, shallow=False):
            stale.append("differs: %s" % dst.relative_to(RELEASE).as_posix())

    expected = {dst for _, dst in iter_pairs()}
    for name in GENERATED:
        target = RELEASE / name
        if target.is_dir():
            for found in target.rglob("*"):
                if found.is_file() and found not in expected:
                    stale.append("stray:   %s" % found.relative_to(RELEASE).as_posix())

    if stale:
        print("release/ is out of date:")
        for line in stale:
            print("  " + line)
        print("\nRun: python3 tools/build_release.py")
        return 1
    print("release/ is up to date.")
    return 0


if __name__ == "__main__":
    sys.exit(check() if "--check" in sys.argv else build())
