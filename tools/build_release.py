#!/usr/bin/env python3
"""
Package a release archive for attaching to a GitHub release.

    python3 tools/build_release.py

Produces `dist/gaytencoder-<version>.zip`, which unpacks to a single
versioned folder containing everything needed to take a bare Waveshare
ESP32-S3-ETH to a working bridge:

    gaytencoder-1.0/
      README.md          deployment guide
      CIRCUITPY/         copy the contents onto the board's drive
      tuner/             the configuration app, runs on a Mac or PC
      utilities/         one-off jobs (zero reset, encoder addressing)
      MANIFEST.txt       sha256 of every file

The version comes from VERSION in code.py, so the archive name and the string
the board prints at boot cannot disagree.

dist/ is deliberately not committed. A build artifact in the repository goes
stale the moment anything changes, and the archive belongs on the release
page instead.
"""

import hashlib
import pathlib
import re
import shutil
import sys
import tempfile
import zipfile

ROOT = pathlib.Path(__file__).resolve().parent.parent
DIST = ROOT / "dist"
LIB_SOURCE = ROOT / "install bundle" / "lib"

# (source, path inside the archive folder)
FILES = [
    (ROOT / "docs" / "DEPLOYING.md", "README.md"),
    (ROOT / "code.py", "CIRCUITPY/code.py"),
    (ROOT / "config.py", "CIRCUITPY/config.py"),
    (ROOT / "tools" / "osc_tuner.py", "tuner/osc_tuner.py"),
    (ROOT / "tools" / "Encoder Tuner.command", "tuner/Encoder Tuner.command"),
    (ROOT / "zero_reset.py", "utilities/zero_reset.py"),
    (ROOT / "address_config.py", "utilities/address_config.py"),
]
TREES = [(LIB_SOURCE, "CIRCUITPY/lib")]

# Kept executable inside the archive so the macOS launcher works on unzip.
EXECUTABLE = {"tuner/Encoder Tuner.command"}


def read_version():
    text = (ROOT / "code.py").read_text(encoding="utf-8")
    match = re.search(r'^VERSION = "([^"]+)"', text, re.M)
    if not match:
        raise SystemExit("No VERSION found in code.py")
    return match.group(1)


def collect():
    """Yield (source_path, archive_relative_path) for everything shipped."""
    for src, rel in FILES:
        yield src, rel
    for src_root, rel_root in TREES:
        for src in sorted(src_root.rglob("*")):
            if src.is_file():
                yield src, "%s/%s" % (rel_root, src.relative_to(src_root).as_posix())


def main():
    missing = [str(s) for s, _ in FILES if not s.exists()]
    if not LIB_SOURCE.is_dir():
        missing.append(str(LIB_SOURCE))
    if missing:
        print("Cannot build, missing:\n  " + "\n  ".join(missing))
        return 1

    version = read_version()
    stem = "gaytencoder-%s" % version
    DIST.mkdir(exist_ok=True)
    archive = DIST / (stem + ".zip")

    with tempfile.TemporaryDirectory() as tmp:
        staged = pathlib.Path(tmp) / stem
        entries = []
        for src, rel in collect():
            dst = staged / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            entries.append(
                "%s  %s" % (hashlib.sha256(dst.read_bytes()).hexdigest(), rel)
            )

        # A manifest so a copy onto a board can be checked: a FAT drive will
        # truncate a file without complaining, and a short
        # adafruit_wiznet5k.py is easy to miss.
        (staged / "MANIFEST.txt").write_text(
            "# gaytencoder %s\n# sha256  path\n%s\n"
            % (version, "\n".join(sorted(entries)))
        )

        if archive.exists():
            archive.unlink()
        with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as zf:
            for path in sorted(staged.rglob("*")):
                if not path.is_file():
                    continue
                rel = path.relative_to(staged).as_posix()
                info = zipfile.ZipInfo(("%s/%s" % (stem, rel)))
                info.date_time = (2026, 1, 1, 0, 0, 0)  # reproducible archives
                info.compress_type = zipfile.ZIP_DEFLATED
                # 0755 for the launcher, 0644 for everything else.
                info.external_attr = (0o755 if rel in EXECUTABLE else 0o644) << 16
                zf.writestr(info, path.read_bytes())

    size_kb = archive.stat().st_size / 1024
    print("Built %s (%d files, %.0f KB)" % (archive.name, len(entries) + 1, size_kb))
    print("Attach it to a GitHub release tagged v%s." % version)
    return 0


if __name__ == "__main__":
    sys.exit(main())
