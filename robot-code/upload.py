"""
LeadMe — Arduino uploader (no Arduino IDE required)
Uses arduino-cli under the hood. Installs it automatically if missing.

Usage:
    python upload.py                   # auto-detect board + port
    python upload.py --port /dev/ttyACM0
    python upload.py --port COM3
    python upload.py --fqbn arduino:avr:mega   # if using a Mega
"""

import argparse
import platform
import shutil
import subprocess
import sys
import os
import urllib.request

# ── Config ────────────────────────────────────────────────────────────────────

SKETCH_DIR = os.path.dirname(os.path.abspath(__file__))   # robot-code/
DEFAULT_FQBN = "arduino:avr:uno"

# arduino-cli install location when not on PATH
INSTALL_DIR = os.path.expanduser("~/.arduino-cli")
CLI = os.path.join(INSTALL_DIR, "arduino-cli")


# ── Helpers ───────────────────────────────────────────────────────────────────

def run(cmd: list[str], check=True, capture=False) -> subprocess.CompletedProcess:
    print(f"  $ {' '.join(cmd)}")
    return subprocess.run(
        cmd,
        check=check,
        capture_output=capture,
        text=True,
    )


def cli(*args, **kwargs):
    """Run arduino-cli with the given arguments."""
    return run([CLI] + list(args), **kwargs)


# ── Install arduino-cli ───────────────────────────────────────────────────────

def install_arduino_cli():
    system = platform.system()
    machine = platform.machine().lower()

    # Map to arduino-cli release asset names
    os_map  = {"Darwin": "macOS", "Linux": "Linux", "Windows": "Windows"}
    cpu_map = {"x86_64": "64bit", "amd64": "64bit",
               "arm64": "ARM64", "aarch64": "ARM64",
               "armv7l": "ARMv7"}

    os_name  = os_map.get(system)
    cpu_name = cpu_map.get(machine)

    if not os_name or not cpu_name:
        sys.exit(f"Unsupported platform: {system} {machine}. "
                 "Install arduino-cli manually: https://arduino.github.io/arduino-cli/")

    ext      = "zip" if system == "Windows" else "tar.gz"
    filename = f"arduino-cli_latest_{os_name}_{cpu_name}.{ext}"
    url      = f"https://downloads.arduino.cc/arduino-cli/{filename}"

    os.makedirs(INSTALL_DIR, exist_ok=True)
    archive  = os.path.join(INSTALL_DIR, filename)

    print(f"\nDownloading arduino-cli from {url} ...")
    urllib.request.urlretrieve(url, archive)

    print("Extracting ...")
    if ext == "tar.gz":
        run(["tar", "-xzf", archive, "-C", INSTALL_DIR])
    else:
        import zipfile
        with zipfile.ZipFile(archive) as z:
            z.extractall(INSTALL_DIR)

    os.remove(archive)

    binary = CLI + (".exe" if system == "Windows" else "")
    if not os.path.isfile(binary):
        # Some releases nest inside a subdirectory — search for it
        for root, _, files in os.walk(INSTALL_DIR):
            for f in files:
                if f.startswith("arduino-cli"):
                    found = os.path.join(root, f)
                    shutil.move(found, binary)
                    break

    os.chmod(binary, 0o755)
    print(f"arduino-cli installed at {binary}\n")


def ensure_cli():
    global CLI
    # Check system PATH first
    on_path = shutil.which("arduino-cli")
    if on_path:
        CLI = on_path
        return

    # Check our local install
    if not os.path.isfile(CLI) and not os.path.isfile(CLI + ".exe"):
        print("arduino-cli not found — installing...")
        install_arduino_cli()
    else:
        print(f"Using local arduino-cli at {CLI}")


# ── Board core ────────────────────────────────────────────────────────────────

def ensure_core(fqbn: str):
    """Install the core for the given FQBN if not already installed."""
    core = fqbn.split(":")[0] + ":" + fqbn.split(":")[1]   # e.g. arduino:avr
    result = cli("core", "list", capture=True, check=False)
    if core in (result.stdout or ""):
        print(f"Core {core} already installed.")
        return

    print(f"Updating index and installing core {core} ...")
    cli("core", "update-index")
    cli("core", "install", core)


# ── Board detection ───────────────────────────────────────────────────────────

def detect_board() -> tuple[str, str]:
    """
    Return (port, fqbn) for the first detected Arduino.
    Raises RuntimeError if none found.
    """
    result = cli("board", "list", capture=True, check=False)
    lines  = (result.stdout or "").splitlines()

    for line in lines[1:]:   # skip header
        parts = line.split()
        if len(parts) < 2:
            continue
        port = parts[0]
        # arduino-cli prints fqbn somewhere in the line
        fqbn_part = [p for p in parts if p.count(":") == 2]
        fqbn = fqbn_part[0] if fqbn_part else DEFAULT_FQBN
        if port and ("ttyACM" in port or "ttyUSB" in port or
                     "COM" in port or "cu." in port or "tty." in port):
            return port, fqbn

    raise RuntimeError(
        "No Arduino detected. Check USB connection.\n"
        f"arduino-cli board list output:\n{result.stdout}"
    )


# ── Compile + upload ──────────────────────────────────────────────────────────

def compile_sketch(fqbn: str):
    print(f"\nCompiling sketch for {fqbn} ...")
    cli("compile", "--fqbn", fqbn, SKETCH_DIR)
    print("Compile OK.")


def upload_sketch(port: str, fqbn: str):
    print(f"\nUploading to {port} ({fqbn}) ...")
    cli("upload", "-p", port, "--fqbn", fqbn, SKETCH_DIR)
    print("Upload OK.")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Upload Arduino firmware without Arduino IDE")
    parser.add_argument("--port", default=None, help="Serial port (e.g. /dev/ttyACM0 or COM3)")
    parser.add_argument("--fqbn", default=None, help=f"Board FQBN (default: {DEFAULT_FQBN})")
    args = parser.parse_args()

    ensure_cli()

    fqbn = args.fqbn or DEFAULT_FQBN
    ensure_core(fqbn)

    if args.port:
        port = args.port
        print(f"\nUsing specified port: {port}")
    else:
        print("\nDetecting Arduino ...")
        port, detected_fqbn = detect_board()
        if not args.fqbn:
            fqbn = detected_fqbn
        print(f"Found: {port}  ({fqbn})")

    compile_sketch(fqbn)
    upload_sketch(port, fqbn)

    print(f"\nDone. Firmware uploaded to {port}.")


if __name__ == "__main__":
    main()
