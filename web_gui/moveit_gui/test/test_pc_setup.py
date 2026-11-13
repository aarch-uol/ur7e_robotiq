"""scripts/pc_setup.py doctor: machine detection and the image tag per CPU (no Docker or robot needed)."""
import importlib.util
import json
from pathlib import Path

spec = importlib.util.spec_from_file_location("pc_setup", Path(__file__).parents[1] / "scripts" / "pc_setup.py")
pc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pc)


def test_image_tag_per_cpu():
    assert pc.image_tag({"arch": "x86_64"}) == "ur7e_robotiq:jazzy"          # desktop PCs keep the old tag
    assert pc.image_tag({"arch": "aarch64"}) == "ur7e_robotiq:jazzy-arm64"


def test_jetson_detection(monkeypatch):
    files = {"/etc/nv_tegra_release": "# R36 (release), REVISION: 4.4, GCID: 1, BOARD: generic, EABI: aarch64",
             "/proc/device-tree/model": "Orin NX 16GB\0", "/sys/kernel/realtime": "1",
             "/proc/meminfo": "MemTotal:       16000000 kB", "/etc/os-release": 'PRETTY_NAME="Ubuntu 22.04.5 LTS"'}
    monkeypatch.setattr(pc, "read", lambda p, d="": files.get(str(p), d).replace("\0", "").strip())
    monkeypatch.setattr(pc.platform, "machine", lambda: "aarch64")
    m = pc.detect()
    assert (m["platform_kind"], m["l4t"], m["jetpack"], m["gpu"]) == ("jetson", "36.4.4", "6.2.1", "jetson")
    assert m["realtime"] and m["image"] == "ur7e_robotiq:jazzy-arm64"


def test_desktop_without_gpu(monkeypatch):
    monkeypatch.setattr(pc, "read", lambda p, d="": {"/proc/meminfo": "MemTotal: 8000000 kB"}.get(str(p), d))
    monkeypatch.setattr(pc, "run", lambda cmd, timeout=5.0: (127, ""))      # no nvidia-smi
    monkeypatch.setattr(pc.platform, "machine", lambda: "x86_64")
    m = pc.detect()
    assert (m["platform_kind"], m["gpu"], m["image"]) == ("desktop", "none", "ur7e_robotiq:jazzy")


def test_doctor_json_without_docker(monkeypatch, capsys):
    monkeypatch.setattr(pc, "run", lambda cmd, timeout=5.0: (127, ""))      # nothing installed at all
    code = pc.main(["doctor", "--json", "--robot-ip", "192.0.2.1"])
    out = json.loads(capsys.readouterr().out)
    assert code == 1 and out["worst"] == "FAIL"
    assert any(r["group"] == "docker" and r["what"] == "Docker is not installed" for r in out["checks"])
