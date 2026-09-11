"""Compile unchanged upstream checks against the installed MuJoCo library."""

from pathlib import Path
import json
import subprocess
import mujoco

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT.parent / "rmuc2026_battlefield_mjcf"


def main():
    output = ROOT / "rm_world/artifacts/upstream"
    output.mkdir(parents=True, exist_ok=True)
    library = Path(mujoco.__path__[0])
    commands = []
    failed = False
    for name in [
        "check_commanding_heights",
        "check_undulating_road",
        "check_ramp_barrier_crossing",
    ]:
        cmd = ["g++", "-O2", "-I" + str(library / "include")]
        if tuple(map(int, mujoco.__version__.split(".")[:2])) >= (3, 12):
            cmd += ["-include", str(ROOT / "rm_world/ray_compat.hpp")]
        cmd += [
            str(SOURCE / "tools" / f"{name}.cpp"),
            str(next(library.glob("libmujoco.so.*"))),
            "-Wl,-rpath," + str(library),
            "-o",
            str(output / name),
        ]
        commands.append(cmd)
        build = subprocess.run(cmd, capture_output=True, text=True)
        (output / f"{name}-build.log").write_text(build.stdout + build.stderr)
        if build.returncode:
            failed = True
            continue
        cmd = [str(output / name), str(SOURCE / "model.xml")]
        result = subprocess.run(cmd, capture_output=True, text=True)
        commands.append(cmd)
        (output / f"{name}.log").write_text(result.stdout + result.stderr)
        print(name, result.returncode, result.stdout[-600:])
        failed |= result.returncode != 0
    (output / "commands.json").write_text(json.dumps(commands, indent=2))
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
