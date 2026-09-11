"""Verify deterministic field physics by replaying the recorded motor frames."""

import argparse
import hashlib
import json
from pathlib import Path
from .field_adapter import FieldPlant


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    plant = FieldPlant()
    original = hashlib.sha256()
    replayed = hashlib.sha256()
    count = 0
    with (args.run / "frames.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            state = plant.step(row["pwm"])
            encoded = json.dumps(state, sort_keys=True).encode()
            original.update(json.dumps(row["state_sent"], sort_keys=True).encode())
            replayed.update(encoded)
            count += 1
            if state != row["state_sent"]:
                raise RuntimeError(
                    f'physics replay diverged at frame {row["frame_count"]}'
                )
    result = {
        "frames": count,
        "identical_state_history": original.hexdigest() == replayed.hexdigest(),
        "original_sha256": original.hexdigest(),
        "replay_sha256": replayed.hexdigest(),
    }
    (args.run / "replay.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
