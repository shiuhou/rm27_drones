"""Read-only ULog replay for the registered local quad-132p6g attitude fixture."""

import argparse
from hashlib import sha256
from io import BytesIO
import json
import math
from pathlib import Path

from rm27_exchange.offline_contracts import Blocked, FRAMES, VERSION


RECORDING = Path('data/flight_logs/data/2026-11-13_quad-132p6g_sysid/1113_z.ulg')
RECORDING_SHA256 = '9b0f34b71bd88327e4e5bffa99861522ee8cdc0aee5a29a77a2ba4e4f3808e77'
FIRMWARE = 'd6f12ad1c4f70ad3230afd7d86e971421e02fef4'
TOPIC = 'vehicle_attitude'
SOURCE = 'PX4_EKF2_vehicle_attitude'
RUN_ID = f'ulog-sha256:{RECORDING_SHA256}'
CLOCK = f'{RUN_ID}:px4-boot-clock'
FRAME = f'{RUN_ID}:attitude-local-NED/FRD'
# Scope is one recorded attitude segment, not a vehicle-wide epoch or UTC map.
# This window precedes the logged dropout near boot time 81 s.
START_US, END_US = 79_110_000, 79_160_000


def _microseconds(value):
    if type(value) is not int or value <= 0:
        raise Blocked('missing or invalid PX4 boot timestamp')
    return value


def _adapt_attitude(log_row, *, sequence, receiver_boot_us, max_age_us):
    sample = _microseconds(log_row['timestamp_sample'])
    published = _microseconds(log_row['timestamp'])
    now = _microseconds(receiver_boot_us)
    age = now - sample
    if log_row['quat_reset_counter'] != 0:
        raise Blocked('attitude reset outside registered epoch')
    if not START_US <= sample <= published <= now or published > END_US:
        raise Blocked('out-of-window or future attitude timestamp')
    if not 0 <= age <= _microseconds(max_age_us):
        raise Blocked('recorded attitude is stale')
    q = [float(log_row[f'q[{i}]']) for i in range(4)]
    norm = math.hypot(*q)
    if not math.isfinite(norm) or abs(norm - 1.) > 1e-6:
        raise Blocked('invalid recorded attitude quaternion')
    key = 'q_WB_wxyz'
    record_id = f'{RUN_ID}:{TOPIC}:0:{sequence}'
    unknown = {
        'position_W_m': 'COM origin is not registered for this recording',
        'velocity_W_m_s': 'COM origin is not registered for this recording',
        'body_rates_B_rad_s': 'No angular-rate sample was joined to this attitude',
        'uncertainty': 'No attitude covariance in this topic',
        'actuator_state': 'No actuator data in this topic',
    }
    return {
        'metadata': {
            'contract_id': 'rm27.state', 'contract_version': VERSION,
            'run_id': RUN_ID, 'source': SOURCE, 'sequence': sequence,
            'record_id': record_id, 'input_kind': 'ESTIMATED',
            'clock_id': CLOCK, 'frame_id': FRAME, 'reset_epoch': 0,
            'lineage': [{'input_kind': 'RECORDED', 'source': RUN_ID, 'record_id': record_id,
                         'transform': f'PX4:{FIRMWARE}:vehicle_attitude:boot-us-to-s'}],
            'availability': {name: {'status': 'UNKNOWN', 'reason': reason}
                             for name, reason in unknown.items()},
        },
        'frames': FRAMES.copy(), 'state_time_s': sample / 1_000_000,
        key: [x / norm for x in q],
        'measurement_times_s': {key: {'time_s': sample / 1_000_000, 'clock_id': CLOCK}},
        'published_time_s': published / 1_000_000, 'published_clock_id': CLOCK,
        'data_age_s': {key: age / 1_000_000},
        'validity': {key: {'availability': {'status': 'AVAILABLE', 'reason': None},
                           'initialized': True, 'aligned': True, 'finite': True, 'current': True}},
        'epoch_provenance': f'{RUN_ID}:{TOPIC}:quat_reset_counter=0; registered attitude window only',
    }


def replay_attitude(path, *, receiver_boot_us, max_age_us):
    """Project the registered attitude window using a caller-supplied replay time.

    The firmware defines the FRD-to-local-NED Hamilton quaternion. No position,
    UTC, inter-log frame alignment or physical receiver timestamp is inferred.
    """
    from pyulog import ULog  # Optional dependency used only by the recording adapter.

    try:
        contents = Path(path).read_bytes()
    except OSError as exc:
        raise Blocked('registered ULog unavailable') from exc
    if sha256(contents).hexdigest() != RECORDING_SHA256:
        raise Blocked('ULog SHA-256 does not match the attitude registration')
    # Decode the same bytes used in the identity check.
    with BytesIO(contents) as stream:
        # Do not filter topics: the parser associates dropout markers with the
        # last seen topic timestamp, which could shift if topics are filtered.
        log = ULog(stream)
    if (log.file_corruption or log.msg_info_dict.get('sys_name') != 'PX4'
            or log.msg_info_dict.get('ver_sw') != FIRMWARE
            or log.initial_parameters.get('EKF2_EN') != 1
            or log.initial_parameters.get('EKF2_MULTI_IMU') != 0):
        raise Blocked('uncorroborated PX4 attitude producer')
    if any(d.timestamp <= END_US and d.timestamp + d.duration * 1000 >= START_US
           for d in log.dropouts):
        raise Blocked('recorded attitude window overlaps a ULog dropout')
    data = log.get_dataset(TOPIC, multi_instance=0).data
    fields = ('timestamp', 'timestamp_sample', 'quat_reset_counter', *(f'q[{i}]' for i in range(4)))
    records = []
    last_sample = last_publication = 0
    for seq, raw_sample in enumerate(data['timestamp_sample']):
        if not START_US <= int(raw_sample) <= END_US:
            continue
        row = {field: data[field][seq].item() for field in fields}
        if not (last_sample < row['timestamp_sample'] and last_publication < row['timestamp']):
            raise Blocked('non-monotonic attitude stream')
        records.append(_adapt_attitude(row, sequence=seq, receiver_boot_us=receiver_boot_us,
                                       max_age_us=max_age_us))
        last_sample, last_publication = row['timestamp_sample'], row['timestamp']
    if not records:
        raise Blocked('registered attitude window contains no observations')
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--receiver-boot-us', type=int, required=True)
    parser.add_argument('--max-age-us', type=int, required=True)
    args = parser.parse_args()
    try:
        records = replay_attitude(args.recording, receiver_boot_us=args.receiver_boot_us,
                                  max_age_us=args.max_age_us)
    except Blocked as exc:
        parser.exit(1, f'BLOCKED: {exc}\n')
    for record in records:
        print(json.dumps(record, allow_nan=False))


if __name__ == '__main__':
    main()
