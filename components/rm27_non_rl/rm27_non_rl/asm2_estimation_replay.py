"""Sensor-only R2.2-B estimator replay and passive mode integration.

Usage: python -m rm27_sim.experiments.asm2_estimation_replay \
    --sensors recording/sensors.jsonl.gz --output replay --mode external_aided
There is deliberately no truth/model/plant argument. See the separate offline
experiment evaluator for truth comparisons. No controller is instantiated.
"""
from __future__ import annotations
import argparse
from collections import Counter
from dataclasses import asdict
from enum import Enum
import gzip
import hashlib
import json
from pathlib import Path
from time import perf_counter

import numpy as np
from rm27_non_rl.asm2_sensor_estimator import (
    SensorEstimatorConfig, SensorOnlyEstimator, time_ns,
)
from rm27_non_rl.asm2_estimation_integrity import (
    CapabilitySupervisor, ImpactEvent, Source, StateSample,
)

from .paths import ROOT, SIM_ROOT
DEFAULT_CONFIG = ROOT/'configs/asm2_state_estimation.json'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def json_value(value):
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(type(value).__name__)


def read_lines(path):
    with gzip.open(path, 'rt') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def runtime_packet(record):
    """Explicit sensor namespace: recorded raw/filter metadata never becomes state."""
    fields = ('kind', 'acquisition_s', 'delivery_s', 'value', 'source', 'valid', 'clipped')
    result = {key: record[key] for key in fields if key in record}
    # Canonicalize versioned generator field names; no physical-state inputs.
    if result['kind'] == 'external' and set(result['value']) == {'position', 'velocity'}:
        result['value'] = {'position_m': result['value']['position'],
                           'velocity_m_s': result['value']['velocity']}
    elif result['kind'] == 'impact' and set(result['value']) == {'interval_start_s', 'interval_end_s'}:
        result['value'] = {'lower_s': result['value']['interval_start_s'],
                           'upper_s': result['value']['interval_end_s']}
    return result


def supervisor_samples(estimate, config):
    """Keep local frames; this adapter cannot invent local-to-world registration."""
    fields = {'position': 'position_m', 'velocity': 'velocity_m_s',
              'attitude': 'rotation_body_to_local', 'body_rate': 'angular_velocity_body_rad_s'}
    samples = []
    for name, field in fields.items():
        channel = estimate['channels'][name]
        if channel['acquisition_s'] is None or channel['delivery_s'] is None:
            continue
        # Raw clocks stay in estimator logging; the supervisor uses the same
        # explicitly declared integer-nanosecond clock domain as the estimator.
        acquired = time_ns(channel['acquisition_s']) / 1e9
        delivered = time_ns(channel['delivery_s']) / 1e9
        represented = time_ns(channel['represented_s']) / 1e9
        samples.append(StateSample(
            quantity=name, value=estimate[field], acquisition_s=acquired,
            delivery_s=delivered, state_time_s=represented,
            epoch_id=estimate['epoch_id'], frame=channel['frame'], source=Source.SYNTHETIC,
            max_age_s=config['external_max_age_s'] if name in ('position', 'velocity') else config['imu_max_age_s'],
            valid=channel['valid'], propagated=time_ns(channel['represented_s']) != time_ns(acquired),
        ))
    return samples


def run(sensors, output, *, mode='external_aided', config_path=DEFAULT_CONFIG):
    start = perf_counter()
    if mode not in ('external_aided', 'imu_only'):
        raise ValueError('Unknown estimator mode')
    sensors, output, config_path = Path(sensors), Path(output), Path(config_path)
    config = json.loads(config_path.read_text())
    packets = [runtime_packet(row) for row in read_lines(sensors)]
    if mode == 'imu_only':
        packets = [packet for packet in packets if packet['kind'] != 'external']
    packets.sort(key=lambda p: (time_ns(p['delivery_s']), time_ns(p['acquisition_s']), p['kind']))
    if not packets:
        raise ValueError('No sensor packets')
    estimator = SensorOnlyEstimator(SensorEstimatorConfig(
        imu_gap_s=config['imu_max_age_s'], external_max_age_s=config['external_max_age_s'],
        history_s=config['history_horizon_s'],
        external_frame_registration=(
            'DECLARED_NUMERICAL_LOCAL_NAV_TO_EXTERNAL_FRAME_ASSUMPTION_NOT_OBSERVED'
            if mode == 'external_aided' else None),
        alpha_position=config['external_position_gain'], alpha_velocity=config['external_velocity_gain'],
    ))
    supervisor = CapabilitySupervisor()  # No fabricated sensor or propagation approvals.
    output.mkdir(parents=True, exist_ok=False)
    acquisition = [time_ns(p['acquisition_s']) for p in packets if p['kind'] in ('accel', 'gyro')]
    first, last = min(acquisition), max(acquisition)
    period = time_ns(config['query_period_s'])
    queries = list(range(first, last + 1, period))
    if queries[-1] != last:
        queries.append(last)
    index = 0
    rows, dispositions, mode_counts, valid_counts = [], Counter(), Counter(), Counter()
    for query in queries:
        now = query / 1e9
        while index < len(packets) and time_ns(packets[index]['delivery_s']) <= query:
            packet = packets[index]
            result = estimator.ingest(packet, now_s=now)
            dispositions[result['reason']] += 1
            if (packet['kind'] == 'impact' and result['accepted']
                    and packet.get('valid', True) and not packet.get('clipped', False)):
                value = packet['value']
                supervisor.impact(ImpactEvent(
                    event_id=f"synthetic-impact-{time_ns(packet['acquisition_s'])}",
                    lower_s=time_ns(value['lower_s'])/1e9, upper_s=time_ns(value['upper_s'])/1e9,
                    delivery_s=time_ns(packet['delivery_s'])/1e9, source=Source.SYNTHETIC, confirmed=False), now=now)
            index += 1
        estimate = estimator.estimate(now)
        decision = supervisor.evaluate(now=now, samples=supervisor_samples(estimate, config),
                                       actuator_available=False)
        mode_counts[decision.estimator_mode.value] += 1
        for channel, meta in estimate['channels'].items():
            valid_counts[channel] += int(meta['valid'])
        rows.append({'query_s': now, 'estimate': estimate,
                     'passive_supervisor': asdict(decision)})
    with (output/'estimates.jsonl.gz').open('xb') as raw:
        with gzip.GzipFile(fileobj=raw, mode='wb', mtime=0) as zipped:
            for row in rows:
                zipped.write((json.dumps(row, default=json_value, allow_nan=False)+'\n').encode())
    summary = dict(mode=mode, queries=len(rows), input_packets=len(packets),
                   packets_delivered_within_horizon=index, packet_dispositions=dict(dispositions),
                   original_rejections=estimator.rejections, valid_query_counts=dict(valid_counts),
                   passive_estimator_modes=dict(mode_counts), emits_commands=False,
                   recovery_modes_eligible=sum(bool(row['passive_supervisor']['recovery_modes']) for row in rows),
                   generation_max=max(row['estimate']['epoch_id'] for row in rows),
                   covariance=None, covariance_status='NOT_PROVIDED_BY_REUSED_COMPLEMENTARY_ESTIMATOR',
                   state_source='ACTUAL_M3_INERTIAL_PROPAGATION_AND_EXTERNAL_COMPLEMENTARY_UPDATES',
                   initialization='NO_TRUE_ATTITUDE_OR_STATIONARY_BOOTSTRAP_AVAILABLE_IN_AIRBORNE_ARCHIVE',
                   frame_assumption='Numerical nav/world identity for retrospective error diagnostics only; no observed world yaw.',
                   wall_s=perf_counter()-start,
                   passive_clock_domain='NEAREST_INTEGER_NANOSECOND; RAW_ACQUISITION_DELIVERY_PRESERVED_IN_ESTIMATE_CHANNELS; NOT_MEASURED_HARDWARE_RESOLUTION',
                   sensor_input_sha256=digest(sensors), config_sha256=digest(config_path))
    (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    sources = [Path(__file__), ROOT/'rm27_non_rl/asm2_sensor_estimator.py',
               ROOT/'rm27_non_rl/asm2_estimation_integrity.py', ROOT/'rm27_non_rl/m3_estimator.py', SIM_ROOT/'m3_sensors.py']
    (output/'provenance.json').write_text(json.dumps(dict(
        runtime_input_only=str(sensors.resolve()), truth_input=False, config=str(config_path.resolve()),
        sources_sha256={str(path.resolve()): digest(path) for path in sources},
        output_sha256={p.name:digest(p) for p in output.iterdir() if p.is_file()},
    ), indent=2)+'\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sensors', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('external_aided', 'imu_only'), default='external_aided')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    print(json.dumps(run(args.sensors, args.output, mode=args.mode, config_path=args.config), indent=2))


if __name__ == '__main__':
    main()
