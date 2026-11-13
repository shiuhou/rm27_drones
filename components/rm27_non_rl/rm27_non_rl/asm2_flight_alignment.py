"""Sensor-only inertial/external-velocity alignment from an excited flight history."""
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .asm2_sensor_estimator import SensorPacket


@dataclass(frozen=True)
class FlightAlignment:
    acquisition_s: float
    delivery_s: float
    rotation: np.ndarray
    external: SensorPacket
    residual_m_s: float
    excitation_ratio: float
    intervals: int


def solve_alignment(records, *, now_s, minimum_history_s=.65,
                    minimum_excitation_ratio=.001, maximum_residual_m_s=.06):
    packets = [SensorPacket.from_mapping(record) for record in records
               if record['delivery_s'] <= now_s and record['kind'] != 'impact']
    if any(not packet.valid or packet.clipped for packet in packets):
        return None
    external = sorted([packet for packet in packets if packet.kind == 'external'],
                      key=lambda packet: packet.acquisition_s)
    if len(external) < 8 or external[-1].acquisition_s-external[0].acquisition_s < minimum_history_s:
        return None
    timeline = sorted(packets, key=lambda packet: (packet.acquisition_s, packet.kind))
    rotation, impulse = np.eye(3), np.zeros(3)
    gyro, force = None, None
    gyro_time, force_time = None, None
    previous = timeline[0].acquisition_s
    observations = []
    for packet in timeline:
        duration = packet.acquisition_s-previous
        if duration > .01:
            return None
        if ((gyro_time is not None and packet.acquisition_s-gyro_time > .01)
                or (force_time is not None and packet.acquisition_s-force_time > .01)):
            return None
        if gyro is not None and force is not None:
            impulse += rotation @ force * duration
            rotation = rotation @ Rotation.from_rotvec(gyro*duration).as_matrix()
        previous = packet.acquisition_s
        if packet.kind == 'gyro':
            gyro = np.asarray(packet.value)
            gyro_time = packet.acquisition_s
        elif packet.kind == 'accel':
            force = np.asarray(packet.value)
            force_time = packet.acquisition_s
        elif packet.kind == 'external':
            observations.append((packet, impulse.copy(), rotation.copy()))
    local = np.asarray([observation[1] for observation in observations])
    world = np.asarray([np.asarray(observation[0].value[1])+[0., 0., 9.81*observation[0].acquisition_s]
                        for observation in observations])
    local -= local.mean(axis=0)
    world -= world.mean(axis=0)
    left, singular, right = np.linalg.svd(world.T @ local)
    excitation = singular[1]/singular[0]
    registration = left @ np.diag([1., 1., np.linalg.det(left @ right)]) @ right
    residual = float(np.sqrt(np.mean(np.sum((local @ registration.T-world)**2, axis=1))))
    if excitation < minimum_excitation_ratio or residual > maximum_residual_m_s:
        return None
    latest, _, orientation = observations[-1]
    return FlightAlignment(latest.acquisition_s, now_s, registration @ orientation,
                           latest, residual, float(excitation), len(local))
