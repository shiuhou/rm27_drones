"""Future raw-IMU evidence validation only. No VIO estimator or synchronization."""
from ..schema import TimestampEvidence
from .common import require, number


def validate_imu_sample(sample):
    require(sample.get('schema_version') == 1 and type(sample.get('schema_version')) is int,
            'IMU_SCHEMA_INVALID','version 1')
    require(type(sample.get('sequence')) is int and sample['sequence']>=0,'IMU_SEQUENCE_INVALID','sequence')
    TimestampEvidence.from_dict(sample['timestamp'])
    require(sample.get('measurement_kind')=='RAW_IMU','IMU_KIND_INVALID','attitude telemetry is not raw gyro/accelerometer data')
    require(sample.get('gyro_unit')=='rad/s' and sample.get('accel_unit')=='m/s^2', 'IMU_UNITS_INVALID','explicit SI units required')
    require(isinstance(sample.get('device_id'),str) and sample['device_id'] not in ('','UNKNOWN'), 'IMU_DEVICE_MISSING','identity')
    for key in ('gyro_xyz','accel_xyz'):
        vector=sample.get(key)
        require(isinstance(vector,list) and len(vector)==3,'IMU_VECTOR_INVALID',key)
        for x in vector:
            number(x,key)
    if sample.get('saturation') is not None:
        require(type(sample['saturation']) is bool,'IMU_SATURATION_INVALID','boolean or absent/null')
    return sample
