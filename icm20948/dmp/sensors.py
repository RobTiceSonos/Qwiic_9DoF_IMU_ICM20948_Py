from enum import IntEnum


class SensorNotSupported(Exception):
    """ Raised when a sensor is requested in the DMP that is not supported """
    pass


# Sensor identifier for control function
class Sensor_Types(IntEnum):
    ACCELEROMETER = 0,
    GYROSCOPE = 1
    RAW_ACCELEROMETER = 2
    RAW_GYROSCOPE = 3
    MAGNETIC_FIELD_UNCALIBRATED = 4
    GYROSCOPE_UNCALIBRATED = 5
    ACTIVITY_CLASSIFICATON = 6
    STEP_DETECTOR = 7
    STEP_COUNTER = 8
    GAME_ROTATION_VECTOR = 9
    ROTATION_VECTOR = 10
    GEOMAGNETIC_ROTATION_VECTOR = 11
    GEOMAGNETIC_FIELD = 12
    WAKEUP_SIGNIFICANT_MOTION = 13
    FLIP_PICKUP = 14
    WAKEUP_TILT_DETECTOR = 15
    GRAVITY = 16
    LINEAR_ACCELERATION = 17
    ORIENTATION = 18
    B2S = 19
    RAW_MAGNETOMETER = 20
    MAX = 21


# enum for android sensor
class Android_Sensors(IntEnum):
    META_DATA = 0
    ACCELEROMETER = 1
    GEOMAGNETIC_FIELD = 2
    ORIENTATION = 3
    GYROSCOPE = 4
    LIGHT = 5
    PRESSURE = 6
    TEMPERATURE = 7
    WAKEUP_PROXIMITY = 8
    GRAVITY = 9
    LINEAR_ACCELERATION = 10
    ROTATION_VECTOR = 11
    HUMIDITY = 12
    AMBIENT_TEMPERATURE = 13
    MAGNETIC_FIELD_UNCALIBRATED = 14
    GAME_ROTATION_VECTOR = 15
    GYROSCOPE_UNCALIBRATED = 16
    WAKEUP_SIGNIFICANT_MOTION = 17
    STEP_DETECTOR = 18
    STEP_COUNTER = 19
    GEOMAGNETIC_ROTATION_VECTOR = 20
    HEART_RATE = 21
    PROXIMITY = 22

    WAKEUP_ACCELEROMETER = 23
    WAKEUP_MAGNETIC_FIELD = 24
    WAKEUP_ORIENTATION = 25
    WAKEUP_GYROSCOPE = 26
    WAKEUP_LIGHT = 27
    WAKEUP_PRESSURE = 28
    WAKEUP_GRAVITY = 29
    WAKEUP_LINEAR_ACCELERATION = 30
    WAKEUP_ROTATION_VECTOR = 31
    WAKEUP_RELATIVE_HUMIDITY = 32
    WAKEUP_AMBIENT_TEMPERATURE = 33
    WAKEUP_MAGNETIC_FIELD_UNCALIBRATED = 34
    WAKEUP_GAME_ROTATION_VECTOR = 35
    WAKEUP_GYROSCOPE_UNCALIBRATED = 36
    WAKEUP_STEP_DETECTOR = 37
    WAKEUP_STEP_COUNTER = 38
    WAKEUP_GEOMAGNETIC_ROTATION_VECTOR = 39
    WAKEUP_HEART_RATE = 40
    WAKEUP_TILT_DETECTOR = 41

    RAW_ACCELEROMETER = 42
    RAW_GYROSCOPE = 43
    NUM_MAX = 44
    B2S = 45
    FLIP_PICKUP = 46
    ACTIVITY_CLASSIFICATON = 47
    SCREEN_ROTATION = 48
    SELF_TEST = 49
    SETUP = 50
    GENERAL_SENSORS_MAX = 51


def sensor_type_2_android_sensor(sensor: Sensor_Types):
    lookup = {
        Sensor_Types.ACCELEROMETER: Android_Sensors.ACCELEROMETER,
        Sensor_Types.GYROSCOPE: Android_Sensors.GYROSCOPE,
        Sensor_Types.RAW_ACCELEROMETER: Android_Sensors.RAW_ACCELEROMETER,
        Sensor_Types.RAW_GYROSCOPE: Android_Sensors.RAW_GYROSCOPE,
        Sensor_Types.MAGNETIC_FIELD_UNCALIBRATED: Android_Sensors.MAGNETIC_FIELD_UNCALIBRATED,
        Sensor_Types.GYROSCOPE_UNCALIBRATED: Android_Sensors.GYROSCOPE_UNCALIBRATED,
        Sensor_Types.ACTIVITY_CLASSIFICATON: Android_Sensors.ACTIVITY_CLASSIFICATON,
        Sensor_Types.STEP_DETECTOR: Android_Sensors.STEP_DETECTOR,
        Sensor_Types.STEP_COUNTER: Android_Sensors.STEP_COUNTER,
        Sensor_Types.GAME_ROTATION_VECTOR: Android_Sensors.GAME_ROTATION_VECTOR,
        Sensor_Types.ROTATION_VECTOR: Android_Sensors.ROTATION_VECTOR,
        Sensor_Types.GEOMAGNETIC_ROTATION_VECTOR: Android_Sensors.GEOMAGNETIC_ROTATION_VECTOR,
        Sensor_Types.GEOMAGNETIC_FIELD: Android_Sensors.GEOMAGNETIC_FIELD,
        Sensor_Types.WAKEUP_SIGNIFICANT_MOTION: Android_Sensors.WAKEUP_SIGNIFICANT_MOTION,
        Sensor_Types.FLIP_PICKUP: Android_Sensors.FLIP_PICKUP,
        Sensor_Types.WAKEUP_TILT_DETECTOR: Android_Sensors.WAKEUP_TILT_DETECTOR,
        Sensor_Types.GRAVITY: Android_Sensors.GRAVITY,
        Sensor_Types.LINEAR_ACCELERATION: Android_Sensors.LINEAR_ACCELERATION,
        Sensor_Types.ORIENTATION: Android_Sensors.ORIENTATION,
        Sensor_Types.B2S: Android_Sensors.B2S,
    }

    if sensor in lookup:
        return lookup[sensor]

    raise SensorNotSupported('Sensor type not supported.')


def androidSensor_to_control_bits(sensor):
    # Data output control 1 register bit definition
    # 16-bit accel                                0x8000
    # 16-bit gyro                                 0x4000
    # 16-bit compass                              0x2000
    # 16-bit ALS                                  0x1000
    # 32-bit 6-axis quaternion                    0x0800
    # 32-bit 9-axis quaternion + heading accuracy 0x0400
    # 16-bit pedometer quaternion                 0x0200
    # 32-bit Geomag rv + heading accuracy         0x0100
    # 16-bit Pressure                             0x0080
    # 32-bit calibrated gyro                      0x0040
    # 32-bit calibrated compass                   0x0020
    # Pedometer Step Detector                     0x0010
    # Header 2                                    0x0008
    # Pedometer Step Indicator Bit 2              0x0004
    # Pedometer Step Indicator Bit 1              0x0002
    # Pedometer Step Indicator Bit 0              0x0001
    # Unsupported Sensors are 0xFFFF
    lookup = {
        Android_Sensors.META_DATA: 0xFFFF,
        Android_Sensors.ACCELEROMETER: 0x8008,
        Android_Sensors.GEOMAGNETIC_FIELD: 0x0028,
        Android_Sensors.ORIENTATION: 0x0408,
        Android_Sensors.GYROSCOPE: 0x4048,
        Android_Sensors.LIGHT: 0x1008,
        Android_Sensors.PRESSURE: 0x0088,
        Android_Sensors.TEMPERATURE: 0xFFFF,
        Android_Sensors.WAKEUP_PROXIMITY: 0xFFFF,
        Android_Sensors.GRAVITY: 0x0808,
        Android_Sensors.LINEAR_ACCELERATION: 0x8808,
        Android_Sensors.ROTATION_VECTOR: 0x0408,
        Android_Sensors.HUMIDITY: 0xFFFF,
        Android_Sensors.AMBIENT_TEMPERATURE: 0xFFFF,
        Android_Sensors.MAGNETIC_FIELD_UNCALIBRATED: 0x2008,
        Android_Sensors.GAME_ROTATION_VECTOR: 0x0808,
        Android_Sensors.GYROSCOPE_UNCALIBRATED: 0x4008,
        Android_Sensors.WAKEUP_SIGNIFICANT_MOTION: 0x0000,
        Android_Sensors.STEP_DETECTOR: 0x0018,
        Android_Sensors.STEP_COUNTER: 0x0010,
        Android_Sensors.GEOMAGNETIC_ROTATION_VECTOR: 0x0108,
        Android_Sensors.HEART_RATE: 0xFFFF,
        Android_Sensors.PROXIMITY: 0xFFFF,
        Android_Sensors.WAKEUP_ACCELEROMETER: 0x8008,
        Android_Sensors.WAKEUP_MAGNETIC_FIELD: 0x0028,
        Android_Sensors.WAKEUP_ORIENTATION: 0x0408,
        Android_Sensors.WAKEUP_GYROSCOPE: 0x4048,
        Android_Sensors.WAKEUP_LIGHT: 0x1008,
        Android_Sensors.WAKEUP_PRESSURE: 0x0088,
        Android_Sensors.WAKEUP_GRAVITY: 0x0808,
        Android_Sensors.WAKEUP_LINEAR_ACCELERATION: 0x8808,
        Android_Sensors.WAKEUP_ROTATION_VECTOR: 0x0408,
        Android_Sensors.WAKEUP_RELATIVE_HUMIDITY: 0xFFFF,
        Android_Sensors.WAKEUP_AMBIENT_TEMPERATURE: 0xFFFF,
        Android_Sensors.WAKEUP_MAGNETIC_FIELD_UNCALIBRATED: 0x2008,
        Android_Sensors.WAKEUP_GAME_ROTATION_VECTOR: 0x0808,
        Android_Sensors.WAKEUP_GYROSCOPE_UNCALIBRATED: 0x4008,
        Android_Sensors.WAKEUP_STEP_DETECTOR: 0x0018,
        Android_Sensors.WAKEUP_STEP_COUNTER: 0x0010,
        Android_Sensors.WAKEUP_GEOMAGNETIC_ROTATION_VECTOR: 0x0108,
        Android_Sensors.WAKEUP_HEART_RATE: 0xFFFF,
        Android_Sensors.WAKEUP_TILT_DETECTOR: 0x0000,
        Android_Sensors.RAW_ACCELEROMETER: 0x8008,
        Android_Sensors.RAW_GYROSCOPE: 0x4048,
    }

    if sensor in lookup and lookup[sensor] != 0xFFFF:
        return lookup[sensor]

    raise SensorNotSupported('Sensor type not supported.')


# Determines which base sensor needs to be on
NEEDS_ACCEL_MASK = [
    Android_Sensors.ACCELEROMETER,
    Android_Sensors.ORIENTATION,
    Android_Sensors.GRAVITY,
    Android_Sensors.LINEAR_ACCELERATION,
    Android_Sensors.ROTATION_VECTOR,
    Android_Sensors.GAME_ROTATION_VECTOR,
    Android_Sensors.WAKEUP_SIGNIFICANT_MOTION,
    Android_Sensors.STEP_DETECTOR,
    Android_Sensors.STEP_COUNTER,
    Android_Sensors.GEOMAGNETIC_ROTATION_VECTOR,
    Android_Sensors.WAKEUP_ACCELEROMETER,
    Android_Sensors.WAKEUP_ORIENTATION,
    Android_Sensors.WAKEUP_GRAVITY,
    Android_Sensors.WAKEUP_LINEAR_ACCELERATION,
    Android_Sensors.WAKEUP_ROTATION_VECTOR,

    Android_Sensors.WAKEUP_GAME_ROTATION_VECTOR,
    Android_Sensors.WAKEUP_STEP_DETECTOR,
    Android_Sensors.WAKEUP_STEP_COUNTER,
    Android_Sensors.WAKEUP_GEOMAGNETIC_ROTATION_VECTOR,
    Android_Sensors.WAKEUP_TILT_DETECTOR,
    Android_Sensors.RAW_ACCELEROMETER,
]

NEEDS_GYRO_MASK = [
    Android_Sensors.ORIENTATION,
    Android_Sensors.GYROSCOPE,
    Android_Sensors.GRAVITY,
    Android_Sensors.LINEAR_ACCELERATION,
    Android_Sensors.ROTATION_VECTOR,
    Android_Sensors.GAME_ROTATION_VECTOR,
    Android_Sensors.GYROSCOPE_UNCALIBRATED,
    Android_Sensors.WAKEUP_ORIENTATION,
    Android_Sensors.WAKEUP_GYROSCOPE,
    Android_Sensors.WAKEUP_GRAVITY,
    Android_Sensors.WAKEUP_LINEAR_ACCELERATION,
    Android_Sensors.WAKEUP_ROTATION_VECTOR,

    Android_Sensors.WAKEUP_GAME_ROTATION_VECTOR,
    Android_Sensors.WAKEUP_GYROSCOPE_UNCALIBRATED,
    Android_Sensors.RAW_GYROSCOPE,
]

NEEDS_COMPASS_MASK = [
    Android_Sensors.GEOMAGNETIC_FIELD,
    Android_Sensors.ORIENTATION,
    Android_Sensors.ROTATION_VECTOR,
    Android_Sensors.MAGNETIC_FIELD_UNCALIBRATED,
    Android_Sensors.GEOMAGNETIC_ROTATION_VECTOR,
    Android_Sensors.WAKEUP_MAGNETIC_FIELD,
    Android_Sensors.WAKEUP_ORIENTATION,
    Android_Sensors.WAKEUP_ROTATION_VECTOR,

    Android_Sensors.WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
    Android_Sensors.WAKEUP_GEOMAGNETIC_ROTATION_VECTOR,
]

NEEDS_PRESSURE = [
    Android_Sensors.PRESSURE,
    Android_Sensors.WAKEUP_PRESSURE,
]
