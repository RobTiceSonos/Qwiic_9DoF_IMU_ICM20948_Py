import ctypes
from enum import IntEnum


# Constants
class Header_Mask(IntEnum):
    HEADER2 = 0x0008
    STEP_DETECTOR = 0x0010
    COMPASS_CALIBR = 0x0020
    GYRO_CALIBR = 0x0040
    PRESSURE = 0x0080
    GEOMAG = 0x0100
    PQUAT6 = 0x0200
    QUAT9 = 0x0400
    QUAT6 = 0x0800
    ALS = 0x1000
    COMPASS = 0x2000
    GYRO = 0x4000
    ACCEL = 0x8000


class Header2_Mask(IntEnum):
    SECONDARY_ON_OFF = 0x0040
    ACTIVITY_RECOG = 0x0080
    PICKUP = 0x0400
    FSYNC = 0x0800
    COMPASS_ACCURACY = 0x1000
    GYRO_ACCURACY = 0x2000
    ACCEL_ACCURACY = 0x4000


class BaseStruct(ctypes.Structure):
    _pack_ = 1

    def asdict(self):
        result = {}
        for field, _ in self._fields_:
            value = getattr(self, field)
            # if the type is not a primitive and it evaluates to False ...
            if (type(value) not in [int, float, bool]) and not bool(value):
                # it's a null pointer
                value = None
            elif hasattr(value, "_length_") and hasattr(value, "_type_"):
                # Probably an array
                value = list(value)
            elif hasattr(value, "_fields_"):
                # Probably another struct
                value = self.asdict(value)
            result[field] = value
        return result


class RawAccel(BaseStruct):
    mask = Header_Mask.ACCEL
    layout = '!hhh'
    _fields_ = [
        ('X', ctypes.c_int16),
        ('Y', ctypes.c_int16),
        ('Z', ctypes.c_int16),
    ]


class RawGyro(BaseStruct):
    mask = Header_Mask.GYRO
    layout = '!hhhhhh'
    _fields_ = [
        ('X', ctypes.c_int16),
        ('Y', ctypes.c_int16),
        ('Z', ctypes.c_int16),
        ('BiasX', ctypes.c_int16),
        ('BiasY', ctypes.c_int16),
        ('BiasZ', ctypes.c_int16),
    ]


class Compass(BaseStruct):
    mask = Header_Mask.COMPASS
    layout = '!hhh'
    _fields_ = [
        ('X', ctypes.c_int16),
        ('Y', ctypes.c_int16),
        ('Z', ctypes.c_int16),
    ]


class ALS(BaseStruct):
    mask = Header_Mask.ALS
    layout = '!xHHHx'
    _fields_ = [
        ('Ch0DATA', ctypes.c_uint16),
        ('Ch1DATA', ctypes.c_uint16),
        ('PDATA', ctypes.c_uint16),
    ]

#     # The 6-Axis and 9-axis Quaternion outputs each consist of 12 bytes of data.
#     # These 12 bytes in turn consists of three 4-byte elements.
#     # 9-axis quaternion data and Geomag rv is always followed by 2-bytes of heading accuracy, hence the size of Quat9 and Geomag data size in the FIFO is 14 bytes.
#     # Quaternion data for both cases is cumulative/integrated values.
#     # For a given quaternion Q, the ordering of its elements is {Q1, Q2, Q3}.
#     # Each element is represented using Big Endian byte order.
#     # Q0 value is computed from this equation: Q20 + Q21 + Q22 + Q23 = 1.
#     # In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
#     # The quaternion data is scaled by 2^30.


class Quat6(BaseStruct):
    mask = Header_Mask.QUAT6
    layout = '!iii'
    _fields_ = [
        ('Q1', ctypes.c_int32),
        ('Q2', ctypes.c_int32),
        ('Q3', ctypes.c_int32),
    ]

    def asdict(self):
        div = 1073741824.0
        return {
            'Q1': self.Q1 / div,
            'Q2': self.Q2 / div,
            'Q3': self.Q3 / div,
        }


class Quat9(Quat6):
    mask = Header_Mask.QUAT9
    layout = '!iiih'
    _fields_ = [
        # Q1, Q2, Q3 inherited from Quat6
        ('Accuracy', ctypes.c_int16),
    ]

    def asdict(self):
        ret = super().asdict()
        ret['Accuracy'] = self.Accuracy
        return ret


class PQuat6(BaseStruct):
    mask = Header_Mask.PQUAT6
    layout = '!hhh'
    _fields_ = [
        ('Q1', ctypes.c_int16),
        ('Q2', ctypes.c_int16),
        ('Q3', ctypes.c_int16),
    ]


class Geomag(Quat9):
    mask = Header_Mask.GEOMAG
    layout = '!iiih'


class Pressure(BaseStruct):
    mask = Header_Mask.PRESSURE
    layout = '!3s3s'
    _fields_ = [
        ('Pressure', ctypes.c_uint8 * 3),
        ('Temperature', ctypes.c_uint8 * 3),
    ]


class Gyro_Calibr(BaseStruct):
    mask = Header_Mask.GYRO_CALIBR
    layout = '!iii'
    _fields_ = [
        ('X', ctypes.c_int32), # Hardware unit scaled by 2^15
        ('Y', ctypes.c_int32),
        ('Z', ctypes.c_int32),
    ]


class Compass_Calibr(BaseStruct):
    mask = Header_Mask.COMPASS_CALIBR
    layout = '!iii'
    _fields_ = [
        ('X', ctypes.c_int32), # The unit is uT scaled by 2^16
        ('Y', ctypes.c_int32),
        ('Z', ctypes.c_int32),
    ]


class Pedometer_Timestamp(BaseStruct):
    mask = Header_Mask.STEP_DETECTOR
    layout = '!I'
    _fields_ = [
        ('Timestamp', ctypes.c_uint32), # Timestamp as DMP cycle
    ]


class Accel_Accuracy(BaseStruct):
    mask = Header2_Mask.ACCEL_ACCURACY
    layout = '!H'
    _fields_ = [
        ('Accuracy', ctypes.c_uint16), # The accuracy is expressed as 0~3. The lowest is 0 and 3 is the highest.
    ]


class Gyro_Accuracy(BaseStruct):
    mask = Header2_Mask.GYRO_ACCURACY
    layout = '!H'
    _fields_ = [
        ('Accuracy', ctypes.c_uint16), # The accuracy is expressed as 0~3. The lowest is 0 and 3 is the highest.
    ]


class Compass_Accuracy(BaseStruct):
    mask = Header2_Mask.COMPASS_ACCURACY
    layout = '!H'
    _fields_ = [
        ('Accuracy', ctypes.c_uint16), # The accuracy is expressed as 0~3. The lowest is 0 and 3 is the highest.
    ]


class Fsync_Delay_Time(BaseStruct):
    mask = Header2_Mask.FSYNC
    layout = '!H'
    _fields_ = [
        ('Value', ctypes.c_uint16), # The data is delay time between Fsync event and the 1st ODR event after Fsync event.
    ]


class Pickup(BaseStruct):
    mask = Header2_Mask.PICKUP
    layout = '!H'
    _fields_ = [
        ('Value', ctypes.c_uint16), # The value “2” indicates pick up is detected.
    ]

#     # Activity Recognition data
#     # The data include Start and End states, and timestamp as DMP cycle.
#     # Byte [0]: State-Start, Byte [1]: State-End, Byte [5:2]: timestamp.
#     # The states are expressed as below.
#     # Drive: 0x01
#     # Walk: 0x02
#     # Run: 0x04
#     # Bike: 0x08
#     # Tilt: 0x10
#     # Still: 0x20


class Activity_Recognition(BaseStruct):
    mask = Header2_Mask.ACTIVITY_RECOG
    layout = '!BBI'
    _fields_ = [
        ('State_Start', ctypes.c_uint8),
        ('State_End', ctypes.c_uint8),
        ('Timestamp', ctypes.c_uint32),
    ]

#     # Secondary On/Off data
#     # BAC algorithm requires sensors on/off through FIFO data to detect activities effectively and save power.
#     # The driver is expected to control sensors accordingly.
#     # The data indicates which sensor and on or off as below.
#     # Gyro Off: 0x01
#     # Gyro On: 0x02
#     # Compass Off: 0x04
#     # Compass On: 0x08
#     # Proximity Off: 0x10
#     # Proximity On: 0x20


class Secondary_On_Off(BaseStruct):
    mask = Header2_Mask.SECONDARY_ON_OFF
    layout = '!H'
    _fields_ = [
        ('Sensors', ctypes.c_uint16),
    ]


class Footer(BaseStruct):
    layout = '!H'
    _fields_ = [
        ('Value', ctypes.c_uint16),
    ]


HEADER_SENSORS = [
    RawAccel,
    RawGyro,
    Compass,
    # ALS,
    Quat6,
    Quat9,
    PQuat6,
    Geomag,
    # Pressure,
    Gyro_Calibr,
    Compass_Calibr,
    Pedometer_Timestamp,
]

HEADER2_SENSORS = [
    Accel_Accuracy,
    Gyro_Accuracy,
    Compass_Accuracy,
    # Fsync_Delay_Time,
    Pickup,
    # Activity_Recognition,
    Secondary_On_Off,
]
