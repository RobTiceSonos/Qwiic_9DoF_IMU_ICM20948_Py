from enum import IntEnum


# define custom exceptions
class DMPFIFOUnderflow(Exception):
    """ Raised when the DMP FIFO does not contain enough data yet """
    pass


class DMPUninitialized(Exception):
    """ Raised when a DMP method is used prior to initialization """
    pass


class DMPInitFailure(Exception):
    """ Raised when an error occurs in DMP initialization """
    pass


DMP_START_ADDRESS = 0x1000
DMP_MEM_BANK_SIZE = 256
DMP_LOAD_START = 0x90

CFG_FIFO_SIZE = (4222)

# AGB0_REG_DMP_INT_STATUS bit definitions
BIT_WAKE_ON_MOTION_INT = 0x08
BIT_MSG_DMP_INT = 0x0002
BIT_MSG_DMP_INT_0 = 0x0100 # CI Command

BIT_MSG_DMP_INT_2 = 0x0200 # CIM Command - SMD
BIT_MSG_DMP_INT_3 = 0x0400 # CIM Command - Pedometer

BIT_MSG_DMP_INT_4 = 0x1000 # CIM Command - Pedometer binning
BIT_MSG_DMP_INT_5 = 0x2000 # CIM Command - Bring To See Gesture
BIT_MSG_DMP_INT_6 = 0x4000 # CIM Command - Look To See Gesture

# Appendix I: DMP register addresses

# data output control
DATA_OUT_CTL1 = (4 * 16) # 16-bit: Data output control 1 register : configure DMP to output required data
DATA_OUT_CTL2 = (4 * 16 + 2) # 16-bit: Data output control 2 register : configure the BM, accel/gyro/compass accuracy and gesture such as Pick-up
DATA_INTR_CTL = (4 * 16 + 12) # 16-bit: Determines which sensors can generate interrupt according to bit map defined for DATA_OUT_CTL1
FIFO_WATERMARK = (31 * 16 + 14) # 16-bit: DMP will send FIFO interrupt if FIFO count > FIFO watermark. FIFO watermark is set to 80% of actual FIFO size by default

# motion event control
MOTION_EVENT_CTL = (4 * 16 + 14) # 16-bit: configure DMP for Android L and Invensense specific features

# indicates to DMP which sensors are available
#	1: gyro samples available
#	2: accel samples available
#	8: secondary compass samples available
DATA_RDY_STATUS = (8 * 16 + 10) # 16-bit: indicates to DMP which sensors are available

# batch mode
BM_BATCH_CNTR = (27 * 16) # 32-bit: Batch counter
BM_BATCH_THLD = (19 * 16 + 12) # 32-bit: Batch mode threshold
BM_BATCH_MASK = (21 * 16 + 14) # 16-bit

# sensor output data rate: all 16-bit
ODR_ACCEL = (11 * 16 + 14) # ODR_ACCEL Register for accel ODR
ODR_GYRO = (11 * 16 + 10) # ODR_GYRO Register for gyro ODR
ODR_CPASS = (11 * 16 + 6) # ODR_CPASS Register for compass ODR
ODR_ALS = (11 * 16 + 2) # ODR_ALS Register for ALS ODR
ODR_QUAT6 = (10 * 16 + 12) # ODR_QUAT6 Register for 6-axis quaternion ODR
ODR_QUAT9 = (10 * 16 + 8) # ODR_QUAT9 Register for 9-axis quaternion ODR
ODR_PQUAT6 = (10 * 16 + 4) # ODR_PQUAT6 Register for 6-axis pedometer quaternion ODR
ODR_GEOMAG = (10 * 16 + 0) # ODR_GEOMAG Register for Geomag rv ODR
ODR_PRESSURE = (11 * 16 + 12) # ODR_PRESSURE Register for pressure ODR
ODR_GYRO_CALIBR = (11 * 16 + 8) # ODR_GYRO_CALIBR Register for calibrated gyro ODR
ODR_CPASS_CALIBR = (11 * 16 + 4) # ODR_CPASS_CALIBR Register for calibrated compass ODR

# sensor output data rate counter: all 16-bit
ODR_CNTR_ACCEL = (9 * 16 + 14) # ODR_CNTR_ACCEL Register for accel ODR counter
ODR_CNTR_GYRO = (9 * 16 + 10) # ODR_CNTR_GYRO Register for gyro ODR counter
ODR_CNTR_CPASS = (9 * 16 + 6) # ODR_CNTR_CPASS Register for compass ODR counter
ODR_CNTR_ALS = (9 * 16 + 2) # ODR_CNTR_ALS Register for ALS ODR counter
ODR_CNTR_QUAT6 = (8 * 16 + 12) # ODR_CNTR_QUAT6 Register for 6-axis quaternion ODR counter
ODR_CNTR_QUAT9 = (8 * 16 + 8) # ODR_CNTR_QUAT9 Register for 9-axis quaternion ODR counter
ODR_CNTR_PQUAT6 = (8 * 16 + 4) # ODR_CNTR_PQUAT6 Register for 6-axis pedometer quaternion ODR counter
ODR_CNTR_GEOMAG = (8 * 16 + 0) # ODR_CNTR_GEOMAG Register for Geomag rv ODR counter
ODR_CNTR_PRESSURE = (9 * 16 + 12) # ODR_CNTR_PRESSURE Register for pressure ODR counter
ODR_CNTR_GYRO_CALIBR = (9 * 16 + 8) # ODR_CNTR_GYRO_CALIBR Register for calibrated gyro ODR counter
ODR_CNTR_CPASS_CALIBR = (9 * 16 + 4) # ODR_CNTR_CPASS_CALIBR Register for calibrated compass ODR counter

# mounting matrix: all 32-bit
CPASS_MTX_00 = (23 * 16) # Compass mount matrix and scale
CPASS_MTX_01 = (23 * 16 + 4) # Compass mount matrix and scale
CPASS_MTX_02 = (23 * 16 + 8) # Compass mount matrix and scale
CPASS_MTX_10 = (23 * 16 + 12) # Compass mount matrix and scale
CPASS_MTX_11 = (24 * 16) # Compass mount matrix and scale
CPASS_MTX_12 = (24 * 16 + 4) # Compass mount matrix and scale
CPASS_MTX_20 = (24 * 16 + 8) # Compass mount matrix and scale
CPASS_MTX_21 = (24 * 16 + 12) # Compass mount matrix and scale
CPASS_MTX_22 = (25 * 16) # Compass mount matrix and scale

# bias calibration: all 32-bit
# The biases are 32-bits in chip frame in hardware unit scaled by:
# 2^12 (FSR 4g) for accel, 2^15 for gyro, in uT scaled by 2^16 for compass.
GYRO_BIAS_X = (139 * 16 + 4)
GYRO_BIAS_Y = (139 * 16 + 8)
GYRO_BIAS_Z = (139 * 16 + 12)
ACCEL_BIAS_X = (110 * 16 + 4)
ACCEL_BIAS_Y = (110 * 16 + 8)
ACCEL_BIAS_Z = (110 * 16 + 12)
CPASS_BIAS_X = (126 * 16 + 4)
CPASS_BIAS_Y = (126 * 16 + 8)
CPASS_BIAS_Z = (126 * 16 + 12)

GYRO_ACCURACY = (138 * 16 + 2)
GYRO_BIAS_SET = (138 * 16 + 6)
GYRO_LAST_TEMPR = (134 * 16)
GYRO_SLOPE_X = (78 * 16 + 4)
GYRO_SLOPE_Y = (78 * 16 + 8)
GYRO_SLOPE_Z = (78 * 16 + 12)

# parameters for accel calibration
ACCEL_ACCURACY = (97 * 16)
ACCEL_CAL_RESET = (77 * 16)
ACCEL_VARIANCE_THRESH = (93 * 16)
ACCEL_CAL_RATE = (94 * 16 + 4) # 16-bit: 0 (225Hz, 112Hz, 56Hz)
ACCEL_PRE_SENSOR_DATA = (97 * 16 + 4)
ACCEL_COVARIANCE = (101 * 16 + 8)
ACCEL_ALPHA_VAR = (91 * 16) # 32-bit: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
ACCEL_A_VAR = (92 * 16) # 32-bit: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
ACCEL_CAL_INIT = (94 * 16 + 2)
ACCEL_CAL_SCALE_COVQ_IN_RANGE = (194 * 16)
ACCEL_CAL_SCALE_COVQ_OUT_RANGE = (195 * 16)
ACCEL_CAL_TEMPERATURE_SENSITIVITY = (194 * 16 + 4)
ACCEL_CAL_TEMPERATURE_OFFSET_TRIM = (194 * 16 + 12)

CPASS_ACCURACY = (37 * 16)
CPASS_BIAS_SET = (34 * 16 + 14)
MAR_MODE = (37 * 16 + 2)
CPASS_COVARIANCE = (115 * 16)
CPASS_COVARIANCE_CUR = (118 * 16 + 8)
CPASS_REF_MAG_3D = (122 * 16)
CPASS_CAL_INIT = (114 * 16)
CPASS_EST_FIRST_BIAS = (113 * 16)
MAG_DISTURB_STATE = (113 * 16 + 2)
CPASS_VAR_COUNT = (112 * 16 + 6)
CPASS_COUNT_7 = (87 * 16 + 2)
CPASS_MAX_INNO = (124 * 16)
CPASS_BIAS_OFFSET = (113 * 16 + 4)
CPASS_CUR_BIAS_OFFSET = (114 * 16 + 4)
CPASS_PRE_SENSOR_DATA = (87 * 16 + 4)

# Compass Cal params to be adjusted according to sampling rate
CPASS_TIME_BUFFER = (112 * 16 + 14)
CPASS_RADIUS_3D_THRESH_ANOMALY = (112 * 16 + 8)

CPASS_STATUS_CHK = (25 * 16 + 12)

# gains
ACCEL_FB_GAIN = (34 * 16)
ACCEL_ONLY_GAIN = (16 * 16 + 12) # 32-bit: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
GYRO_SF = (19 * 16) # 32-bit: gyro scaling factor

# 9-axis
MAGN_THR_9X = (80 * 16)
MAGN_LPF_THR_9X = (80 * 16 + 8)
QFB_THR_9X = (80 * 16 + 12)

# DMP running counter
DMPRATE_CNTR = (18 * 16 + 4)

# pedometer
PEDSTD_BP_B = (49 * 16 + 12)
PEDSTD_BP_A4 = (52 * 16)
PEDSTD_BP_A3 = (52 * 16 + 4)
PEDSTD_BP_A2 = (52 * 16 + 8)
PEDSTD_BP_A1 = (52 * 16 + 12)
PEDSTD_SB = (50 * 16 + 8)
PEDSTD_SB_TIME = (50 * 16 + 12)
PEDSTD_PEAKTHRSH = (57 * 16 + 8)
PEDSTD_TIML = (50 * 16 + 10)
PEDSTD_TIMH = (50 * 16 + 14)
PEDSTD_PEAK = (57 * 16 + 4)
PEDSTD_STEPCTR = (54 * 16)
PEDSTD_STEPCTR2 = (58 * 16 + 8)
PEDSTD_TIMECTR = (60 * 16 + 4)
PEDSTD_DECI = (58 * 16)
PEDSTD_SB2 = (60 * 16 + 14)
STPDET_TIMESTAMP = (18 * 16 + 8)
PEDSTEP_IND = (19 * 16 + 4)
PED_Y_RATIO = (17 * 16 + 0)

# SMD
SMD_VAR_TH = (141 * 16 + 12)
SMD_VAR_TH_DRIVE = (143 * 16 + 12)
SMD_DRIVE_TIMER_TH = (143 * 16 + 8)
SMD_TILT_ANGLE_TH = (179 * 16 + 12)
BAC_SMD_ST_TH = (179 * 16 + 8)
BAC_ST_ALPHA4 = (180 * 16 + 12)
BAC_ST_ALPHA4A = (176 * 16 + 12)

# Wake on Motion
WOM_ENABLE = (64 * 16 + 14)
WOM_STATUS = (64 * 16 + 6)
WOM_THRESHOLD_DMP = (64 * 16) # Renamed by PaulZC to avoid duplication with the Bank 2 Reg 0x13
WOM_CNTR_TH = (64 * 16 + 12)

# Activity Recognition
BAC_RATE = (48 * 16 + 10)
BAC_STATE = (179 * 16 + 0)
BAC_STATE_PREV = (179 * 16 + 4)
BAC_ACT_ON = (182 * 16 + 0)
BAC_ACT_OFF = (183 * 16 + 0)
BAC_STILL_S_F = (177 * 16 + 0)
BAC_RUN_S_F = (177 * 16 + 4)
BAC_DRIVE_S_F = (178 * 16 + 0)
BAC_WALK_S_F = (178 * 16 + 4)
BAC_SMD_S_F = (178 * 16 + 8)
BAC_BIKE_S_F = (178 * 16 + 12)
BAC_E1_SHORT = (146 * 16 + 0)
BAC_E2_SHORT = (146 * 16 + 4)
BAC_E3_SHORT = (146 * 16 + 8)
BAC_VAR_RUN = (148 * 16 + 12)
BAC_TILT_INIT = (181 * 16 + 0)
BAC_MAG_ON = (225 * 16 + 0)
BAC_PS_ON = (74 * 16 + 0)
BAC_BIKE_PREFERENCE = (173 * 16 + 8)
BAC_MAG_I2C_ADDR = (229 * 16 + 8)
BAC_PS_I2C_ADDR = (75 * 16 + 4)
BAC_DRIVE_CONFIDENCE = (144 * 16 + 0)
BAC_WALK_CONFIDENCE = (144 * 16 + 4)
BAC_SMD_CONFIDENCE = (144 * 16 + 8)
BAC_BIKE_CONFIDENCE = (144 * 16 + 12)
BAC_STILL_CONFIDENCE = (145 * 16 + 0)
BAC_RUN_CONFIDENCE = (145 * 16 + 4)
BAC_MODE_CNTR = (150 * 16)
BAC_STATE_T_PREV = (185 * 16 + 4)
BAC_ACT_T_ON = (184 * 16 + 0)
BAC_ACT_T_OFF = (184 * 16 + 4)
BAC_STATE_WRDBS_PREV = (185 * 16 + 8)
BAC_ACT_WRDBS_ON = (184 * 16 + 8)
BAC_ACT_WRDBS_OFF = (184 * 16 + 12)
BAC_ACT_ON_OFF = (190 * 16 + 2)
PREV_BAC_ACT_ON_OFF = (188 * 16 + 2)
BAC_CNTR = (48 * 16 + 2)

# Flip/Pick-up
FP_VAR_ALPHA = (245 * 16 + 8)
FP_STILL_TH = (246 * 16 + 4)
FP_MID_STILL_TH = (244 * 16 + 8)
FP_NOT_STILL_TH = (246 * 16 + 8)
FP_VIB_REJ_TH = (241 * 16 + 8)
FP_MAX_PICKUP_T_TH = (244 * 16 + 12)
FP_PICKUP_TIMEOUT_TH = (248 * 16 + 8)
FP_STILL_CONST_TH = (246 * 16 + 12)
FP_MOTION_CONST_TH = (240 * 16 + 8)
FP_VIB_COUNT_TH = (242 * 16 + 8)
FP_STEADY_TILT_TH = (247 * 16 + 8)
FP_STEADY_TILT_UP_TH = (242 * 16 + 12)
FP_Z_FLAT_TH_MINUS = (243 * 16 + 8)
FP_Z_FLAT_TH_PLUS = (243 * 16 + 12)
FP_DEV_IN_POCKET_TH = (76 * 16 + 12)
FP_PICKUP_CNTR = (247 * 16 + 4)
FP_RATE = (240 * 16 + 12)

# Gyro FSR
GYRO_FULLSCALE = (72 * 16 + 12)

# Accel FSR
# The DMP scales accel raw data internally to align 1g as 2^25.
# To do this and output hardware unit again as configured FSR, write 0x4000000 to ACC_SCALE DMP register, and write 0x40000 to ACC_SCALE2 DMP register.
ACC_SCALE = (30 * 16 + 0) # 32-bit: Write accel scaling value for internal use
ACC_SCALE2 = (79 * 16 + 4) # 32-bit: Write accel scaling down value

# EIS authentication
EIS_AUTH_INPUT = (160 * 16 + 4)
EIS_AUTH_OUTPUT = (160 * 16 + 0)

# B2S
B2S_RATE = (48 * 16 + 8)

# B2S mounting matrix
B2S_MTX_00 = (208 * 16)
B2S_MTX_01 = (208 * 16 + 4)
B2S_MTX_02 = (208 * 16 + 8)
B2S_MTX_10 = (208 * 16 + 12)
B2S_MTX_11 = (209 * 16)
B2S_MTX_12 = (209 * 16 + 4)
B2S_MTX_20 = (209 * 16 + 8)
B2S_MTX_21 = (209 * 16 + 12)
B2S_MTX_22 = (210 * 16)

# Dmp3 orientation parameters (Q30) initialization
Q0_QUAT6 = (33 * 16 + 0)
Q1_QUAT6 = (33 * 16 + 4)
Q2_QUAT6 = (33 * 16 + 8)
Q3_QUAT6 = (33 * 16 + 12)

DMP_ODR_REG_ACCEL = ODR_ACCEL               # ODR_ACCEL Register for accel ODR
DMP_ODR_REG_GYRO = ODR_GYRO                 # ODR_GYRO Register for gyro ODR
DMP_ODR_REG_CPASS = ODR_CPASS               # ODR_CPASS Register for compass ODR
DMP_ODR_REG_ALS = ODR_ALS                   # ODR_ALS Register for ALS ODR
DMP_ODR_REG_QUAT6 = ODR_QUAT6               # ODR_QUAT6 Register for 6-axis quaternion ODR
DMP_ODR_REG_QUAT9 = ODR_QUAT9               # ODR_QUAT9 Register for 9-axis quaternion ODR
DMP_ODR_REG_PQUAT6 = ODR_PQUAT6             # ODR_PQUAT6 Register for 6-axis pedometer quaternion ODR
DMP_ODR_REG_GEOMAG = ODR_GEOMAG             # ODR_GEOMAG Register for Geomag RV ODR
DMP_ODR_REG_PRESSURE = ODR_PRESSURE         # ODR_PRESSURE Register for pressure ODR
DMP_ODR_REG_GYRO_CALIBR = ODR_GYRO_CALIBR   # ODR_GYRO_CALIBR Register for calibrated gyro ODR
DMP_ODR_REG_CPASS_CALIBR = ODR_CPASS_CALIBR # ODR_CPASS_CALIBR Register for calibrated compass ODR


# Sensor identifier for control function
class inv_icm20948_sensor(IntEnum):
    INV_ICM20948_SENSOR_ACCELEROMETER = 0,
    INV_ICM20948_SENSOR_GYROSCOPE = 1
    INV_ICM20948_SENSOR_RAW_ACCELEROMETER = 2
    INV_ICM20948_SENSOR_RAW_GYROSCOPE = 3
    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED = 4
    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED = 5
    INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON = 6
    INV_ICM20948_SENSOR_STEP_DETECTOR = 7
    INV_ICM20948_SENSOR_STEP_COUNTER = 8
    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR = 9
    INV_ICM20948_SENSOR_ROTATION_VECTOR = 10
    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR = 11
    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD = 12
    INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION = 13
    INV_ICM20948_SENSOR_FLIP_PICKUP = 14
    INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR = 15
    INV_ICM20948_SENSOR_GRAVITY = 16
    INV_ICM20948_SENSOR_LINEAR_ACCELERATION = 17
    INV_ICM20948_SENSOR_ORIENTATION = 18
    INV_ICM20948_SENSOR_B2S = 19
    INV_ICM20948_SENSOR_RAW_MAGNETOMETER = 20
    INV_ICM20948_SENSOR_MAX = 21


# enum for android sensor
class ANDROID_SENSORS(IntEnum):
    ANDROID_SENSOR_META_DATA = 0
    ANDROID_SENSOR_ACCELEROMETER = 1
    ANDROID_SENSOR_GEOMAGNETIC_FIELD = 2
    ANDROID_SENSOR_ORIENTATION = 3
    ANDROID_SENSOR_GYROSCOPE = 4
    ANDROID_SENSOR_LIGHT = 5
    ANDROID_SENSOR_PRESSURE = 6
    ANDROID_SENSOR_TEMPERATURE = 7
    ANDROID_SENSOR_WAKEUP_PROXIMITY = 8
    ANDROID_SENSOR_GRAVITY = 9
    ANDROID_SENSOR_LINEAR_ACCELERATION = 10
    ANDROID_SENSOR_ROTATION_VECTOR = 11
    ANDROID_SENSOR_HUMIDITY = 12
    ANDROID_SENSOR_AMBIENT_TEMPERATURE = 13
    ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED = 14
    ANDROID_SENSOR_GAME_ROTATION_VECTOR = 15
    ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED = 16
    ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION = 17
    ANDROID_SENSOR_STEP_DETECTOR = 18
    ANDROID_SENSOR_STEP_COUNTER = 19
    ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR = 20
    ANDROID_SENSOR_HEART_RATE = 21
    ANDROID_SENSOR_PROXIMITY = 22

    ANDROID_SENSOR_WAKEUP_ACCELEROMETER = 23
    ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD = 24
    ANDROID_SENSOR_WAKEUP_ORIENTATION = 25
    ANDROID_SENSOR_WAKEUP_GYROSCOPE = 26
    ANDROID_SENSOR_WAKEUP_LIGHT = 27
    ANDROID_SENSOR_WAKEUP_PRESSURE = 28
    ANDROID_SENSOR_WAKEUP_GRAVITY = 29
    ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION = 30
    ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR = 31
    ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY = 32
    ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE = 33
    ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED = 34
    ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR = 35
    ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED = 36
    ANDROID_SENSOR_WAKEUP_STEP_DETECTOR = 37
    ANDROID_SENSOR_WAKEUP_STEP_COUNTER = 38
    ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR = 39
    ANDROID_SENSOR_WAKEUP_HEART_RATE = 40
    ANDROID_SENSOR_WAKEUP_TILT_DETECTOR = 41

    ANDROID_SENSOR_RAW_ACCELEROMETER = 42
    ANDROID_SENSOR_RAW_GYROSCOPE = 43
    ANDROID_SENSOR_NUM_MAX = 44
    ANDROID_SENSOR_B2S = 45
    ANDROID_SENSOR_FLIP_PICKUP = 46
    ANDROID_SENSOR_ACTIVITY_CLASSIFICATON = 47
    ANDROID_SENSOR_SCREEN_ROTATION = 48
    SELF_TEST = 49
    SETUP = 50
    GENERAL_SENSORS_MAX = 51


def sensor_type_2_android_sensor(sensor):
    lookup = {
        inv_icm20948_sensor.INV_ICM20948_SENSOR_ACCELEROMETER: ANDROID_SENSORS.ANDROID_SENSOR_ACCELEROMETER,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_GYROSCOPE: ANDROID_SENSORS.ANDROID_SENSOR_GYROSCOPE,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_RAW_ACCELEROMETER: ANDROID_SENSORS.ANDROID_SENSOR_RAW_ACCELEROMETER,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_RAW_GYROSCOPE: ANDROID_SENSORS.ANDROID_SENSOR_RAW_GYROSCOPE,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED: ANDROID_SENSORS.ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED: ANDROID_SENSORS.ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON: ANDROID_SENSORS.ANDROID_SENSOR_ACTIVITY_CLASSIFICATON,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_STEP_DETECTOR: ANDROID_SENSORS.ANDROID_SENSOR_STEP_DETECTOR,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_STEP_COUNTER: ANDROID_SENSORS.ANDROID_SENSOR_STEP_COUNTER,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR: ANDROID_SENSORS.ANDROID_SENSOR_GAME_ROTATION_VECTOR,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_ROTATION_VECTOR: ANDROID_SENSORS.ANDROID_SENSOR_ROTATION_VECTOR,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR: ANDROID_SENSORS.ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD: ANDROID_SENSORS.ANDROID_SENSOR_GEOMAGNETIC_FIELD,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION: ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_FLIP_PICKUP: ANDROID_SENSORS.ANDROID_SENSOR_FLIP_PICKUP,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR: ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_GRAVITY: ANDROID_SENSORS.ANDROID_SENSOR_GRAVITY,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_LINEAR_ACCELERATION: ANDROID_SENSORS.ANDROID_SENSOR_LINEAR_ACCELERATION,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_ORIENTATION: ANDROID_SENSORS.ANDROID_SENSOR_ORIENTATION,
        inv_icm20948_sensor.INV_ICM20948_SENSOR_B2S: ANDROID_SENSORS.ANDROID_SENSOR_B2S,
    }
    if sensor in lookup:
        return lookup[sensor]

    raise Exception('Sensor type not supported.')

def inv_androidSensor_to_control_bits(sensor):
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
        ANDROID_SENSORS.ANDROID_SENSOR_META_DATA: 0xFFFF,
        ANDROID_SENSORS.ANDROID_SENSOR_ACCELEROMETER: 0x8008,
        ANDROID_SENSORS.ANDROID_SENSOR_GEOMAGNETIC_FIELD: 0x0028,
        ANDROID_SENSORS.ANDROID_SENSOR_ORIENTATION: 0x0408,
        ANDROID_SENSORS.ANDROID_SENSOR_GYROSCOPE: 0x4048,
        ANDROID_SENSORS.ANDROID_SENSOR_LIGHT: 0x1008,
        ANDROID_SENSORS.ANDROID_SENSOR_PRESSURE: 0x0088,
        ANDROID_SENSORS.ANDROID_SENSOR_TEMPERATURE: 0xFFFF,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_PROXIMITY: 0xFFFF,
        ANDROID_SENSORS.ANDROID_SENSOR_GRAVITY: 0x0808,
        ANDROID_SENSORS.ANDROID_SENSOR_LINEAR_ACCELERATION: 0x8808,
        ANDROID_SENSORS.ANDROID_SENSOR_ROTATION_VECTOR: 0x0408,
        ANDROID_SENSORS.ANDROID_SENSOR_HUMIDITY: 0xFFFF,
        ANDROID_SENSORS.ANDROID_SENSOR_AMBIENT_TEMPERATURE: 0xFFFF,
        ANDROID_SENSORS.ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED: 0x2008,
        ANDROID_SENSORS.ANDROID_SENSOR_GAME_ROTATION_VECTOR: 0x0808,
        ANDROID_SENSORS.ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED: 0x4008,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION: 0x0000,
        ANDROID_SENSORS.ANDROID_SENSOR_STEP_DETECTOR: 0x0018,
        ANDROID_SENSORS.ANDROID_SENSOR_STEP_COUNTER: 0x0010,
        ANDROID_SENSORS.ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR: 0x0108,
        ANDROID_SENSORS.ANDROID_SENSOR_HEART_RATE: 0xFFFF,
        ANDROID_SENSORS.ANDROID_SENSOR_PROXIMITY: 0xFFFF,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_ACCELEROMETER: 0x8008,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD: 0x0028,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_ORIENTATION: 0x0408,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_GYROSCOPE: 0x4048,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_LIGHT: 0x1008,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_PRESSURE: 0x0088,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_GRAVITY: 0x0808,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION: 0x8808,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR: 0x0408,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY: 0xFFFF,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE: 0xFFFF,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED: 0x2008,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR: 0x0808,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED: 0x4008,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_STEP_DETECTOR: 0x0018,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_STEP_COUNTER: 0x0010,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR: 0x0108,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_HEART_RATE: 0xFFFF,
        ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_TILT_DETECTOR: 0x0000,
        ANDROID_SENSORS.ANDROID_SENSOR_RAW_ACCELEROMETER: 0x8008,
        ANDROID_SENSORS.ANDROID_SENSOR_RAW_GYROSCOPE: 0x4048,
    }

    if sensor in lookup and lookup[sensor] != 0xFFFF:
        return lookup[sensor]

    raise Exception('Sensor type not supported.')

# Determines which base sensor needs to be on
INV_NEEDS_ACCEL_MASK = [
    ANDROID_SENSORS.ANDROID_SENSOR_ACCELEROMETER,
    ANDROID_SENSORS.ANDROID_SENSOR_ORIENTATION,
    ANDROID_SENSORS.ANDROID_SENSOR_GRAVITY,
    ANDROID_SENSORS.ANDROID_SENSOR_LINEAR_ACCELERATION,
    ANDROID_SENSORS.ANDROID_SENSOR_ROTATION_VECTOR,
    ANDROID_SENSORS.ANDROID_SENSOR_GAME_ROTATION_VECTOR,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,
    ANDROID_SENSORS.ANDROID_SENSOR_STEP_DETECTOR,
    ANDROID_SENSORS.ANDROID_SENSOR_STEP_COUNTER,
    ANDROID_SENSORS.ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_ORIENTATION,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_GRAVITY,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,

    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
    ANDROID_SENSORS.ANDROID_SENSOR_RAW_ACCELEROMETER,
]

INV_NEEDS_GYRO_MASK = [
    ANDROID_SENSORS.ANDROID_SENSOR_ORIENTATION,
    ANDROID_SENSORS.ANDROID_SENSOR_GYROSCOPE,
    ANDROID_SENSORS.ANDROID_SENSOR_GRAVITY,
    ANDROID_SENSORS.ANDROID_SENSOR_LINEAR_ACCELERATION,
    ANDROID_SENSORS.ANDROID_SENSOR_ROTATION_VECTOR,
    ANDROID_SENSORS.ANDROID_SENSOR_GAME_ROTATION_VECTOR,
    ANDROID_SENSORS.ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_ORIENTATION,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_GYROSCOPE,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_GRAVITY,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,

    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
    ANDROID_SENSORS.ANDROID_SENSOR_RAW_GYROSCOPE,
]

INV_NEEDS_COMPASS_MASK = [
    ANDROID_SENSORS.ANDROID_SENSOR_GEOMAGNETIC_FIELD,
    ANDROID_SENSORS.ANDROID_SENSOR_ORIENTATION,
    ANDROID_SENSORS.ANDROID_SENSOR_ROTATION_VECTOR,
    ANDROID_SENSORS.ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,
    ANDROID_SENSORS.ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_ORIENTATION,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,

    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR,
]

INV_NEEDS_PRESSURE = [
    ANDROID_SENSORS.ANDROID_SENSOR_PRESSURE,
    ANDROID_SENSORS.ANDROID_SENSOR_WAKEUP_PRESSURE,
]

DMP_DATA_READY_GYRO = 0x0001             # Gyro samples available
DMP_DATA_READY_ACCEL = 0x0002            # Accel samples available
DMP_DATA_READY_SECONDARY_COMPASS = 0x0008 # Secondary compass samples available

DMP_DATA_OUTPUT_CONTROL_1_STEP_IND_0 = 0x0001     # Pedometer Step Indicator Bit 0
DMP_DATA_OUTPUT_CONTROL_1_STEP_IND_1 = 0x0002     # Pedometer Step Indicator Bit 1
DMP_DATA_OUTPUT_CONTROL_1_STEP_IND_2 = 0x0004     # Pedometer Step Indicator Bit 2
DMP_DATA_OUTPUT_CONTROL_1_HEADER2 = 0x0008        # Header 2
DMP_DATA_OUTPUT_CONTROL_1_STEP_DETECTOR = 0x0010  # Pedometer Step Detector
DMP_DATA_OUTPUT_CONTROL_1_COMPASS_CALIBR = 0x0020 # 32-bit calibrated compass
DMP_DATA_OUTPUT_CONTROL_1_GYRO_CALIBR = 0x0040    # 32-bit calibrated gyro
DMP_DATA_OUTPUT_CONTROL_1_PRESSURE = 0x0080       # 16-bit Pressure
DMP_DATA_OUTPUT_CONTROL_1_GEOMAG = 0x0100         # 32-bit Geomag rv + heading accuracy
DMP_DATA_OUTPUT_CONTROL_1_PQUAT6 = 0x0200         # 16-bit pedometer quaternion
DMP_DATA_OUTPUT_CONTROL_1_QUAT9 = 0x0400          # 32-bit 9-axis quaternion + heading accuracy
DMP_DATA_OUTPUT_CONTROL_1_QUAT6 = 0x0800          # 32-bit 6-axis quaternion
DMP_DATA_OUTPUT_CONTROL_1_ALS = 0x1000            # 16-bit ALS
DMP_DATA_OUTPUT_CONTROL_1_COMPASS = 0x2000        # 16-bit compass
DMP_DATA_OUTPUT_CONTROL_1_GYRO = 0x4000           # 16-bit gyro
DMP_DATA_OUTPUT_CONTROL_1_ACCEL = 0x8000           # 16-bit accel

DMP_DATA_OUTPUT_CONTROL_2_SECONDARY_ON_OFF = 0x0040
DMP_DATA_OUTPUT_CONTROL_2_ACTIVITY_RECOGNITION_BAC = 0x0080
DMP_DATA_OUTPUT_CONTROL_2_BATCH_MODE_ENABLE = 0x0100
DMP_DATA_OUTPUT_CONTROL_2_PICKUP = 0x0400
DMP_DATA_OUTPUT_CONTROL_2_FSYNC_DETECTION = 0x0800
DMP_DATA_OUTPUT_CONTROL_2_COMPASS_ACCURACY = 0x1000
DMP_DATA_OUTPUT_CONTROL_2_GYRO_ACCURACY = 0x2000
DMP_DATA_OUTPUT_CONTROL_2_ACCEL_ACCURACY = 0x4000


DMP_MOTION_EVENT_CONTROL_ACTIVITY_RECOG_PEDOM_ACCEL = 0x0002 # Activity Recognition / Pedometer accel only
DMP_MOTION_EVENT_CONTROL_BRING_LOOK_TO_SEE = 0x0004
DMP_MOTION_EVENT_CONTROL_GEOMAG = 0x0008 # Geomag rv
DMP_MOTION_EVENT_CONTROL_PICKUP = 0x0010
DMP_MOTION_EVENT_CONTROL_BTS = 0x0020
DMP_MOTION_EVENT_CONTROL_9AXIS = 0x0040
DMP_MOTION_EVENT_CONTROL_COMPASS_CALIBR = 0x0080
DMP_MOTION_EVENT_CONTROL_GYRO_CALIBR = 0x0100
DMP_MOTION_EVENT_CONTROL_ACCEL_CALIBR = 0x0200
DMP_MOTION_EVENT_CONTROL_SIGNIFICANT_MOTION_DET = 0x0800
DMP_MOTION_EVENT_CONTROL_TILT_INTERRUPT = 0x1000
DMP_MOTION_EVENT_CONTROL_PEDOMETER_INTERRUPT = 0x2000
DMP_MOTION_EVENT_CONTROL_ACTIVITY_RECOG_PEDOM = 0x4000
DMP_MOTION_EVENT_CONTROL_BAC_WEARABLE = 0x8000


DMP_HEADER_BITMAP_HEADER2 = 0x0008
DMP_HEADER_BITMAP_STEP_DETECTOR = 0x0010
DMP_HEADER_BITMAP_COMPASS_CALIBR = 0x0020
DMP_HEADER_BITMAP_GYRO_CALIBR = 0x0040
DMP_HEADER_BITMAP_PRESSURE = 0x0080
DMP_HEADER_BITMAP_GEOMAG = 0x0100
DMP_HEADER_BITMAP_PQUAT6 = 0x0200
DMP_HEADER_BITMAP_QUAT9 = 0x0400
DMP_HEADER_BITMAP_QUAT6 = 0x0800
DMP_HEADER_BITMAP_ALS = 0x1000
DMP_HEADER_BITMAP_COMPASS = 0x2000
DMP_HEADER_BITMAP_GYRO = 0x4000
DMP_HEADER_BITMAP_ACCEL = 0x8000

DMP_HEADER2_BITMAP_SECONDARY_ON_OFF = 0x0040
DMP_HEADER2_BITMAP_ACTIVITY_RECOG = 0x0080
DMP_HEADER2_BITMAP_PICKUP = 0x0400
DMP_HEADER2_BITMAP_FSYNC = 0x0800
DMP_HEADER2_BITMAP_COMPASS_ACCURACY = 0x1000
DMP_HEADER2_BITMAP_GYRO_ACCURACY = 0x2000
DMP_HEADER2_BITMAP_ACCEL_ACCURACY = 0x4000


#   typedef struct # DMP Activity Recognition data
#   {
#     uint8_t Drive : 1;
#     uint8_t Walk : 1;
#     uint8_t Run : 1;
#     uint8_t Bike : 1;
#     uint8_t Tilt : 1;
#     uint8_t Still : 1;
#     uint8_t reserved : 2;
#   } icm_20948_DMP_Activity_t;

#   typedef struct # DMP Secondary On/Off data
#   {
#     uint16_t Gyro_Off : 1;
#     uint16_t Gyro_On : 1;
#     uint16_t Compass_Off : 1;
#     uint16_t Compass_On : 1;
#     uint16_t Proximity_Off : 1;
#     uint16_t Proximity_On : 1;
#     uint16_t reserved : 10;
#   } icm_20948_DMP_Secondary_On_Off_t;

ICM_20948_DMP_HEADER_BYTES = 2
ICM_20948_DMP_HEADER2_BYTES = 2
ICM_20948_DMP_RAW_ACCEL_BYTES = 6
ICM_20948_DMP_RAW_GYRO_BYTES = 6
ICM_20948_DMP_GYRO_BIAS_BYTES = 6
ICM_20948_DMP_COMPASS_BYTES = 6
ICM_20948_DMP_ALS_BYTES = 8
ICM_20948_DMP_QUAT6_BYTES = 12
ICM_20948_DMP_QUAT9_BYTES = 14
# <-- lcm20948MPUFifoControl.c suggests icm_20948_DMP_Step_Detector_Bytes comes here <--
ICM_20948_DMP_PQUAT6_BYTES = 6
ICM_20948_DMP_GEOMAG_BYTES = 14
ICM_20948_DMP_PRESSURE_BYTES = 6
ICM_20948_DMP_GYRO_CALIBR_BYTES = 12 # lcm20948MPUFifoControl.c suggests icm_20948_DMP_Gyro_Calibr_Bytes is not supported?
ICM_20948_DMP_COMPASS_CALIBR_BYTES = 12
ICM_20948_DMP_STEP_DETECTOR_BYTES = 4 # See note above
ICM_20948_DMP_ACCEL_ACCURACY_BYTES = 2
ICM_20948_DMP_GYRO_ACCURACY_BYTES = 2
ICM_20948_DMP_COMPASS_ACCURACY_BYTES = 2
ICM_20948_DMP_FSYNC_DETECTION_BYTES = 2 # lcm20948MPUFifoControl.c suggests icm_20948_DMP_Fsync_Detection_Bytes is not supported?
ICM_20948_DMP_PICKUP_BYTES = 2
ICM_20948_DMP_ACTIVITY_RECOGNITION_BYTES = 6
ICM_20948_DMP_SECONDARY_ON_OFF_BYTES = 2
ICM_20948_DMP_FOOTER_BYTES = 2
ICM_20948_DMP_MAXIMUM_BYTES = 14 # The most bytes we will attempt to read from the FIFO in one go

INV_MAX_SERIAL_WRITE = 16 # Max size that can be written across I2C or SPI data lines

#   typedef struct
#   {
#     uint16_t header;
#     uint16_t header2;
#     union
#     {
#       uint8_t Bytes[icm_20948_DMP_Raw_Accel_Bytes];
#       struct
#       {
#         int16_t X;
#         int16_t Y;
#         int16_t Z;
#       } Data;
#     } Raw_Accel;
#     union
#     {
#       uint8_t Bytes[icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes];
#       struct
#       {
#         int16_t X;
#         int16_t Y;
#         int16_t Z;
#         int16_t BiasX;
#         int16_t BiasY;
#         int16_t BiasZ;
#       } Data;
#     } Raw_Gyro;
#     union
#     {
#       uint8_t Bytes[icm_20948_DMP_Compass_Bytes];
#       struct
#       {
#         int16_t X;
#         int16_t Y;
#         int16_t Z;
#       } Data;
#     } Compass;
#     uint8_t ALS[icm_20948_DMP_ALS_Bytes]; # Byte[0]: Dummy, Byte[2:1]: Ch0DATA, Byte[4:3]: Ch1DATA, Byte[6:5]: PDATA, Byte[7]: Dummy
#     # The 6-Axis and 9-axis Quaternion outputs each consist of 12 bytes of data.
#     # These 12 bytes in turn consists of three 4-byte elements.
#     # 9-axis quaternion data and Geomag rv is always followed by 2-bytes of heading accuracy, hence the size of Quat9 and Geomag data size in the FIFO is 14 bytes.
#     # Quaternion data for both cases is cumulative/integrated values.
#     # For a given quaternion Q, the ordering of its elements is {Q1, Q2, Q3}.
#     # Each element is represented using Big Endian byte order.
#     # Q0 value is computed from this equation: Q20 + Q21 + Q22 + Q23 = 1.
#     # In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
#     # The quaternion data is scaled by 2^30.
#     union
#     {
#       uint8_t Bytes[icm_20948_DMP_Quat6_Bytes];
#       struct
#       {
#         int32_t Q1;
#         int32_t Q2;
#         int32_t Q3;
#       } Data;
#     } Quat6;
#     union
#     {
#       uint8_t Bytes[icm_20948_DMP_Quat9_Bytes];
#       struct
#       {
#         int32_t Q1;
#         int32_t Q2;
#         int32_t Q3;
#         int16_t Accuracy;
#       } Data;
#     } Quat9;
#     union
#     {
#       uint8_t Bytes[icm_20948_DMP_PQuat6_Bytes];
#       struct
#       {
#         int16_t Q1;
#         int16_t Q2;
#         int16_t Q3;
#       } Data;
#     } PQuat6;
#     union
#     {
#       uint8_t Bytes[icm_20948_DMP_Geomag_Bytes];
#       struct
#       {
#         int32_t Q1;
#         int32_t Q2;
#         int32_t Q3;
#         int16_t Accuracy;
#       } Data;
#     } Geomag;
#     uint8_t Pressure[6]; # Byte [2:0]: Pressure data, Byte [5:3]: Temperature data
#     union
#     {
#       uint8_t Bytes[icm_20948_DMP_Gyro_Calibr_Bytes];
#       struct
#       {
#         int32_t X;
#         int32_t Y;
#         int32_t Z;
#       } Data;
#     } Gyro_Calibr; # Hardware unit scaled by 2^15
#     union
#     {
#       uint8_t Bytes[icm_20948_DMP_Compass_Calibr_Bytes];
#       struct
#       {
#         int32_t X;
#         int32_t Y;
#         int32_t Z;
#       } Data;
#     } Compass_Calibr;             # The unit is uT scaled by 2^16
#     uint32_t Pedometer_Timestamp; # Timestamp as DMP cycle
#     uint16_t Accel_Accuracy;      # The accuracy is expressed as 0~3. The lowest is 0 and 3 is the highest.
#     uint16_t Gyro_Accuracy;       # The accuracy is expressed as 0~3. The lowest is 0 and 3 is the highest.
#     uint16_t Compass_Accuracy;    # The accuracy is expressed as 0~3. The lowest is 0 and 3 is the highest.
#     uint16_t Fsync_Delay_Time;    # The data is delay time between Fsync event and the 1st ODR event after Fsync event.
#     uint16_t Pickup;              # The value “2” indicates pick up is detected.
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
#     union
#     {
#       uint8_t Bytes[icm_20948_DMP_Activity_Recognition_Bytes];
#       struct
#       {
#         icm_20948_DMP_Activity_t State_Start;
#         icm_20948_DMP_Activity_t State_End;
#         uint32_t Timestamp;
#       } Data;
#     } Activity_Recognition;
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
#     union
#     {
#       uint8_t Bytes[icm_20948_DMP_Secondary_On_Off_Bytes];
#       icm_20948_DMP_Secondary_On_Off_t Sensors;
#     } Secondary_On_Off;
#     uint16_t Footer; # Gyro count?
#   } icm_20948_DMP_data_t;

