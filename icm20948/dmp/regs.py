from enum import IntEnum, Enum


START_ADDRESS = 0x1000
MEM_BANK_SIZE = 256
LOAD_START = 0x90

CFG_FIFO_SIZE = 4222

# AGB0_REG_INT_STATUS bit definitions
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
class Output_Data_Rate(IntEnum):
    ACCEL = (11 * 16 + 14)
    GYRO = (11 * 16 + 10)
    CPASS = (11 * 16 + 6)
    ALS = (11 * 16 + 2)
    QUAT6 = (10 * 16 + 12)
    QUAT9 = (10 * 16 + 8)
    PQUAT6 = (10 * 16 + 4)
    GEOMAG = (10 * 16 + 0)
    PRESSURE = (11 * 16 + 12)
    GYRO_CALIBR = (11 * 16 + 8)
    CPASS_CALIBR = (11 * 16 + 4)


# sensor output data rate counter: all 16-bit
class Output_Data_Rate_Counter(IntEnum):
    ACCEL = (9 * 16 + 14)
    GYRO = (9 * 16 + 10)
    CPASS = (9 * 16 + 6)
    ALS = (9 * 16 + 2)
    QUAT6 = (8 * 16 + 12)
    QUAT9 = (8 * 16 + 8)
    PQUAT6 = (8 * 16 + 4)
    GEOMAG = (8 * 16 + 0)
    PRESSURE = (9 * 16 + 12)
    GYRO_CALIBR = (9 * 16 + 8)
    CPASS_CALIBR = (9 * 16 + 4)


class Output_Data_Rate_Control(Enum):
    ACCEL = (Output_Data_Rate.ACCEL, Output_Data_Rate_Counter.ACCEL)
    GYRO = (Output_Data_Rate.GYRO, Output_Data_Rate_Counter.GYRO)
    CPASS = (Output_Data_Rate.CPASS, Output_Data_Rate_Counter.CPASS)
    ALS = (Output_Data_Rate.ALS, Output_Data_Rate_Counter.ALS)
    QUAT6 = (Output_Data_Rate.QUAT6, Output_Data_Rate_Counter.QUAT6)
    QUAT9 = (Output_Data_Rate.QUAT9, Output_Data_Rate_Counter.QUAT9)
    PQUAT6 = (Output_Data_Rate.PQUAT6, Output_Data_Rate_Counter.PQUAT6)
    GEOMAG = (Output_Data_Rate.GEOMAG, Output_Data_Rate_Counter.GEOMAG)
    PRESSURE = (Output_Data_Rate.PRESSURE, Output_Data_Rate_Counter.PRESSURE)
    GYRO_CALIBR = (Output_Data_Rate.GYRO_CALIBR, Output_Data_Rate_Counter.GYRO_CALIBR)
    CPASS_CALIBR = (Output_Data_Rate.CPASS_CALIBR, Output_Data_Rate_Counter.CPASS_CALIBR)


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
class Bias_Calibration(IntEnum):
    GYRO_X = (139 * 16 + 4)
    GYRO_Y = (139 * 16 + 8)
    GYRO_Z = (139 * 16 + 12)
    ACCEL_X = (110 * 16 + 4)
    ACCEL_Y = (110 * 16 + 8)
    ACCEL_Z = (110 * 16 + 12)
    CPASS_X = (126 * 16 + 4)
    CPASS_Y = (126 * 16 + 8)
    CPASS_Z = (126 * 16 + 12)


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


class mpu_gyro_fs(IntEnum):
    MPU_FS_250dps = 0
    MPU_FS_500dps = 1
    MPU_FS_1000dps = 2
    MPU_FS_2000dps = 3
    NUM_MPU_GFS = 4


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


class Data_Ready(IntEnum):
    GYRO = 0x0001             # Gyro samples available
    ACCEL = 0x0002            # Accel samples available
    SECONDARY_COMPASS = 0x0008 # Secondary compass samples available


class Data_Output_Control_1(IntEnum):
    STEP_IND_0 = 0x0001     # Pedometer Step Indicator Bit 0
    STEP_IND_1 = 0x0002     # Pedometer Step Indicator Bit 1
    STEP_IND_2 = 0x0004     # Pedometer Step Indicator Bit 2
    HEADER2 = 0x0008        # Header 2
    STEP_DETECTOR = 0x0010  # Pedometer Step Detector
    COMPASS_CALIBR = 0x0020 # 32-bit calibrated compass
    GYRO_CALIBR = 0x0040    # 32-bit calibrated gyro
    PRESSURE = 0x0080       # 16-bit Pressure
    GEOMAG = 0x0100         # 32-bit Geomag rv + heading accuracy
    PQUAT6 = 0x0200         # 16-bit pedometer quaternion
    QUAT9 = 0x0400          # 32-bit 9-axis quaternion + heading accuracy
    QUAT6 = 0x0800          # 32-bit 6-axis quaternion
    ALS = 0x1000            # 16-bit ALS
    COMPASS = 0x2000        # 16-bit compass
    GYRO = 0x4000           # 16-bit gyro
    ACCEL = 0x8000           # 16-bit accel


class Data_Output_Control_2(IntEnum):
    SECONDARY_ON_OFF = 0x0040
    ACTIVITY_RECOGNITION_BAC = 0x0080
    BATCH_MODE_ENABLE = 0x0100
    PICKUP = 0x0400
    FSYNC_DETECTION = 0x0800
    COMPASS_ACCURACY = 0x1000
    GYRO_ACCURACY = 0x2000
    ACCEL_ACCURACY = 0x4000


class Motion_Event_Control(IntEnum):
    ACTIVITY_RECOG_PEDOM_ACCEL = 0x0002  # Activity Recognition / Pedometer accel only
    BRING_LOOK_TO_SEE = 0x0004
    GEOMAG = 0x0008  # Geomag rv
    PICKUP = 0x0010
    BTS = 0x0020
    AXIS9 = 0x0040
    COMPASS_CALIBR = 0x0080
    GYRO_CALIBR = 0x0100
    ACCEL_CALIBR = 0x0200
    SIGNIFICANT_MOTION_DET = 0x0800
    TILT_INTERRUPT = 0x1000
    PEDOMETER_INTERRUPT = 0x2000
    ACTIVITY_RECOG_PEDOM = 0x4000
    BAC_WEARABLE = 0x8000
