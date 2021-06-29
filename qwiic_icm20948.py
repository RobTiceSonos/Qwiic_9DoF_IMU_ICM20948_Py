#-----------------------------------------------------------------------------
# qwiic_icm20948.py
#
# Python library for the SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic).
#
# https://www.sparkfun.com/products/15335
#
#------------------------------------------------------------------------
#
# Written by  SparkFun Electronics, March 2020
#
# This python library supports the SparkFun Electroncis qwiic
# qwiic sensor/board ecosystem
#
# More information on qwiic is at https:// www.sparkfun.com/qwiic
#
# Do you like this library? Help support SparkFun. Buy a board!
#==================================================================================
# Copyright (c) 2020 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#==================================================================================

"""
qwiic_icm20948
============
Python module for the [SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic)](https://www.sparkfun.com/products/15335)

This python package is a port of the existing [SparkFun ICM-20948 Arduino Library](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary)

This package can be used in conjunction with the overall [SparkFun qwiic Python Package](https://github.com/sparkfun/Qwiic_Py)

New to qwiic? Take a look at the entire [SparkFun qwiic ecosystem](https://www.sparkfun.com/qwiic).

"""
#-----------------------------------------------------------------------------

from fcntl import F_EXLCK
import qwiic_i2c
import time

# Define the device name and I2C addresses. These are set in the class definition
# as class variables, making them avilable without having to create a class instance.
# This allows higher level logic to rapidly create a index of qwiic devices at
# runtine
#
# The name of this device
_DEFAULT_NAME = "Qwiic ICM20948"

# Some devices have multiple available addresses - this is a list of these addresses.
# NOTE: The first address in this list is considered the default I2C address for the
# device.
_AVAILABLE_I2C_ADDRESS = [0x69, 0x68]

# define our valid chip IDs
_validChipIDs = [0xEA]

# Internal Sensor IDs, used in various functions as arguments to know who to affect
ICM_20948_Internal_Acc = (1 << 0)
ICM_20948_Internal_Gyr = (1 << 1)
ICM_20948_Internal_Mag = (1 << 2)
ICM_20948_Internal_Tmp = (1 << 3)
ICM_20948_Internal_Mst = (1 << 4) # I2C Master Ineternal

# Sample mode options
ICM_20948_Sample_Mode_Continuous = 0x00
ICM_20948_Sample_Mode_Cycled = 0x01

# Accel full scale range options [AGB2_REG_ACCEL_CONFIG]
gpm2 = 0x00 # G forces Plus or Minus (aka "gpm")
gpm4 = 0x01
gpm8 = 0x02
gpm16 = 0x03

# Gyro full scale range options [AGB2_REG_GYRO_CONFIG_1]
dps250 = 0x00 #degrees per second (aka "dps")
dps500 = 0x01
dps1000 = 0x02
dps2000 = 0x03

# Accelerometer low pass filter configuration options
# Format is dAbwB_nXbwZ - A is integer part of 3db BW, B is fraction. X is integer part of nyquist bandwidth, Y is fraction
acc_d246bw_n265bw = 0x00
acc_d246bw_n265bw_1 = 0x01
acc_d111bw4_n136bw = 0x02
acc_d50bw4_n68bw8 = 0x03
acc_d23bw9_n34bw4 = 0x04
acc_d11bw5_n17bw = 0x05
acc_d5bw7_n8bw3 = 0x06
acc_d473bw_n499bw = 0x07

# Gryo low pass filter configuration options
# Format is dAbwB_nXbwZ - A is integer part of 3db BW, B is fraction. X is integer part of nyquist bandwidth, Y is fraction
gyr_d196bw6_n229bw8 = 0x00
gyr_d151bw8_n187bw6 = 0x01
gyr_d119bw5_n154bw3 = 0x02
gyr_d51bw2_n73bw3 = 0x03
gyr_d23bw9_n35bw9 = 0x04
gyr_d11bw6_n17bw8 = 0x05
gyr_d5bw7_n8bw9 = 0x06
gyr_d361bw4_n376bw5 = 0x07

# Magnetometer specific stuff
MAG_AK09916_I2C_ADDR = 0x0C
MAG_AK09916_WHO_AM_I = 0x4809
MAG_REG_WHO_AM_I = 0x00
AK09916_mode_power_down = 0x00
AK09916_mode_single 	= (0x01 << 0)
AK09916_mode_cont_10hz 	= (0x01 << 1)
AK09916_mode_cont_20hz 	= (0x02 << 1)
AK09916_mode_cont_50hz 	= (0x03 << 1)
AK09916_mode_cont_100hz = (0x04 << 1)
AK09916_mode_self_test 	= (0x01 << 4)

# Magnetometer Registers (aka sub-addresses when reading as I2C Master)
AK09916_REG_WIA1 = 0x00
AK09916_REG_WIA2 = 0x01
AK09916_REG_RSV1 = 0x03
AK09916_REG_RSV2 = 0x04
AK09916_REG_ST1 = 0x10
AK09916_REG_HXL = 0x11
AK09916_REG_HXH = 0x12
AK09916_REG_HYL = 0x13
AK09916_REG_HYH = 0x14
AK09916_REG_HZL = 0x15
AK09916_REG_HZH = 0x16
AK09916_REG_ST2 = 0x18
AK09916_REG_CNTL2 = 0x31
AK09916_REG_CNTL3 = 0x32


# define the class that encapsulates the device being created. All information associated with this
# device is encapsulated by this class. The device class should be the only value exported
# from this module.

class QwiicIcm20948(object):
    """
    QwiicIcm20948

        :param address: The I2C address to use for the device.
                        If not provided, the default address is used.
        :param i2c_driver: An existing i2c driver object. If not provided
                        a driver object is created.
        :return: The ICM20948 device object.
        :rtype: Object
    """
    # Constructor
    device_name			= _DEFAULT_NAME
    available_addresses	= _AVAILABLE_I2C_ADDRESS

     # Generalized
    REG_BANK_SEL = 						0x7F

    # Gyroscope and Accelerometer
    # User Bank 0
    AGB0_REG_WHO_AM_I = 				0x00
    AGB0_REG_USER_CTRL = 				0x03
    AGB0_REG_LP_CONFIG = 				0x05
    AGB0_REG_PWR_MGMT_1 = 				0x06
    AGB0_REG_PWR_MGMT_2 = 				0x07
    AGB0_REG_INT_PIN_CONFIG = 			0x0F
    AGB0_REG_INT_ENABLE = 				0x10
    AGB0_REG_INT_ENABLE_1 = 			0x11
    AGB0_REG_INT_ENABLE_2 = 			0x12
    AGB0_REG_INT_ENABLE_3 = 			0x13
    AGB0_REG_I2C_MST_STATUS = 			0x17
    AGB0_REG_INT_STATUS = 				0x19
    AGB0_REG_INT_STATUS_1 = 			0x1A
    AGB0_REG_INT_STATUS_2 = 			0x1B
    AGB0_REG_INT_STATUS_3 = 			0x1C
    AGB0_REG_DELAY_TIMEH = 				0x28
    AGB0_REG_DELAY_TIMEL = 				0x29
    AGB0_REG_ACCEL_XOUT_H = 			0x2D
    AGB0_REG_ACCEL_XOUT_L = 			0x2E
    AGB0_REG_ACCEL_YOUT_H = 			0x2F
    AGB0_REG_ACCEL_YOUT_L = 			0x30
    AGB0_REG_ACCEL_ZOUT_H = 			0x31
    AGB0_REG_ACCEL_ZOUT_L = 			0x32
    AGB0_REG_GYRO_XOUT_H = 				0x33
    AGB0_REG_GYRO_XOUT_L = 				0x34
    AGB0_REG_GYRO_YOUT_H = 				0x35
    AGB0_REG_GYRO_YOUT_L = 				0x36
    AGB0_REG_GYRO_ZOUT_H = 				0x37
    AGB0_REG_GYRO_ZOUT_L = 				0x38
    AGB0_REG_TEMP_OUT_H  = 				0x39
    AGB0_REG_TEMP_OUT_L = 				0x3A
    AGB0_REG_EXT_SLV_SENS_DATA_00 =		0x3B
    AGB0_REG_EXT_SLV_SENS_DATA_01 = 	0x3C
    AGB0_REG_EXT_SLV_SENS_DATA_02 = 	0x3D
    AGB0_REG_EXT_SLV_SENS_DATA_03 = 	0x3E
    AGB0_REG_EXT_SLV_SENS_DATA_04 = 	0x3F
    AGB0_REG_EXT_SLV_SENS_DATA_05 = 	0x40
    AGB0_REG_EXT_SLV_SENS_DATA_06 = 	0x41
    AGB0_REG_EXT_SLV_SENS_DATA_07 = 	0x42
    AGB0_REG_EXT_SLV_SENS_DATA_08 = 	0x43
    AGB0_REG_EXT_SLV_SENS_DATA_09 = 	0x44
    AGB0_REG_EXT_SLV_SENS_DATA_10 = 	0x45
    AGB0_REG_EXT_SLV_SENS_DATA_11 = 	0x46
    AGB0_REG_EXT_SLV_SENS_DATA_12 = 	0x47
    AGB0_REG_EXT_SLV_SENS_DATA_13 = 	0x48
    AGB0_REG_EXT_SLV_SENS_DATA_14 = 	0x49
    AGB0_REG_EXT_SLV_SENS_DATA_15 = 	0x4A
    AGB0_REG_EXT_SLV_SENS_DATA_16 = 	0x4B
    AGB0_REG_EXT_SLV_SENS_DATA_17 = 	0x4C
    AGB0_REG_EXT_SLV_SENS_DATA_18 = 	0x4D
    AGB0_REG_EXT_SLV_SENS_DATA_19 = 	0x4E
    AGB0_REG_EXT_SLV_SENS_DATA_20 = 	0x4F
    AGB0_REG_EXT_SLV_SENS_DATA_21 = 	0x50
    AGB0_REG_EXT_SLV_SENS_DATA_22 = 	0x51
    AGB0_REG_EXT_SLV_SENS_DATA_23 = 	0x52
    AGB0_REG_FIFO_EN_1 = 				0x66
    AGB0_REG_FIFO_EN_2 = 				0x67
    AGB0_REG_FIFO_RST =                 0x68
    AGB0_REG_FIFO_MODE = 				0x69
    AGB0_REG_FIFO_COUNT_H = 			0x70
    AGB0_REG_FIFO_COUNT_L = 			0x71
    AGB0_REG_FIFO_R_W = 				0x72
    AGB0_REG_DATA_RDY_STATUS = 			0x74
    AGB0_REG_FIFO_CFG = 				0x76
    AGB0_REG_MEM_START_ADDR = 			0x7C # Hmm  Invensense thought they were sneaky not listing these locations on the datasheet...
    AGB0_REG_MEM_R_W = 					0x7D # These three locations seem to be able to access some memory within the device
    AGB0_REG_MEM_BANK_SEL = 			0x7E # And that location is also where the DMP image gets loaded
    AGB0_REG_REG_BANK_SEL = 			0x7F
    # Bank 1
    AGB1_REG_SELF_TEST_X_GYRO = 		0x02
    AGB1_REG_SELF_TEST_Y_GYRO = 		0x03
    AGB1_REG_SELF_TEST_Z_GYRO = 		0x04
    AGB1_REG_SELF_TEST_X_ACCEL = 		0x0E
    AGB1_REG_SELF_TEST_Y_ACCEL = 		0x0F
    AGB1_REG_SELF_TEST_Z_ACCEL = 		0x10
    AGB1_REG_XA_OFFS_H = 				0x14
    AGB1_REG_XA_OFFS_L = 				0x15
    AGB1_REG_YA_OFFS_H = 				0x17
    AGB1_REG_YA_OFFS_L = 				0x18
    AGB1_REG_ZA_OFFS_H = 				0x1A
    AGB1_REG_ZA_OFFS_L = 				0x1B
    AGB1_REG_TIMEBASE_CORRECTION_PLL = 	0x28
    AGB1_REG_REG_BANK_SEL = 			0x7F
    # Bank 2
    AGB2_REG_GYRO_SMPLRT_DIV = 			0x00
    AGB2_REG_GYRO_CONFIG_1 = 			0x01
    AGB2_REG_GYRO_CONFIG_2 = 			0x02
    AGB2_REG_XG_OFFS_USRH = 			0x03
    AGB2_REG_XG_OFFS_USRL = 			0x04
    AGB2_REG_YG_OFFS_USRH = 			0x05
    AGB2_REG_YG_OFFS_USRL = 			0x06
    AGB2_REG_ZG_OFFS_USRH = 			0x07
    AGB2_REG_ZG_OFFS_USRL = 			0x08
    AGB2_REG_ODR_ALIGN_EN = 			0x09
    AGB2_REG_ACCEL_SMPLRT_DIV_1 = 		0x10
    AGB2_REG_ACCEL_SMPLRT_DIV_2 = 		0x11
    AGB2_REG_ACCEL_INTEL_CTRL = 		0x12
    AGB2_REG_ACCEL_WOM_THR = 			0x13
    AGB2_REG_ACCEL_CONFIG_1 = 			0x14
    AGB2_REG_ACCEL_CONFIG_2 = 			0x15
    AGB2_REG_PRGM_START_ADDRH =         0x50
    AGB2_REG_PRGM_START_ADDRL =         0x51
    AGB2_REG_FSYNC_CONFIG = 			0x52
    AGB2_REG_TEMP_CONFIG = 				0x53
    AGB2_REG_MOD_CTRL_USR = 			0x54
    AGB2_REG_REG_BANK_SEL = 			0x7F
    # Bank 3
    AGB3_REG_I2C_MST_ODR_CONFIG = 		0x00
    AGB3_REG_I2C_MST_CTRL = 			0x01
    AGB3_REG_I2C_MST_DELAY_CTRL = 		0x02
    AGB3_REG_I2C_SLV0_ADDR = 			0x03
    AGB3_REG_I2C_SLV0_REG = 			0x04
    AGB3_REG_I2C_SLV0_CTRL = 			0x05
    AGB3_REG_I2C_SLV0_DO = 				0x06
    AGB3_REG_I2C_SLV1_ADDR = 			0x07
    AGB3_REG_I2C_SLV1_REG = 			0x08
    AGB3_REG_I2C_SLV1_CTRL = 			0x09
    AGB3_REG_I2C_SLV1_DO = 				0x0A
    AGB3_REG_I2C_SLV2_ADDR = 			0x0B
    AGB3_REG_I2C_SLV2_REG = 			0x0C
    AGB3_REG_I2C_SLV2_CTRL = 			0x0D
    AGB3_REG_I2C_SLV2_DO = 				0x0E
    AGB3_REG_I2C_SLV3_ADDR = 			0x0F
    AGB3_REG_I2C_SLV3_REG = 			0x10
    AGB3_REG_I2C_SLV3_CTRL = 			0x11
    AGB3_REG_I2C_SLV3_DO = 				0x12
    AGB3_REG_I2C_SLV4_ADDR = 			0x13
    AGB3_REG_I2C_SLV4_REG = 			0x14
    AGB3_REG_I2C_SLV4_CTRL = 			0x15
    AGB3_REG_I2C_SLV4_DO = 				0x16
    AGB3_REG_I2C_SLV4_DI = 				0x17
    AGB3_REG_REG_BANK_SEL = 			0x7F

    # Magnetometer
    M_REG_WIA2 = 						0x01
    M_REG_ST1 = 						0x10
    M_REG_HXL = 						0x11
    M_REG_HXH = 						0x12
    M_REG_HYL = 						0x13
    M_REG_HYH = 						0x14
    M_REG_HZL = 						0x15
    M_REG_HZH = 						0x16
    M_REG_ST2 = 						0x18
    M_REG_CNTL2 = 						0x31
    M_REG_CNTL3 = 						0x32
    M_REG_TS1 = 						0x33
    M_REG_TS2 = 						0x34

    # Constructor
    def __init__(self, address=None, i2c_driver=None):
        # Did the user specify an I2C address?
        self.address = address if address != None else self.available_addresses[0]

        # load the I2C driver if one isn't provided

        if i2c_driver == None:
            self._i2c = qwiic_i2c.getI2CDriver()
            if self._i2c == None:
                print("Unable to load I2C driver for this platform.")
                return
        else:
            self._i2c = i2c_driver

    # ----------------------------------
    # isConnected()
    #
    # Is an actual board connected to our system?

    def isConnected(self):
        """
            Determine if a ICM20948 device is conntected to the system..

            :return: True if the device is connected, otherwise False.
            :rtype: bool

        """
        return qwiic_i2c.isDeviceConnected(self.address)

    connected = property(isConnected)

    def writeByte(self, reg, val):
        return self._i2c.writeByte(self.address, reg, val)

    def write(self, reg, data):
        return self._i2c.writeBlock(self.address, reg, data)

    def readByte(self, reg):
        return self._i2c.readByte(self.address, reg)

    def read(self, reg, len):
        return self._i2c.readBlock(self.address, reg, len)

    # ----------------------------------
    # setBank()
    #
    # Sets the bank register of the ICM20948 module
    def setBank(self, bank):
        """
            Sets the bank register of the ICM20948 module

            :return: Returns true if the bank was a valid value and it was set, otherwise False.
            :rtype: bool

        """
        if bank > 3:	# Only 4 possible banks
            print("Invalid Bank value: %d" % bank)
            return False
        bank = ((bank << 4) & 0x30) # bits 5:4 of REG_BANK_SEL
        #return ICM_20948_execute_w(pdev, REG_BANK_SEL, &bank, 1)
        return self.writeByte(self.REG_BANK_SEL, bank)

    # ----------------------------------
    # swReset()
    #
    # Performs a software reset on the ICM20948 module
    def swReset(self):
        """
            Performs a software reset on the ICM20948 module

            :return: Returns true if the software reset was successful, otherwise False.
            :rtype: bool

        """
        # Read the Power Management Register, store in local variable "register"
        self.setBank(0)
        register = self.readByte(self.AGB0_REG_PWR_MGMT_1)

        # Set the device reset bit [7]
        register |= (1<<7)

        # Write register
        self.setBank(0)
        return self.writeByte(self.AGB0_REG_PWR_MGMT_1, register)

    # ----------------------------------
    # sleep()
    #
    # Sets the ICM20948 module in or out of sleep mode
    def sleep(self, on):
        """
            Sets the ICM20948 module in or out of sleep mode

            :return: Returns true if the sleep setting write was successful, otherwise False.
            :rtype: bool

        """
        # Read the Power Management Register, store in local variable "register"
        self.setBank(0)
        register = self.readByte(self.AGB0_REG_PWR_MGMT_1)

        # Set/clear the sleep bit [6] as needed
        if on:
            register |= (1<<6) # set bit
        else:
            register &= ~(1<<6) # clear bit

        # Write register
        self.setBank(0)
        return self.writeByte(self.AGB0_REG_PWR_MGMT_1, register)

    # ----------------------------------
    # lowPower()
    #
    # Sets the ICM20948 module in or out of low power mode
    def lowPower(self, on):
        """
            Sets the ICM20948 module in or out of low power mode

            :return: Returns true if the power mode setting write was successful, otherwise False.
            :rtype: bool

        """
        # Read the Power Management Register, store in local variable "register"
        self.setBank(0)
        register = self.readByte(self.AGB0_REG_PWR_MGMT_1)

        # Set/clear the low power mode bit [5] as needed
        if on:
            register |= (1<<5) # set bit
        else:
            register &= ~(1<<5) # clear bit

        # Write register
        self.setBank(0)
        return self.writeByte(self.AGB0_REG_PWR_MGMT_1, register)

    # ----------------------------------
    # setSampleMode()
    #
    # Sets the sample mode of the ICM90248 module
    def setSampleMode(self, sensors, mode):
        """
            Sets the sample mode of the ICM90248 module

            :return: Returns true if the sample mode setting write was successful, otherwise False.
            :rtype: bool

        """
        # check for valid sensor ID from user of this function
        if ((sensors & (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr | ICM_20948_Internal_Mst)) == False):
            print("Invalid Sensor ID")
            return False

        # Read the LP CONFIG Register, store in local variable "register"
        self.setBank(0)
        register = self.readByte(self.AGB0_REG_LP_CONFIG)

        if (sensors & ICM_20948_Internal_Acc):
            # Set/clear the sensor specific sample mode bit as needed
            if mode == ICM_20948_Sample_Mode_Cycled:
                register |= (1<<5) # set bit
            elif mode == ICM_20948_Sample_Mode_Continuous:
                register &= ~(1<<5) # clear bit

        if (sensors & ICM_20948_Internal_Gyr):
            # Set/clear the sensor specific sample mode bit as needed
            if mode == ICM_20948_Sample_Mode_Cycled:
                register |= (1<<4) # set bit
            elif mode == ICM_20948_Sample_Mode_Continuous:
                register &= ~(1<<4) # clear bit

        if (sensors & ICM_20948_Internal_Mst):
            # Set/clear the sensor specific sample mode bit as needed
            if mode == ICM_20948_Sample_Mode_Cycled:
                register |= (1<<6) # set bit
            elif mode == ICM_20948_Sample_Mode_Continuous:
                register &= ~(1<<6) # clear bit

        # Write register
        self.setBank(0)
        return self.writeByte(self.AGB0_REG_LP_CONFIG, register)

    def setSampleRateGyr(self, rate):
        self.setBank(2)
        self.writeByte(self.AGB2_REG_GYRO_SMPLRT_DIV, rate)

    def setSampleRateAcc(self, rate):
        self.setBank(2)
        div1 = rate << 8
        div2 = rate & 0xFF
        self.writeByte(self.AGB2_REG_ACCEL_SMPLRT_DIV_1, div1)
        self.writeByte(self.AGB2_REG_ACCEL_SMPLRT_DIV_2, div2)

    # ----------------------------------
    # setFullScaleRangeAccel()
    #
    # Sets the full scale range for the accel in the ICM20948 module
    def setFullScaleRangeAccel(self, mode):
        """
            Sets the full scale range for the accel in the ICM20948 module

            :return: Returns true if the full scale range setting write was successful, otherwise False.
            :rtype: bool

        """
        # Read the Accel Config Register, store in local variable "register"
        self.setBank(2)
        register = self.readByte(self.AGB2_REG_ACCEL_CONFIG_1)

        register &= ~(0b00000110) # clear bits 2:1 (0b0000.0XX0)

        register |= (mode << 1) # place mode select into bits 2:1 of AGB2_REG_ACCEL_CONFIG

        # Write register
        self.setBank(2)
        return self.writeByte(self.AGB2_REG_ACCEL_CONFIG_1, register)

    # ----------------------------------
    # setFullScaleRangeGyro()
    #
    # Sets the full scale range for the gyro in the ICM20948 module
    def setFullScaleRangeGyro(self, mode):
        """
            Sets the full scale range for the gyro in the ICM20948 module

            :return: Returns true if the full scale range setting write was successful, otherwise False.
            :rtype: bool

        """
        # Read the Gyro Config Register, store in local variable "register"
        self.setBank(2)
        register = self.readByte(self.AGB2_REG_GYRO_CONFIG_1)

        register &= ~(0b00000110) # clear bits 2:1 (0b0000.0XX0)

        register |= (mode << 1) # place mode select into bits 2:1 of AGB2_REG_GYRO_CONFIG_1

        # Write register
        self.setBank(2)
        return self.writeByte(self.AGB2_REG_GYRO_CONFIG_1, register)

    # ----------------------------------
    # setDLPFcfgAccel()
    #
    # Sets the digital low pass filter for the accel in the ICM20948 module
    def setDLPFcfgAccel(self, dlpcfg):
        """
            Sets the digital low pass filter for the accel in the ICM20948 module

            :return: Returns true if the dlp setting write was successful, otherwise False.
            :rtype: bool

        """
        # Read the Accel Config Register, store in local variable "register"
        self.setBank(2)
        register = self.readByte(self.AGB2_REG_ACCEL_CONFIG_1)

        register &= ~(0b00111000) # clear bits 5:3 (0b00XX.X000)

        register |= (dlpcfg << 3) # place dlpcfg select into bits 5:3 of AGB2_REG_ACCEL_CONFIG_1

        # Write register
        self.setBank(2)
        return self.writeByte(self.AGB2_REG_ACCEL_CONFIG_1, register)

    # ----------------------------------
    # setDLPFcfgGyro()
    #
    # Sets the digital low pass filter for the gyro in the ICM20948 module
    def setDLPFcfgGyro(self, dlpcfg):
        """
            Sets the digital low pass filter for the gyro in the ICM20948 module

            :return: Returns true if the dlp setting write was successful, otherwise False.
            :rtype: bool

        """
        # Read the gyro Config Register, store in local variable "register"
        self.setBank(2)
        register = self.readByte(self.AGB2_REG_GYRO_CONFIG_1)

        register &= ~(0b00111000) # clear bits 5:3 (0b00XX.X000)

        register |= (dlpcfg << 3) # place dlpcfg select into bits 5:3 of AGB2_REG_GYRO_CONFIG_1

        # Write register
        self.setBank(2)
        return self.writeByte(self.AGB2_REG_GYRO_CONFIG_1, register)

    # ----------------------------------
    # enableDlpfAccel()
    #
    # Enables or disables the accelerometer DLPF of the ICM90248 module
    def enableDlpfAccel(self, on):
        """
            Enables or disables the accelerometer DLPF of the ICM90248 module

            :return: Returns true if the DLPF mode setting write was successful, otherwise False.
            :rtype: bool

        """

        # Read the AGB2_REG_ACCEL_CONFIG_1, store in local variable "register"
        self.setBank(2)
        register = self.readByte(self.AGB2_REG_ACCEL_CONFIG_1)

        # Set/clear the ACCEL_FCHOICE bit [0] as needed
        if on:
            register |= (1<<0) # set bit
        else:
            register &= ~(1<<0) # clear bit

        # Write register
        self.setBank(2)
        return self.writeByte(self.AGB2_REG_ACCEL_CONFIG_1, register)

    # ----------------------------------
    # enableDlpfGyro()
    #
    # Enables or disables the Gyro DLPF of the ICM90248 module
    def enableDlpfGyro(self, on):
        """
            Enables or disables the Gyro DLPF of the ICM90248 module

            :return: Returns true if the DLPF mode setting write was successful, otherwise False.
            :rtype: bool

        """

        # Read the AGB2_REG_GYRO_CONFIG_1, store in local variable "register"
        self.setBank(2)
        register = self.readByte(self.AGB2_REG_GYRO_CONFIG_1)

        # Set/clear the GYRO_FCHOICE bit [0] as needed
        if on:
            register |= (1<<0) # set bit
        else:
            register &= ~(1<<0) # clear bit

        # Write register
        self.setBank(2)
        return self.writeByte(self.AGB2_REG_GYRO_CONFIG_1, register)

    # ----------------------------------
    # dataReady()
    #
    # Returns status of RAW_DATA_0_RDY_INT the ICM90248 module
    def dataReady(self):
        """
            Returns status of RAW_DATA_0_RDY_INT the ICM90248 module

            :return: Returns true if raw data is ready, otherwise False.
            :rtype: bool

        """

        # Read the AGB0_REG_INT_STATUS_1, store in local variable "register"
        self.setBank(0)
        register = self.readByte(self.AGB0_REG_INT_STATUS_1)

        # check bit [0]
        if (register & (1<<0)):
            return True
        else:
            return False

    # ----------------------------------
    # ToSignedInt()
    #
    # Takes an input data of 16 bits, and returns the signed 32 bit int version of this data
    def ToSignedInt(self, input):
        """
            Takes an input data of 16 bits, and returns the signed 32 bit int version of this data

            :return: Signed 32 bit integer
            :rtype: int

        """
        if input > 32767:
            input -= 65536
        return input

    # ----------------------------------
    # getAgmt()
    #
    # Reads and updates raw values from accel, gyro, mag and temp of the ICM90248 module
    def getAgmt(self):
        """
            Reads and updates raw values from accel, gyro, mag and temp of the ICM90248 module

            :return: Returns True if I2C readBlock was successful, otherwise False.
            :rtype: bool

        """

        # Read all of the readings starting at AGB0_REG_ACCEL_XOUT_H
        numbytes = 14 + 9 # Read Accel, gyro, temp, and 9 bytes of mag
        self.setBank(0)
        buff = self._i2c.readBlock(self.address, self.AGB0_REG_ACCEL_XOUT_H, numbytes)

        self.axRaw = ((buff[0] << 8) | (buff[1] & 0xFF))
        self.ayRaw = ((buff[2] << 8) | (buff[3] & 0xFF))
        self.azRaw = ((buff[4] << 8) | (buff[5] & 0xFF))

        self.gxRaw = ((buff[6] << 8) | (buff[7] & 0xFF))
        self.gyRaw = ((buff[8] << 8) | (buff[9] & 0xFF))
        self.gzRaw = ((buff[10] << 8) | (buff[11] & 0xFF))

        self.tmpRaw = ((buff[12] << 8) | (buff[13] & 0xFF))

        self.magStat1 = buff[14]
        self.mxRaw = ((buff[16] << 8) | (buff[15] & 0xFF)) # Mag data is read little endian
        self.myRaw = ((buff[18] << 8) | (buff[17] & 0xFF))
        self.mzRaw = ((buff[20] << 8) | (buff[19] & 0xFF))
        self.magStat2 = buff[22]

        # Convert all values to signed (because python treats all ints as 32 bit ints
        # and does not see the MSB as the sign of our 16 bit int raw value)
        self.axRaw = self.ToSignedInt(self.axRaw)
        self.ayRaw = self.ToSignedInt(self.ayRaw)
        self.azRaw = self.ToSignedInt(self.azRaw)

        self.gxRaw = self.ToSignedInt(self.gxRaw)
        self.gyRaw = self.ToSignedInt(self.gyRaw)
        self.gzRaw = self.ToSignedInt(self.gzRaw)

        self.mxRaw = self.ToSignedInt(self.mxRaw)
        self.myRaw = self.ToSignedInt(self.myRaw)
        self.mzRaw = self.ToSignedInt(self.mzRaw)

        # check for data read error
        if buff:
            return True
        else:
            return False


    # ----------------------------------
    # i2cMasterPassthrough()
    #
    # Enables or disables I2C Master Passthrough
    def i2cMasterPassthrough(self, passthrough):
        """
            Enables or disables I2C Master Passthrough

            :return: Returns true if the setting write was successful, otherwise False.
            :rtype: bool

        """

        # Read the AGB0_REG_INT_PIN_CONFIG, store in local variable "register"
        self.setBank(0)
        register = self.readByte(self.AGB0_REG_INT_PIN_CONFIG)

        # Set/clear the BYPASS_EN bit [1] as needed
        if passthrough:
            register |= (1<<1) # set bit
        else:
            register &= ~(1<<1) # clear bit

        # Write register
        self.setBank(0)
        return self.writeByte(self.AGB0_REG_INT_PIN_CONFIG, register)

    # ----------------------------------
    # i2cMasterEnable()
    #
    # Enables or disables I2C Master
    def i2cMasterEnable(self, enable):
        """
            Enables or disables I2C Master

            :return: Returns true if the setting write was successful, otherwise False.
            :rtype: bool

        """

        self.i2cMasterPassthrough(False) # Disable BYPASS_EN

        # Setup Master Clock speed as 345.6 kHz, and NSP (aka next slave read) to "stop between reads"
        # Read the AGB3_REG_I2C_MST_CTRL, store in local variable "register"
        self.setBank(3)
        register = self.readByte(self.AGB3_REG_I2C_MST_CTRL)

        register &= ~(0x0F) # clear bits for master clock [3:0]
        register |= (0x07) # set bits for master clock [3:0], 0x07 corresponds to 345.6 kHz, good for up to 400 kHz
        register |= (1<<4) # set bit [4] for NSR (next slave read). 0 = restart between reads. 1 = stop between reads.

        # Write register
        self.setBank(3)
        self.writeByte(self.AGB3_REG_I2C_MST_CTRL, register)

        # enable/disable Master I2C
        # Read the AGB0_REG_USER_CTRL, store in local variable "register"
        self.setBank(0)
        register = self.readByte(self.AGB0_REG_USER_CTRL)

        # Set/clear the I2C_MST_EN bit [5] as needed
        if enable:
            register |= (1<<5) # set bit
        else:
            register &= ~(1<<5) # clear bit

        # Write register
        self.setBank(0)
        return self.writeByte(self.AGB0_REG_USER_CTRL, register)

    # Transact directly with an I2C device, one byte at a time
    # Used to configure a device before it is setup into a normal 0-3 slave slot
    def ICM_20948_i2c_master_slv4_txn(self, addr, reg, data, Rw, send_reg_addr):
        # Thanks MikeFair! // https://github.com/kriswiner/MPU9250/issues/86

        if Rw:
            addr |= 0x80

        self.setBank(3)
        self.writeByte(self.AGB3_REG_I2C_SLV4_ADDR, addr)

        self.setBank(3)
        self.writeByte(self.AGB3_REG_I2C_SLV4_REG, reg)

        ctrl_register_slv4 = 0x00
        ctrl_register_slv4 |= (1<<7) # EN bit [7] (set)
        ctrl_register_slv4 &= ~(1<<6) # INT_EN bit [6] (cleared)
        ctrl_register_slv4 &= ~(0x0F) # DLY bits [4:0] (cleared = 0)
        if(send_reg_addr):
            ctrl_register_slv4 &= ~(1<<5) # REG_DIS bit [5] (cleared)
        else:
            ctrl_register_slv4 |= (1<<5) # REG_DIS bit [5] (set)

        txn_failed = False

        if (Rw == False):
            self.setBank(3)
            self.writeByte(self.AGB3_REG_I2C_SLV4_DO, data)

        # Kick off txn
        self.setBank(3)
        self.writeByte(self.AGB3_REG_I2C_SLV4_CTRL, ctrl_register_slv4)

        max_cycles = 1000
        count = 0
        slave4Done = False
        while (slave4Done == False):
            self.setBank(0)
            i2c_mst_status = self.readByte(self.AGB0_REG_I2C_MST_STATUS)
            if i2c_mst_status & (1<<6): # Check I2C_SLAVE_DONE bit [6]
                slave4Done = True
            if  count > max_cycles:
                slave4Done = True
            count += 1

        if i2c_mst_status & (1<<4): # Check I2C_SLV4_NACK bit [4]
            txn_failed = True

        if count > max_cycles:
            txn_failed = True

        if txn_failed:
            return False

        if Rw:
            self.setBank(3)
            return self.readByte(self.AGB3_REG_I2C_SLV4_DI)

        return True # if we get here, then it was a successful write

    def i2cMasterSingleW(self, addr, reg, data):
        data1 = self.ICM_20948_i2c_master_slv4_txn(addr, reg, data, False, True)
        return data1

    def writeMag(self, reg, data):
        data = self.i2cMasterSingleW(MAG_AK09916_I2C_ADDR, reg, data)
        return data

    def i2cMasterSingleR(self, addr, reg):
        data = self.ICM_20948_i2c_master_slv4_txn(addr, reg, 0, True, True)
        return data

    def readMag(self, reg):
        data = self.i2cMasterSingleR(MAG_AK09916_I2C_ADDR, reg)
        return data

    # ----------------------------------
    # magWhoIAm()
    #
    # Checks to see that the Magnetometer returns the correct ID value
    def magWhoIAm(self):
        """
            Checks to see that the Magnatometer returns the correct ID value

            :return: Returns true if the check was successful, otherwise False.
            :rtype: bool

        """

        whoiam1 = self.readMag(AK09916_REG_WIA1)
        whoiam2 = self.readMag(AK09916_REG_WIA2)

        if ((whoiam1 == (MAG_AK09916_WHO_AM_I >> 8)) and (whoiam2 == (MAG_AK09916_WHO_AM_I & 0xFF))):
            return True
        else:
            return False

    # ----------------------------------
    # i2cMasterReset()
    #
    # Resets I2C Master Module
    def i2cMasterReset(self):
        """
            Resets I2C Master Module

            :return: Returns true if the i2c write was successful, otherwise False.
            :rtype: bool

        """

        # Read the AGB0_REG_USER_CTRL, store in local variable "register"
        self.setBank(0)
        register = self.readByte(self.AGB0_REG_USER_CTRL)

        # Set the I2C_MST_RST bit [1]
        register |= (1<<1) # set bit

        # Write register
        self.setBank(0)
        return self.writeByte(self.AGB0_REG_USER_CTRL, register)

    # ----------------------------------
    # ICM_20948_i2c_master_configure_slave()
    #
    # Configures Master/slave settings for the ICM20948 as master, and slave in slots 0-3
    def i2cMasterConfigureSlave(self, slave, addr, reg, len, Rw, enable, data_only, grp, swap):
        """
            Configures Master/slave settings for the ICM20948 as master, and slave in slots 0-3

            :return: Returns true if the configuration was successful, otherwise False.
            :rtype: bool

        """
        # Adjust slave address, reg (aka sub-address), and control as needed for each slave slot (0-3)
        slv_addr_reg = 0x00
        slv_reg_reg = 0x00
        slv_ctrl_reg = 0x00
        if slave == 0:
            slv_addr_reg = self.AGB3_REG_I2C_SLV0_ADDR
            slv_reg_reg = self.AGB3_REG_I2C_SLV0_REG
            slv_ctrl_reg = self.AGB3_REG_I2C_SLV0_CTRL
        elif slave == 1:
            slv_addr_reg = self.AGB3_REG_I2C_SLV1_ADDR
            slv_reg_reg = self.AGB3_REG_I2C_SLV1_REG
            slv_ctrl_reg = self.AGB3_REG_I2C_SLV1_CTRL
        elif slave == 2:
            slv_addr_reg = self.AGB3_REG_I2C_SLV2_ADDR
            slv_reg_reg = self.AGB3_REG_I2C_SLV2_REG
            slv_ctrl_reg = self.AGB3_REG_I2C_SLV2_CTRL
        elif slave == 3:
            slv_addr_reg = self.AGB3_REG_I2C_SLV3_ADDR
            slv_reg_reg = self.AGB3_REG_I2C_SLV3_REG
            slv_ctrl_reg = self.AGB3_REG_I2C_SLV3_CTRL
        else:
            return False

        self.setBank(3)

        # Set the slave address and the Rw flag
        address = addr
        if Rw:
            address |= (1<<7) # set bit# set RNW bit [7]

        self.writeByte(slv_addr_reg, address)

        # Set the slave sub-address (reg)
        subAddress = reg
        self.writeByte(slv_reg_reg, subAddress)

        # Set up the control info
        ctrl_reg_slvX = 0x00
        ctrl_reg_slvX |= len
        ctrl_reg_slvX |= (enable << 7)
        ctrl_reg_slvX |= (swap << 6)
        ctrl_reg_slvX |= (data_only << 5)
        ctrl_reg_slvX |= (grp << 4)
        return self.writeByte(slv_ctrl_reg, ctrl_reg_slvX)

    def i2cControllerConfigurePeripheral(self, peripheral, addr, reg, len, Rw, enable, data_only, grp, swap, dataOut=None):
        # Adjust slave address, reg (aka sub-address), and control as needed for each slave slot (0-3)
        periph_addr_reg = 0x00
        periph_reg_reg = 0x00
        periph_ctrl_reg = 0x00
        periph_do_reg = 0x00

        if peripheral == 0:
            periph_addr_reg = self.AGB3_REG_I2C_SLV0_ADDR
            periph_reg_reg = self.AGB3_REG_I2C_SLV0_REG
            periph_ctrl_reg = self.AGB3_REG_I2C_SLV0_CTRL
            periph_do_reg = self.AGB3_REG_I2C_SLV0_DO
        elif peripheral == 1:
            periph_addr_reg = self.AGB3_REG_I2C_SLV1_ADDR
            periph_reg_reg = self.AGB3_REG_I2C_SLV1_REG
            periph_ctrl_reg = self.AGB3_REG_I2C_SLV1_CTRL
            periph_do_reg = self.AGB3_REG_I2C_SLV1_DO
        elif peripheral == 2:
            periph_addr_reg = self.AGB3_REG_I2C_SLV2_ADDR
            periph_reg_reg = self.AGB3_REG_I2C_SLV2_REG
            periph_ctrl_reg = self.AGB3_REG_I2C_SLV2_CTRL
            periph_do_reg = self.AGB3_REG_I2C_SLV2_DO
        elif peripheral == 3:
            periph_addr_reg = self.AGB3_REG_I2C_SLV3_ADDR
            periph_reg_reg = self.AGB3_REG_I2C_SLV3_REG
            periph_ctrl_reg = self.AGB3_REG_I2C_SLV3_CTRL
            periph_do_reg = self.AGB3_REG_I2C_SLV3_DO
        else:
            return False

        self.setBank(3)

        # Set the slave address and the Rw flag
        address = addr
        if Rw:
            address |= (1<<7) # set bit# set RNW bit [7]

        self.writeByte(periph_addr_reg, address)

        # If we are setting up a write, configure the Data Out register too
        if not Rw and dataOut:
            self.writeByte(periph_do_reg, dataOut)

        # Set the slave sub-address (reg)
        subAddress = reg
        self.writeByte(periph_reg_reg, subAddress)

        # Set up the control info
        ctrl_reg_slvX = 0x00
        ctrl_reg_slvX |= len
        ctrl_reg_slvX |= (enable << 7)
        ctrl_reg_slvX |= (swap << 6)
        ctrl_reg_slvX |= (data_only << 5)
        ctrl_reg_slvX |= (grp << 4)
        return self.writeByte(periph_ctrl_reg, ctrl_reg_slvX)

    # ----------------------------------
    # startupMagnetometer()
    #
    # Initialize the magnotometer with default values
    def startupMagnetometer(self):
        """
            Initialize the magnotometer with default values

            :return: Returns true of the initializtion was successful, otherwise False.
            :rtype: bool

        """
        self.i2cMasterPassthrough(False) #Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
        self.i2cMasterEnable(True)

        # After a ICM reset the Mag sensor may stop responding over the I2C master
        # Reset the Master I2C until it responds
        tries = 0
        maxTries = 5
        while (tries < maxTries):
            # See if we can read the WhoIAm register correctly
            if (self.magWhoIAm()):
                break # WIA matched!
            self.i2cMasterReset() # Otherwise, reset the master I2C and try again
            tries += 1

        if (tries == maxTries):
            print("Mag ID fail. Tries: %d\n", tries)
            return False

        #Set up magnetometer
        mag_reg_ctrl2 = 0x00
        mag_reg_ctrl2 |= AK09916_mode_cont_100hz
        self.writeMag(AK09916_REG_CNTL2, mag_reg_ctrl2)

        return self.i2cMasterConfigureSlave(0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, True, True, False, False, False)

    # ----------------------------------
    # begin()
    #
    # Initialize the system/validate the board.
    def begin(self, dmp=False):
        """
            Initialize the operation of the ICM20948 module

            :return: Returns true of the initializtion was successful, otherwise False.
            :rtype: bool

        """
        # are we who we need to be?
        self.setBank(0)
        chipID = self.readByte(self.AGB0_REG_WHO_AM_I)
        if not chipID in _validChipIDs:
            print("Invalid Chip ID: 0x%.2X" % chipID)
            return False

        # software reset
        self.swReset()
        time.sleep(.05)

        # set sleep mode off
        self.sleep(False)

        # set lower power mode off
        self.lowPower(False)

        # set sample mode to continuous for both accel and gyro
        self.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous)

        # set full scale range for both accel and gryo (separate functions)
        self.setFullScaleRangeAccel(gpm2)
        self.setFullScaleRangeGyro(dps250)

        # set low pass filter for both accel and gyro (separate functions)
        self.setDLPFcfgAccel(acc_d473bw_n499bw)
        self.setDLPFcfgGyro(gyr_d361bw4_n376bw5)

        # disable digital low pass filters on both accel and gyro
        self.enableDlpfAccel(False)
        self.enableDlpfGyro(False)

        self.startupMagnetometer()

        if dmp:
            self.initializeDMP()

        return True

    def enableFIFO(self, enable):
        self.setBank(0)
        ctrl = self.readByte(self.AGB0_REG_USER_CTRL)
        if enable:
            ctrl = ctrl | 0x40
        else:
            ctrl = ctrl & 0xBF
        self.writeByte(self.AGB0_REG_USER_CTRL, ctrl)

    def resetFIFO(self):
        self.setBank(0)
        ctrl = self.readByte(self.AGB0_REG_FIFO_RST) & 0xE0
        ctrl = ctrl | 0x1F
        self.writeByte(self.AGB0_REG_FIFO_RST, ctrl)
        # The InvenSense Nucleo examples write 0x1F followed by 0x1E
        ctrl = ctrl & 0xFE
        self.writeByte(self.AGB0_REG_FIFO_RST, ctrl)

    def enableDMP(self, enable):
        self.setBank(0)
        ctrl = self.readByte(self.AGB0_REG_USER_CTRL)
        if enable:
            ctrl = ctrl | 0x80
        else:
            ctrl = ctrl & 0x7F
        self.writeByte(self.AGB0_REG_USER_CTRL, ctrl)

    def loadDMPFirmware(self):
        pass

    def initializeDMP(self):
        # So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
        # 0: use I2C_SLV0
        # MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
        # AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
        # 10: we read 10 bytes each cycle
        # true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
        # true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
        # false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
        # true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
        # true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
        # result = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true);
        if not self.i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, True, True, False, True, True):
            raise Exception('Failed to set up I2C_SLV0 to do the ten byte reading')

        # We also need to set up I2C_SLV1 to do the Single Measurement triggering:
        # 1: use I2C_SLV1
        # MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
        # AK09916_REG_CNTL2: we start writing here (0x31)
        # 1: not sure why, but the write does not happen if this is set to zero
        # false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
        # true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
        # false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
        # false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
        # false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
        # AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample
        # result = i2cControllerConfigurePeripheral(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single);
        if not self.i2cControllerConfigurePeripheral(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, False, True, False, False, False, AK09916_mode_single):
            raise Exception('Failed to set up I2C_SLV1 to do the Single Measurement triggering')

        # Set the I2C Master ODR configuration
        # It is not clear why we need to do this... But it appears to be essential! From the datasheet:
        # "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
        #  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
        #  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
        #  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
        # Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
        # You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
        # result = setBank(3); if (result > worstResult) worstResult = result; // Select Bank 3
        # uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
        # result = write(AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1); if (result > worstResult) worstResult = result; // Write one byte to the I2C_MST_ODR_CONFIG register
        self.setBank(3)
        mstODRconfig = 0x04
        self.writeByte(self.AGB3_REG_I2C_MST_ODR_CONFIG, mstODRconfig)

        #  Configure clock source through PWR_MGMT_1
        # ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
        # result = setClockSource(ICM_20948_Clock_Auto);
        self.setBank(0)
        pwr_mgmt = self.readByte(self.AGB0_REG_PWR_MGMT_1) & 0xF8
        pwr_mgmt = pwr_mgmt | 0x01
        self.writeByte(self.AGB0_REG_PWR_MGMT_1, pwr_mgmt)

        #  Enable accel and gyro sensors through PWR_MGMT_2
        # Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
        # result = setBank(0); if (result > worstResult) worstResult = result;                               // Select Bank 0
        # uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
        # result = write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1); if (result > worstResult) worstResult = result; // Write one byte to the PWR_MGMT_2 register
        self.setBank(0)
        self.writeByte(self.AGB0_REG_PWR_MGMT_2, 0x40)

        #  Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
        #  The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
        # result = setSampleMode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled); if (result > worstResult) worstResult = result;
        self.setSampleMode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled)

        # Disable the FIFO
        # result = enableFIFO(false); if (result > worstResult) worstResult = result;
        self.enableFIFO(False)

        #  Disable the DMP
        # result = enableDMP(false); if (result > worstResult) worstResult = result;
        self.enableDMP(False)

        #  Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
        # Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
        # ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
        # myFSS.a = gpm4;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
        # myFSS.g = dps2000;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
        # result = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS); if (result > worstResult) worstResult = result;
        self.setFullScaleRangeAccel(gpm4)
        self.setFullScaleRangeGyro(dps2000)

        #  The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
        #  We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
        # The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
        # result = enableDLPF(ICM_20948_Internal_Gyr, true); if (result > worstResult) worstResult = result;
        self.enableDlpfGyro(True)

        #  Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
        # Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
        # result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
        # uint8_t zero = 0;
        # result = write(AGB0_REG_FIFO_EN_1, &zero, 1); if (result > worstResult) worstResult = result;
        self.setBank(0)
        self.writeByte(self.AGB0_REG_FIFO_EN_1, 0)

        # Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
        # result = write(AGB0_REG_FIFO_EN_2, &zero, 1); if (result > worstResult) worstResult = result;
        self.writeByte(self.AGB0_REG_FIFO_EN_2, 0)

        # Turn off data ready interrupt through INT_ENABLE_1
        # result = intEnableRawDataReady(false); if (result > worstResult) worstResult = result;
        int_en = self.readByte(self.AGB0_REG_INT_ENABLE_1)
        int_en = int_en & 0xFE
        self.writeByte(self.AGB0_REG_INT_ENABLE_1, int_en)

        # Reset FIFO through FIFO_RST
        # result = resetFIFO(); if (result > worstResult) worstResult = result;
        self.resetFIFO()

        # Set gyro sample rate divider with GYRO_SMPLRT_DIV
        # Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
        # ICM_20948_smplrt_t mySmplrt;
        # mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
        # mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
        #result = setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt); if (result > worstResult) worstResult = result;
        self.setSampleRateGyr(19)
        self.setSampleRateAcc(19)

        #  Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
        # result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS
        dmp_addr = 4096
        self.setBank(2)
        self.writeByte(self.AGB2_REG_PRGM_START_ADDRH, dmp_addr >> 8)
        self.writeByte(self.AGB2_REG_PRGM_START_ADDRL, dmp_addr & 0xFF)

        # // Now load the DMP firmware
        # result = loadDMPFirmware(); if (result > worstResult) worstResult = result;
        self.loadDMPFirmware()

        # // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
        # result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

        # // Set the Hardware Fix Disable register to 0x48
        # result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
        # uint8_t fix = 0x48;
        # result = write(AGB0_REG_HW_FIX_DISABLE, &fix, 1); if (result > worstResult) worstResult = result;

        # // Set the Single FIFO Priority Select register to 0xE4
        # result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
        # uint8_t fifoPrio = 0xE4;
        # result = write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1); if (result > worstResult) worstResult = result;

        # // Configure Accel scaling to DMP
        # // The DMP scales accel raw data internally to align 1g as 2^25
        # // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
        # const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
        # result = writeDMPmems(ACC_SCALE, 4, &accScale[0]); if (result > worstResult) worstResult = result; // Write accScale to ACC_SCALE DMP register
        # // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
        # const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
        # result = writeDMPmems(ACC_SCALE2, 4, &accScale2[0]); if (result > worstResult) worstResult = result; // Write accScale2 to ACC_SCALE2 DMP register

        # // Configure Compass mount matrix and scale to DMP
        # // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
        # // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
        # // Each compass axis will be converted as below:
        # // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
        # // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
        # // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
        # // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
        # // 2^30 / 6.66666 = 161061273 = 0x9999999
        # const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
        # const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
        # const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
        # result = writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;

        # // Configure the B2S Mounting Matrix
        # const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
        # const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
        # result = writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
        # result = writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;

        # // Configure the DMP Gyro Scaling Factor
        # // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
        # //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
        # //            10=102.2727Hz sample rate, ... etc.
        # // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
        # result = setGyroSF(19, 3); if (result > worstResult) worstResult = result; // 19 = 55Hz (see above), 3 = 2000dps (see above)

        # // Configure the Gyro full scale
        # // 2000dps : 2^28
        # // 1000dps : 2^27
        # //  500dps : 2^26
        # //  250dps : 2^25
        # const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
        # result = writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]); if (result > worstResult) worstResult = result;

        # // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
        # const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
        # //const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
        # //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
        # result = writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]); if (result > worstResult) worstResult = result;

        # // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
        # const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
        # //const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
        # //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
        # result = writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]); if (result > worstResult) worstResult = result;

        # // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
        # const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
        # //const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
        # //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
        # result = writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]); if (result > worstResult) worstResult = result;

        # // Configure the Accel Cal Rate
        # const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
        # result = writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]); if (result > worstResult) worstResult = result;

        # // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
        # // Let's set the Compass Time Buffer to 69 (Hz).
        # const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
        # result = writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]); if (result > worstResult) worstResult = result;