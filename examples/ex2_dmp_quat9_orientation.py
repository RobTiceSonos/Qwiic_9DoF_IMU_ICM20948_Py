from __future__ import print_function
import qwiic_icm20948
from dmp_defines import *
import math
import time
import sys

def runExample():

    print("\nSparkFun 9DoF ICM-20948 Sensor DMP Example 1\n")
    IMU = qwiic_icm20948.QwiicIcm20948()

    if IMU.connected == False:
        print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return

    IMU.begin(dmp=True)
    IMU.enableDMPSensor(inv_icm20948_sensor.INV_ICM20948_SENSOR_ORIENTATION)
    IMU.setDMPODRrate(DMP_ODR_REG_QUAT9, 0)
    IMU.enableFIFO()
    IMU.resetDMP()
    IMU.resetFIFO()

    while True:
        try:
            data = IMU.readDMPdataFromFIFO()

            # We have asked for orientation data so we should receive Quat9
            # Scale to +/- 1 and convert to double. Divide by 2^30
            q1 = data['quat9']['Q1'] / 1073741824.0
            q2 = data['quat9']['Q2'] / 1073741824.0
            q3 = data['quat9']['Q3'] / 1073741824.0
            q0 = math.sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)))
            print(f'W:{q0} X:{q1} Y:{q2} Z:{q3} Acc:{data["quat9"]["Accuracy"]}')

        except DMPFIFOUnderflow:
            print('Not enough data in FIFO... continuing')
        finally:
            time.sleep(0.01)

if __name__ == '__main__':
    try:
        runExample()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example 1")
        sys.exit(0)
