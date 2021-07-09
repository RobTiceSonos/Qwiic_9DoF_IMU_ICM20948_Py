from __future__ import print_function

from icm20948 import qwiic_icm20948, dmp

import time
import sys


def runExample():

    SENSOR_LIST = [
        dmp.sensors.Sensor_Types.ACCELEROMETER,
        dmp.sensors.Sensor_Types.GYROSCOPE,
        # dmp.sensors.Sensor_Types.MAGNETIC_FIELD_UNCALIBRATED,
        # dmp.sensors.Sensor_Types.STEP_DETECTOR,
        # dmp.sensors.Sensor_Types.STEP_COUNTER,
        # dmp.sensors.Sensor_Types.GAME_ROTATION_VECTOR,
        # dmp.sensors.Sensor_Types.ROTATION_VECTOR,
        # dmp.sensors.Sensor_Types.GEOMAGNETIC_ROTATION_VECTOR,
        # dmp.sensors.Sensor_Types.GEOMAGNETIC_FIELD,
        # dmp.sensors.Sensor_Types.GRAVITY,
        # dmp.sensors.Sensor_Types.LINEAR_ACCELERATION,
        dmp.sensors.Sensor_Types.ORIENTATION,
    ]

    print("\nSparkFun 9DoF ICM-20948 Sensor DMP Multiple Sensors Example\n")
    IMU = qwiic_icm20948.QwiicIcm20948()

    if IMU.connected == False:
        print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return

    # Initialize the ICM-20948 and Initialize the DMP
    IMU.begin(dmp=True)
    # Enable all of the sensors
    for sensor in dmp.sensors.Sensor_Types:
        IMU.enableDMPSensor(sensor)
    # Configuring DMP to output data at multiple ODRs:
    # DMP is capable of outputting multiple sensor data at different rates to FIFO.
    # Setting value can be calculated as follows:
    # Value = (DMP running rate / ODR ) - 1
    # E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    for reg in dmp.regs.Output_Data_Rate_Control:
        IMU.setDMPODRrate(reg, 10)
    # Enable the FIFO
    IMU.enableFIFO()
    # Enable the DMP
    IMU.enableDMP()
    # Reset DMP
    IMU.resetDMP()
    # Reset FIFO
    IMU.resetFIFO()

    while True:
        # Read any DMP data waiting in the FIFO
        data = IMU.readDMPdataFromFIFO()

        print(data)

        time.sleep(0.08)


if __name__ == '__main__':
    runExample()
