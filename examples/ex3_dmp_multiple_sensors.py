from __future__ import print_function

from icm20948 import qwiic_icm20948, dmp

import json
import time
import sys


SENSOR_LIST = [
    dmp.sensors.Sensor_Types.ACCELEROMETER,
    dmp.sensors.Sensor_Types.GYROSCOPE,
    dmp.sensors.Sensor_Types.MAGNETIC_FIELD_UNCALIBRATED,
    dmp.sensors.Sensor_Types.GEOMAGNETIC_ROTATION_VECTOR,
    dmp.sensors.Sensor_Types.ORIENTATION,
    dmp.sensors.Sensor_Types.GAME_ROTATION_VECTOR,
]


def runExample():

    print("\nSparkFun 9DoF ICM-20948 Sensor DMP Multiple Sensors Example\n")
    IMU = qwiic_icm20948.QwiicIcm20948()

    if IMU.connected == False:
        print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return

    # Initialize the ICM-20948 and Initialize the DMP
    IMU.begin(dmp=True)
    # Enable all of the sensors
    for sensor in SENSOR_LIST:
        IMU.enableDMPSensor(sensor)
    # Configuring DMP to output data at multiple ODRs:
    # DMP is capable of outputting multiple sensor data at different rates to FIFO.
    # Setting value can be calculated as follows:
    # Value = (DMP running rate / ODR ) - 1
    # E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    for reg in dmp.regs.Output_Data_Rate_Control:
        IMU.setDMPODRrate(reg, 0)
    # Enable the FIFO
    IMU.enableFIFO()
    # Enable the DMP
    IMU.enableDMP()
    # Reset DMP
    IMU.resetDMP()
    # Reset FIFO
    IMU.resetFIFO()

    with open('data.json', 'w', encoding='utf-8') as f:
        now = time.time()
        t_end = now + 60 * 2

        while now < t_end:
            try:
                # Read any DMP data waiting in the FIFO
                data = IMU.readDMPdataFromFIFO()
                now = time.time()

            except (dmp.fifo.FifoUnderflow, dmp.fifo.FifoOverflow) as ex:
                print(f'Exception: {type(ex).__name__}. Continuing...')
                continue

            proc_data = IMU.processDMPdata(data)

            if proc_data:
                proc_data.update({'timestamp': now})

                json.dump(proc_data, f, ensure_ascii=False, indent=4)


if __name__ == '__main__':
    runExample()
