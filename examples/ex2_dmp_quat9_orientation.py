from __future__ import print_function
import qwiic_icm20948
from dmp_defines import *

import argparse
import json
import math
import os
import time
import sys

IPC_FIFO_NAME_TX = "quat9_pipe"

parser = argparse.ArgumentParser(description='SparkFun 9DoF ICM-20948 Sensor DMP Quat 9 Example.')
parser.add_argument('-p', '--pipe', action='store_true', default=IPC_FIFO_NAME_TX, help='The name of the IPC pipe to send data to.')
parser.add_argument('-l', '--local', action='store_true', help='If this option is set, no pipe is used and data is just printed to stdout.')

def runExample(tx_pipe=None):

    print("\nSparkFun 9DoF ICM-20948 Sensor DMP Quat 9 Example\n")
    IMU = qwiic_icm20948.QwiicIcm20948()

    if IMU.connected == False:
        print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return

    # Initialize the ICM-20948 and Initialize the DMP
    IMU.begin(dmp=True)
    # Enable the DMP orientation sensor
    IMU.enableDMPSensor(inv_icm20948_sensor.INV_ICM20948_SENSOR_ORIENTATION)
    # Configuring DMP to output data at multiple ODRs:
    # DMP is capable of outputting multiple sensor data at different rates to FIFO.
    # Setting value can be calculated as follows:
    # Value = (DMP running rate / ODR ) - 1
    # E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    IMU.setDMPODRrate(DMP_ODR_REG_QUAT9, 10)
    # Enable the FIFO
    IMU.enableFIFO()
    # Enable the DMP
    IMU.enableDMP()
    # Reset DMP
    IMU.resetDMP()
    # Reset FIFO
    IMU.resetFIFO()

    while True:
        try:
            # Read any DMP data waiting in the FIFO
            data = IMU.readDMPdataFromFIFO()

            # We have asked for orientation data so we should receive Quat9
            # Scale to +/- 1 and convert to double. Divide by 2^30
            quat9 = data['Quat9']
            q1 = quat9['Q1'] / 1073741824.0
            q2 = quat9['Q2'] / 1073741824.0
            q3 = quat9['Q3'] / 1073741824.0
            q0 = math.sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)))
            print(f'W:{q0} X:{q1} Y:{q2} Z:{q3} Acc:{quat9["Accuracy"]}')

            if tx_pipe:
                anim_data = {
                    'quat_w': q0,
                    'quat_x': q1,
                    'quat_y': q2,
                    'quat_z': q3
                }

                dat_str = json.dumps(anim_data)

                os.write(tx_pipe, dat_str.encode("utf-8"))

        except DMPFIFOUnderflow as e:
            # Not enough data available in the buffer, thats ok, moving on
            pass
        except:
            print(f"Unexpected error: {sys.exec_info()[0]}")
            raise
        finally:
            time.sleep(0.08)


if __name__ == '__main__':
    args = parser.parse_args()
    pipe = None
    if not args.local:
        pipe = args.pipe
        print('Waiting for IPC pipe to open...')
        while True:
            try:
                tx_pipe = os.open(IPC_FIFO_NAME_TX, os.O_WRONLY)
                print("IPC pipe ready")
                break
            except OSError:
                    # Wait until IPC pipe has been initialized
                    pass
    try:
        runExample(pipe)
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example 1")
        sys.exit(0)
