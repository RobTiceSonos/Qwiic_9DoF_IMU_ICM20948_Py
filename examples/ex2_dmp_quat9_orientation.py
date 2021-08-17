from __future__ import print_function

from icm20948 import qwiic_icm20948, dmp

import argparse
import json
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

    if not IMU.connected:
        print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return

    # Initialize the ICM-20948 and Initialize the DMP
    IMU.begin(dmp=True)
    # Enable the DMP orientation sensor
    IMU.enableDMPSensor(dmp.sensors.Sensor_Types.ORIENTATION)
    # Configuring DMP to output data at multiple ODRs:
    # DMP is capable of outputting multiple sensor data at different rates to FIFO.
    # Setting value can be calculated as follows:
    # Value = (DMP running rate / ODR ) - 1
    # E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    IMU.setDMPODRrate(dmp.regs.Output_Data_Rate_Control.QUAT9, 2)
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
        try:
            # Read any DMP data waiting in the FIFO
            data = IMU.readDMPdataFromFIFO()
        except (dmp.fifo.FifoUnderflow, dmp.fifo.FifoOverflow) as ex:
            print(f'Exception: {type(ex).__name__}. Continuing...')
            continue

        proc_data = IMU.processDMPdata(data)
        if proc_data:
            if tx_pipe:
                anim_data = {
                    'quat_w': proc_data['quat9']['q0'],
                    'quat_x': proc_data['quat9']['q1'],
                    'quat_y': proc_data['quat9']['q2'],
                    'quat_z': proc_data['quat9']['q3']
                }

                dat_str = json.dumps(anim_data)

                os.write(tx_pipe, dat_str.encode("utf-8"))
                time.sleep(0.03)


if __name__ == '__main__':
    args = parser.parse_args()
    tx_pipe = None
    if not args.local:
        pipe = args.pipe
        print('Waiting for IPC pipe to open...')
        while True:
            try:
                tx_pipe = os.open(pipe, os.O_WRONLY)
                print("IPC pipe ready")
                break
            except OSError:
                    # Wait until IPC pipe has been initialized
                    pass
    try:
        runExample(tx_pipe)
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example 1")
        sys.exit(0)
