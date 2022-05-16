import glob
import os

import rosbag
from scipy.io import savemat
from tf.transformations import *

SENSOR_NAME = "IMU_VN100S"
TOPIC_NAME = str(SENSOR_NAME + '/Data')

# ___________________CHANGE PATHS AS NEEDED____________________

bag_path_dir = "/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB3_IMU/data/Bags/Short/"
# All files and directories ending with .bag and that don't begin with a dot:
bag_paths = glob.glob(bag_path_dir + "*.bag")
bag_names = os.listdir(bag_path_dir)


def main():
    """
     Reads Bag files and converts to a .MAT format for MATLAB analysis
    """
    counter = 0
    for path in bag_paths:
        BAG = rosbag.Bag(path, 'r')
        Theta   = [euler_from_quaternion([msg.imu.orientation.x, msg.imu.orientation.y,
                                          msg.imu.orientation.z, msg.imu.orientation.w])
                   for topic, msg, time in BAG.read_messages(topics=[TOPIC_NAME])]
        Theta_x = [thetaX[0] for thetaX in Theta]
        Theta_y = [thetaY[1] for thetaY in Theta]
        Theta_z = [thetaZ[2] for thetaZ in Theta]
        Omega_x = [msg.imu.orientation.x for topic, msg, time in BAG.read_messages(topics=[TOPIC_NAME])]
        Omega_y = [msg.imu.orientation.y for topic, msg, time in BAG.read_messages(topics=[TOPIC_NAME])]
        Omega_z = [msg.imu.orientation.z for topic, msg, time in BAG.read_messages(topics=[TOPIC_NAME])]
        Alpha_x = [msg.imu.linear_acceleration.x for topic, msg, time in BAG.read_messages(topics=[TOPIC_NAME])]
        Alpha_y = [msg.imu.linear_acceleration.y for topic, msg, time in BAG.read_messages(topics=[TOPIC_NAME])]
        Alpha_z = [msg.imu.linear_acceleration.z for topic, msg, time in BAG.read_messages(topics=[TOPIC_NAME])]
        Mag_x = [msg.magField.magnetic_field.x for topic, msg, time in BAG.read_messages(topics=[TOPIC_NAME])]
        Mag_y = [msg.magField.magnetic_field.y for topic, msg, time in BAG.read_messages(topics=[TOPIC_NAME])]
        Mag_z = [msg.magField.magnetic_field.z for topic, msg, time in BAG.read_messages(topics=[TOPIC_NAME])]

        matDict = {"Theta_x": Theta_x, "Theta_y": Theta_y, "Theta_z": Theta_z,
                   "Omega_x": Omega_x, "Omega_y": Omega_y, "Omega_z": Omega_z,
                   "Alpha_x": Alpha_x, "Alpha_y": Alpha_y, "Alpha_z": Alpha_z,
                   "Mag_x": Mag_x, "Mag_y": Mag_y, "Mag_z": Mag_z, "FSampling": 40}
        savemat("/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB3_IMU/data/MAT/{}.mat".format(bag_names[counter]), matDict)
        BAG.close()
        counter = counter + 1

    print("done")


if __name__ == '__main__':
    main()
