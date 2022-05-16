import glob
import math
import os

import rosbag
from scipy.io import savemat

IMU_TOPIC_NAME = str("/ImuLab3")
MAG_TOPIC_NAME = str("/MagLab3")

# ___________________CHANGE PATHS AS NEEDED____________________

bag_path_dir = "/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB3_IMU/data/Bags/FinalBags/"
# All files and directories ending with .bag
bag_paths = glob.glob(bag_path_dir + "*.bag")
bag_names = os.listdir(bag_path_dir)


# Used this script since YPR values were recorded and a swap occured. Saves .mat files in MAT folder

def main():
    global IMU_TOPIC_NAME, MAG_TOPIC_NAME
    counter = 0
    for path in bag_paths:
        BAG = rosbag.Bag(path, 'r')
        # Theta   = [euler_from_quaternion([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]) for topic, msg, time in BAG.read_messages(topics=[IMU_TOPIC_NAME])]
        Theta_z = [math.radians(float(msg.header.frame_id.split(' ')[1])) for topic, msg, time in BAG.read_messages(topics=[IMU_TOPIC_NAME])]
        Theta_y = [math.radians(float(msg.header.frame_id.split(' ')[2])) for topic, msg, time in BAG.read_messages(topics=[IMU_TOPIC_NAME])]
        Theta_x = [math.radians(float(msg.header.frame_id.split(' ')[3])) for topic, msg, time in BAG.read_messages(topics=[IMU_TOPIC_NAME])]
        Omega_x = [msg.orientation.x for topic, msg, time in BAG.read_messages(topics=[IMU_TOPIC_NAME])]
        Omega_y = [msg.orientation.y for topic, msg, time in BAG.read_messages(topics=[IMU_TOPIC_NAME])]
        Omega_z = [msg.orientation.z for topic, msg, time in BAG.read_messages(topics=[IMU_TOPIC_NAME])]
        Alpha_x = [msg.linear_acceleration.x for topic, msg, time in BAG.read_messages(topics=[IMU_TOPIC_NAME])]
        Alpha_y = [msg.linear_acceleration.y for topic, msg, time in BAG.read_messages(topics=[IMU_TOPIC_NAME])]
        Alpha_z = [msg.linear_acceleration.z for topic, msg, time in BAG.read_messages(topics=[IMU_TOPIC_NAME])]
        Mag_x = [msg.magnetic_field.x for topic, msg, time in BAG.read_messages(topics=[MAG_TOPIC_NAME])]
        Mag_y = [msg.magnetic_field.y for topic, msg, time in BAG.read_messages(topics=[MAG_TOPIC_NAME])]
        Mag_z = [msg.magnetic_field.z for topic, msg, time in BAG.read_messages(topics=[MAG_TOPIC_NAME])]
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
