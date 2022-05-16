import math

import rosbag
from scipy.io import savemat

IMU_TOPIC_NAME = str("/ImuLab3")
MAG_TOPIC_NAME = str("/MagLab3")

Theta_x = []
Theta_y = []
Theta_z = []

BAG = rosbag.Bag(f'/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB3_IMU/data/Bags/FinalBags/Short_Final_unswapped.bag', 'r')
for topic, msg, t in BAG.read_messages(topics=[IMU_TOPIC_NAME]):
    yaw_pitch_roll = msg.header.frame_id.split(' ')
    Theta_z.append(math.radians(float(yaw_pitch_roll[1])))
    Theta_y.append(math.radians(float(yaw_pitch_roll[2])))
    Theta_x.append(math.radians(float(yaw_pitch_roll[3])))


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

savemat("/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB3_IMU/data/MAT/Short_FINAL.mat", matDict)
BAG.close()
# break
