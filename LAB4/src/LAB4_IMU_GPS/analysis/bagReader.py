import glob
import os

import rosbag
from LAB4_IMU_GPS.msg import driver_msgs
from scipy.io import savemat
from tf.transformations import *

GPSTOPIC = str('/GPS')
IMUTOPIC = str('/IMU')
MAGTOPIC = str('/Mag')

MyTopic = "/LAB4_Data"

path = "/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB4_IMU_GPS/data/Bags/Test/"
amended_path = "/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB4_IMU_GPS/data/Bags/"
bag_paths = glob.glob(path + "*.bag")
bag_names = os.listdir(path)


def main():
    global GPSTOPIC, IMUTOPIC, MAGTOPIC, MyTopic, bag_paths
    new_msg = driver_msgs
    # NEW_BAG = rosbag.Bag(str(amended_path+"/amended_data.bag"), 'w')
    counter = 1
    ThetaX = ThetaY = ThetaZ = []
    OmegaX = OmegaY = OmegaZ = []
    AlphaX = AlphaY = AlphaZ = []
    MagX   = MagY   = MagZ   = []
    UTME   = UTMN   = LAT = LON = SatNum = []



    for bags in bag_paths:
        OLD_BAG = rosbag.Bag(bags, 'r')
        for topic, msg, t in OLD_BAG.read_messages(topics=[IMUTOPIC]):
            print(msg)

        Theta_z = [math.radians(float(msg.header.frame_id.split(' ')[1])) for topic, msg, time in OLD_BAG.read_messages(topics=[IMUTOPIC])]
        Theta_y = [math.radians(float(msg.header.frame_id.split(' ')[2])) for topic, msg, time in OLD_BAG.read_messages(topics=[IMUTOPIC])]
        Theta_x = [math.radians(float(msg.header.frame_id.split(' ')[3])) for topic, msg, time in OLD_BAG.read_messages(topics=[IMUTOPIC])]
        ThetaX  = ThetaX + Theta_x
        ThetaY  = ThetaY + Theta_y
        ThetaZ  = ThetaZ + Theta_z

        Omega_x = [msg.angular_velocity.x for topic, msg, time in OLD_BAG.read_messages(topics=[IMUTOPIC])]
        Omega_y = [msg.angular_velocity.y for topic, msg, time in OLD_BAG.read_messages(topics=[IMUTOPIC])]
        Omega_z = [msg.angular_velocity.z for topic, msg, time in OLD_BAG.read_messages(topics=[IMUTOPIC])]
        OmegaX  = OmegaX + Omega_x
        OmegaY  = OmegaY + Omega_y
        OmegaZ  = OmegaZ + Omega_z

        Alpha_x = [msg.linear_acceleration.x for topic, msg, time in OLD_BAG.read_messages(topics=[IMUTOPIC])]
        Alpha_y = [msg.linear_acceleration.y for topic, msg, time in OLD_BAG.read_messages(topics=[IMUTOPIC])]
        Alpha_z = [msg.linear_acceleration.z for topic, msg, time in OLD_BAG.read_messages(topics=[IMUTOPIC])]
        AlphaX  = AlphaX + Alpha_x
        AlphaY  = AlphaY + Alpha_y
        AlphaZ  = AlphaZ + Alpha_z

        Mag_x = [msg.magnetic_field.x for topic, msg, time in OLD_BAG.read_messages(topics=[MAGTOPIC])]
        Mag_y = [msg.magnetic_field.y for topic, msg, time in OLD_BAG.read_messages(topics=[MAGTOPIC])]
        Mag_z = [msg.magnetic_field.z for topic, msg, time in OLD_BAG.read_messages(topics=[MAGTOPIC])]
        MagX  = MagX + Mag_x
        MagY  = MagY + Mag_y
        MagZ  = MagZ + Mag_z


        Latitude    = [msg.latitude       for topic, msg, time in OLD_BAG.read_messages(topics=[GPSTOPIC])]
        Longitude   = [msg.longitude      for topic, msg, time in OLD_BAG.read_messages(topics=[GPSTOPIC])]
        UTMEasting  = [msg.utmEasting     for topic, msg, time in OLD_BAG.read_messages(topics=[GPSTOPIC])]
        UTMNorthing = [msg.utmNorthing    for topic, msg, time in OLD_BAG.read_messages(topics=[GPSTOPIC])]
        SatCount    = [msg.numSatellites  for topic, msg, time in OLD_BAG.read_messages(topics=[GPSTOPIC])]
        UTME        = UTME   + UTMEasting
        UTMN        = UTMN   + UTMNorthing
        LAT         = LAT   + Latitude
        LON         = LON   + Longitude
        SatNum      = SatNum + SatCount

        OLD_BAG.close()

        counter = counter + 1
    # NEW_BAG.close()

    matDict = {"ThetaX": ThetaX, "ThetaY": ThetaY, "ThetaZ": ThetaZ,
               "OmegaX": OmegaX, "OmegaY": OmegaY, "OmegaZ": OmegaZ,
               "AlphaX": AlphaX, "AlphaY": AlphaY, "AlphaZ": AlphaZ,
               "MagX"  : MagX,   "MagY"  : MagY,   "MagZ"  : MagZ,
               "UTME"  : UTME,   "UTMN"  : UTMN,   "LAT"   : LAT, "LON": LON, "SatNum" : SatNum,
               "FSampling_IMU": 40.0, "FSampling_GPS": 1.0}
    savemat("/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB4_IMU_GPS/data/MAT/ReadData.mat", matDict)

    print("done")


if __name__ == '__main__':
    main()
