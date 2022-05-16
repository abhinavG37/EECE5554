import glob
import os

import rosbag
import utm
from LAB2_RTKGPS.msg import driver_msgs

SENSOR_NAME = "RTK_GPS"
TOPIC_NAME = str("/GPSLAB2")

# ___________________CHANGE PATHS AS NEEDED____________________
bag_path_dir = "/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB2_RTKGPS/data/Chenghao_Rosbags/"
amended_bag_path_dir = "/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB2_RTKGPS/data/AmendedBags_Chenghao/"
# All files and directories ending with .bag and that don't begin with a dot:
bag_paths = glob.glob(bag_path_dir + "*.bag")
bag_names = os.listdir(bag_path_dir)


# '''
# Helper script written during initial run to rectify in-bag mistake of swapped UTM easting and UTM northing values
# Not used in final processing
# '''
def main():
    global TOPIC_NAME
    global bagPath
    new_msg = driver_msgs()
    file_counter = 0
    for OLD_BAG_PATH in bag_paths:
        # current_time = datetime.now().strftime("%H_%M_%S")
        # BAG_PATH = str(os.getcwd() + '/gps_data_{}_{}_amended.bag'.format(current_time, file_counter))
        NEW_BAG_PATH = str(amended_bag_path_dir + bag_names[file_counter])
        NEW_GPS_BAG = rosbag.Bag(NEW_BAG_PATH, 'w')
        OLD_GPS_BAG = rosbag.Bag(OLD_BAG_PATH, 'r')

        for topic, old_msg, t in OLD_GPS_BAG.read_messages(topics=["/gps_data"]):

            UTM = utm.from_latlon(old_msg.latitude, old_msg.longitude)
            # old_msg.UTM_Easting  = UTM[0]
            # old_msg.utm_northing = UTM[1]
            # old_msg.zone         = UTM[2]
            # old_msg.letter       = UTM[3]

            new_msg.Header = old_msg.header
            new_msg.Altitude = float(old_msg.altitude)
            new_msg.Quality = int(float(old_msg.quality))
            new_msg.Latitude = float(old_msg.latitude)
            new_msg.Longitude = float(old_msg.longitude)
            # new_msg.UTMEasting = float(old_msg.UTM_Easting)
            # new_msg.UTMNorthing = float(old_msg.UTM_Northing)
            # new_msg.ZoneLetter = old_msg.UTM_Letter
            # new_msg.Zone = int(old_msg.UTM_Zone)
            new_msg.UTMEasting = UTM[0]
            new_msg.UTMNorthing = UTM[1]
            new_msg.Zone = UTM[2]
            new_msg.ZoneLetter = UTM[3]

            NEW_GPS_BAG.write(TOPIC_NAME, new_msg)
            print(old_msg)

        OLD_GPS_BAG.close()
        NEW_GPS_BAG.close()
        file_counter = file_counter + 1


if __name__ == '__main__':
    main()
