import glob
import os

import rosbag
import utm

SENSOR_NAME = "RTK_GPS"
TOPIC_NAME = str("/GPSLAB2")

#___________________CHANGE PATHS AS NEEDED____________________
bag_path_dir  = "/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB2_RTKGPS/data/Kevin_Rosbags/"
amended_bag_path_dir  = "/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB2_RTKGPS/data/AmendedBags_Kevin/"
# All files and directories ending with .bag and that don't begin with a dot:
bag_paths = glob.glob(bag_path_dir+"*.bag")
bag_names = os.listdir(bag_path_dir)

# '''
# Helper script written during initial run to rectify in-bag mistake of swapped UTM easting and UTM northing values
# Not used in final processing
# '''
def main():
    global TOPIC_NAME
    global bagPath, static_bagPath, motion_bagPath
    file_counter = 0
    # for path in [bagPath, motion_bagPath]:
    for OLD_BAG_PATH in bag_paths:
        # current_time = datetime.now().strftime("%H_%M_%S")
        # BAG_PATH = str(os.getcwd() + '/gps_data_{}_{}_amended.bag'.format(current_time, file_counter))
        NEW_BAG_PATH = str(amended_bag_path_dir+bag_names[file_counter])
        NEW_GPS_BAG = rosbag.Bag(NEW_BAG_PATH, 'w')
        OLD_GPS_BAG = rosbag.Bag(OLD_BAG_PATH, 'r')

        for topic, old_msg, t in OLD_GPS_BAG.read_messages(topics=["/serial/gps"]):
            old_msg.longitude = old_msg.longitude*-1.
            UTM = utm.from_latlon(old_msg.latitude, old_msg.longitude)
            old_msg.utm_easting  = UTM[0]
            old_msg.utm_northing = UTM[1]
            old_msg.zone         = UTM[2]
            old_msg.letter       = UTM[3]
            # old_msg.ZoneLetter   = UTM[3]

            NEW_GPS_BAG.write(TOPIC_NAME, old_msg)
            print(old_msg)
        OLD_GPS_BAG.close()
        NEW_GPS_BAG.close()
        file_counter = file_counter + 1


if __name__ == '__main__':
    main()
