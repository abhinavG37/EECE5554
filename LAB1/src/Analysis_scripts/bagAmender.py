import os
from datetime import datetime

import rosbag
import utm

SENSOR_NAME = "RTK_GPS"
TOPIC_NAME = str(SENSOR_NAME + '/LocationData')

#___________________CHANGE PATHS AS NEEDED____________________
# static_bagPath = "../data/BAG/__.bag"
# motion_bagPath = "../data/BAG/__.bag"

# '''
# Helper script written during initial run to rectify in-bag mistake of swapped UTM easting and UTM northing values
# Not used in final processing
# '''
def main():
    global TOPIC_NAME
    global static_bagPath, motion_bagPath
    counter = 1
    for path in [static_bagPath, motion_bagPath]:
        current_time = datetime.now().strftime("%H_%M_%S")
        BAG_PATH = str(os.getcwd() + '/gps_data_{}_{}.bag'.format(current_time, counter))
        NEW_GPS_BAG = rosbag.Bag(BAG_PATH, 'w')
        OLD_GPS_BAG = rosbag.Bag(path, 'r')
        for topic, msg, t in OLD_GPS_BAG.read_messages(topics=[TOPIC_NAME]):
            LAT = msg.Latitude
            LON = msg.Longitude
            UTM = utm.from_latlon(LAT, LON)
            msg.Utm_easting  = UTM[0]
            msg.Utm_northing = UTM[1]
            msg.Zone         = UTM[2]
            # temp = msg.Utm_northing
            # msg.Utm_northing = msg.Utm_easting
            # msg.Utm_easting = temp
            NEW_GPS_BAG.write(TOPIC_NAME, msg)
            print(msg)
        OLD_GPS_BAG.close()
        NEW_GPS_BAG.close()
        counter = counter + 1


if __name__ == '__main__':
    main()
