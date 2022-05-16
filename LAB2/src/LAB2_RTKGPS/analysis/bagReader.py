import rosbag
import os

import matplotlib.pyplot as plt
import numpy as np
import rosbag
from matplotlib.ticker import FormatStrFormatter

SENSOR_NAME = "RTK_GPS"
TOPIC_NAME = str("/GPSLAB2")


#___________________CHANGE PATHS AS NEEDED____________________

bag_path_dir  = "/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB2_RTKGPS/data/Kevin_Rosbags"
amended_bag_path_dir  = "/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB2_RTKGPS/data/AmendedBags_Kevin"
# All files and directories ending with .bag and that don't begin with a dot:
bag_paths = glob.glob(amended_bag_path_dir+"*.bag")
bag_names = os.listdir(amended_bag_path_dir)

# '''
# Python version of data plotter for the .bag files.
# '''
def main():
    global TOPIC_NAME
    global static_bagPath, motion_bagPath
    counter = 0
    for path in bag_paths:
        GPS_BAG = rosbag.Bag(path, 'r')
        LATLIST  = []
        LONGLIST = []
        QUALLIST = []
        UTMNLIST = []
        UTMELIST = []
        ALTLIST  = []
        ZONELIST = []
        ZONELETTERLIST = []

        for topic, msg, t in GPS_BAG.read_messages(topics=[TOPIC_NAME]):
            LAT  = msg.Latitude
            LON  = msg.Longitude

            LATLIST.append(msg.Latitude)
            LONGLIST.append(msg.Longitude)
            QUALLIST.append(msg.Quality)
            UTMELIST.append(msg.UTMEasting)
            UTMNLIST.append(msg.UTMNorthing)
            ALTLIST.append(msg.Altitude)
            ZONELIST.append(msg.Zone)

            print(msg)

        # Check if the grid numbers are static for the recorded data
        #  and if yes: truncate them and store the rest for further processing
        VALLIST = []
        VALLIST_FINAL = []
        for element in UTMELIST:
            val = str(element)
            VALLIST.append(val.split('.')[0][:-3])
            VALLIST_FINAL.append(float(val[3:]))
        NUM = np.unique(np.array(VALLIST))
        if NUM.shape[0] == 1:
            UTMELIST = VALLIST_FINAL

        VALLIST = []
        VALLIST_FINAL = []
        for element in UTMNLIST:
            val = str(element)
            VALLIST.append(val.split('.')[0][:-3])
            VALLIST_FINAL.append(float(val[4:]))
        NUM = np.unique(np.array(VALLIST))
        if NUM.shape[0] == 1:
            UTMNLIST = VALLIST_FINAL
        del VALLIST, VALLIST_FINAL, NUM

        fig3D = plt.figure()
        ax3D = plt.axes(projection='3d')
        ax3D.plot3D(UTMELIST, UTMNLIST, ALTLIST, 'gray')
        ax3D.set_xlabel("UTM_easting (m)")
        ax3D.set_ylabel("UTM_northing (m)")
        ax3D.set_zlabel("Altitude (m)")
        ax3D.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        ax3D.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        ax3D.zaxis.set_major_formatter(FormatStrFormatter('%.2f'))

        plt.savefig("../data/Fig/plot3D{}.pdf".format(counter))

        fig2D = plt.figure()
        ax2D = plt.axes()
        ax2D.plot(UTMELIST, UTMNLIST, 'o', color='#88c999', markersize=1.5)
        ax2D.set_xlabel("UTM_easting (m)")
        ax2D.set_ylabel("UTM_northing (m)")
        ax2D.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        ax2D.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        plt.show()
        plt.savefig("../data/Fig/plots2D{}.pdf".format(counter))

        # m, b = np.polyfit(np.array(UTMELIST), np.array(UTMELIST), 1)
        # plt.plot(np.array(UTMELIST), m*np.array(UTMELIST) + b)

        GPS_BAG.close()
        counter = counter + 1

    print("done")


if __name__ == '__main__':
    main()
