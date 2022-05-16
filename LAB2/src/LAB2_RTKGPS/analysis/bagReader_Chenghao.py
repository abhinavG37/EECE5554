import glob
import os

import matplotlib.pyplot as plt
import mplcursors
import numpy as np
import rosbag
from matplotlib.ticker import FormatStrFormatter

SENSOR_NAME = "RTK_GPS"
TOPIC_NAME = str("/GPSLAB2")


#___________________CHANGE PATHS AS NEEDED____________________
amended_bag_path_dir  = "/home/abhinav137/Desktop/NEU/EECE_5554/Local_workspc/src/LAB2_RTKGPS/data/AmendedBags_Chenghao/"
# All files and directories ending with .bag and that don't begin with a dot:
bag_paths = glob.glob(amended_bag_path_dir+"*.bag")
bag_names = os.listdir(amended_bag_path_dir)

# '''
# Python version of data plotter for the .bag files.
# '''


# noinspection PyTupleAssignmentBalance
def main():
    global TOPIC_NAME
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
    #==============================================================================
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
    #==============================================================================
        # dummy  = [np.nan for i in range(5)]
        # QualityLabels = ["GNSS FIX", "NA", "NA", "RTK FIX", "RTK FLOAT"]
        # QualityIndex = [1, 2, 3, 4, 5]
        QualityColors = ['red', 'black', 'black', 'blue', 'green']
        colorList = [QualityColors[i-1] for i in QUALLIST]
    #==============================================================================
        with plt.ion():
            fig3D = plt.figure()
            ax3D  = plt.axes(projection='3d')
            ax3D.scatter(UTMELIST, UTMNLIST, ALTLIST, c=colorList, alpha=0.1,  facecolors='none')
            ax3D.set_xlabel("UTM_easting (m)")
            ax3D.set_ylabel("UTM_northing (m)")
            ax3D.set_zlabel("Altitude (m)")

            ax3D.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
            ax3D.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
            ax3D.zaxis.set_major_formatter(FormatStrFormatter('%.2f'))

            ax3D.scatter(np.nan, np.nan, np.nan, c='red', alpha=1, label="GNSS FIX", s=20)
            ax3D.legend()
            ax3D.scatter(np.nan, np.nan, np.nan, c='blue', alpha=1, label="RTK FIX", s=20)
            ax3D.legend()
            ax3D.scatter(np.nan, np.nan, np.nan, c='green', alpha=1, label="RTK FLOAT", s=40)
            ax3D.legend()
            plt.savefig("../data/Fig/plot3D{}.png".format(bag_names[counter]))
            plt.close(fig3D)
        #==============================================================================
        with plt.ion():
            fig2D = plt.figure()
            ax2D = plt.axes()
            s0 = ax2D.scatter(UTMELIST, UTMNLIST,  c=colorList, alpha=0.5)
            ax2D.set_xlabel("UTM_easting (m)")
            ax2D.set_ylabel("UTM_northing (m)")
            ax2D.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
            ax2D.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))

            s1 = ax2D.scatter(np.nan, np.nan, s=20, c='red', alpha=1, label="GNSS FIX")
            ax2D.legend()
            s4 = ax2D.scatter(np.nan, np.nan, s=20,  c='blue', alpha=1, label="RTK FIX")
            ax2D.legend()
            s5 = ax2D.scatter(np.nan, np.nan, s=20,  c='green', alpha=1, label="RTK FLOAT")
            ax2D.legend()
            mplcursors.cursor(s0)
            # ax2D.scatter(np.mean(np.array(UTMELIST)), np.mean(np.array(UTMNLIST)),  c='black', alpha=1)
            plt.show()


            # Coeffs1 = np.polyfit(np.array(UTMELIST[1733:2291]), np.array(UTMNLIST[1733:2291]), 1, full=True)  # TRTL
            # ax2D.plot(np.array(UTMELIST[1733:2291]), Coeffs1[0][0]*np.array(UTMELIST[1733:2291]) + Coeffs1[0][1])
            #
            # Coeffs2 = np.polyfit(np.array(UTMELIST[2291:3024]), np.array(UTMNLIST[2291:3024]), 1, full=True)  # TLBL
            # ax2D.plot(np.array(UTMELIST[2291:3024]), Coeffs2[0][0]*np.array(UTMELIST[2291:3024]) + Coeffs2[0][1])
            # #
            # Coeffs3 = np.polyfit(np.array(UTMELIST[453:1733]), np.array(UTMNLIST[453:1733]), 1, full=True)  # TRBR
            # ax2D.plot(np.array(UTMELIST[453:1733]), Coeffs3[0][0]*np.array(UTMELIST[453:1733]) + Coeffs3[0][1])
            #
            # TEMPUTMELIST = UTMELIST[0:453]+UTMELIST[3024:3934]
            # TEMPUTMNLIST = UTMNLIST[0:453]+UTMNLIST[3024:3934]
            # Coeffs4 = np.polyfit(np.array(TEMPUTMELIST), np.array(TEMPUTMNLIST), 1, full=True)  # BLBR
            # ax2D.plot(np.array(TEMPUTMELIST), Coeffs4[0][0]*np.array(TEMPUTMELIST) + Coeffs4[0][1])


            # TEMPUTMELIST = UTMELIST[0:245]+UTMELIST[4718:5419]
            # TEMPUTMNLIST = UTMNLIST[0:245]+UTMNLIST[4718:5419]
            # Coeffs1 = np.polyfit(np.array(TEMPUTMELIST), np.array(TEMPUTMNLIST), 1, full=True)  # TRTL
            # ax2D.plot(np.array(TEMPUTMELIST), Coeffs1[0][0]*np.array(TEMPUTMELIST) + Coeffs1[0][1])
            #
            # Coeffs2 = np.polyfit(np.array(UTMELIST[245:2158]), np.array(UTMNLIST[245:2158]), 1, full=True)  # TLBL
            # ax2D.plot(np.array(UTMELIST[245:2158]), Coeffs2[0][0]*np.array(UTMELIST[245:2158]) + Coeffs2[0][1])
            # #
            # Coeffs3 = np.polyfit(np.array(UTMELIST[3164:4718]), np.array(UTMNLIST[3164:4718]), 1, full=True)  # TRBR
            # ax2D.plot(np.array(UTMELIST[3164:4718]), Coeffs3[0][0]*np.array(UTMELIST[3164:4718]) + Coeffs3[0][1])
            #
            # Coeffs4 = np.polyfit(np.array(UTMELIST[2158:3164]), np.array(UTMNLIST[2158:3164]), 1, full=True)   # BLBR
            # ax2D.plot(np.array(UTMELIST[2158:3164]), Coeffs4[0][0]*np.array(UTMELIST[2158:3164]) + Coeffs4[0][1])


            plt.grid(color='r', linestyle='-', linewidth=0.1)
            plt.savefig("../data/Fig/plots2D_{}.png".format(bag_names[counter]))
            plt.close(fig2D)

        GPS_BAG.close()
        counter = counter + 1

    print("done")


def indexMatcher(list):
    Values_isec_motion = [106.59, 100.57, 77.02, 84.23]
    indices = []
    epsilon = 0.1
    counter = 0
    for utmindex in range(len(list)):
        for refutmval in Values_isec_motion:
            if abs(refutmval - list[utmindex]) < epsilon:
                print(refutmval - list[utmindex])
                indices.append(list.index(list[utmindex]))
    return indices



if __name__ == '__main__':
    main()
