import os
import sys
from datetime import datetime

import rosbag
import rospy
import serial
import utm
from LAB1.msg import driver_msgs

# https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm

current_time = datetime.now().strftime("%H_%M_%S")

SENSOR_NAME = "RTK_GPS"
TOPIC_NAME = str(SENSOR_NAME + '/LocationData')
BAG_PATH = str(os.getcwd() + '/gps_data_{}.bag'.format(current_time))
GPS_BAG = rosbag.Bag(BAG_PATH, 'w')


def decimal_degree_convertor(value, direction):
    xDDMM_MMMM = value.split('.')
    xDDMM = xDDMM_MMMM[0]
    MMMM = xDDMM_MMMM[1]
    if direction == "W" or direction == "S":
        xDD = float(xDDMM[:-2])
        MM_MMMM = float(xDDMM[-2:] + "." + MMMM)
        print(-1. * (xDD + (MM_MMMM / 60.)))
        return -1. * (xDD + (MM_MMMM / 60.))

    elif direction == "N" or direction == "E":
        print("DIRECTION COOL")
        xDD = float(xDDMM[:-2])
        MM_MMMM = float(xDDMM[-2:] + "."+MMMM)
        print(xDD + (MM_MMMM / 60.))
        return xDD + (MM_MMMM / 60.)


def data_parser(data):
    global SENSOR_NAME, TOPIC_NAME
    location_publisher = rospy.Publisher(SENSOR_NAME + '/LocationData', driver_msgs, queue_size=5)
    driver_pub_msg = driver_msgs()
    driver_pub_msg.header.seq = 0
    data = data.split(',')
    # print("DATA HEADER:\n{}".format(data[0]))

    if data[0][2:] == '$GPGGA':  # String contains b\$XXXXX
        dataDict = {  # Ignoring the checksum and end portion for dataDict
            'HEADER': data[0][2:],  #
            'utc': data[1],  # UTC time status of position (hours/minutes/seconds/ decimal seconds)
            'lat': data[2],  # Latitude
            'lat_dir': data[3],  # Latitude Direction: (N = North, S = South)
            'lon': data[4],  # Longitude (DDDmm.mm)
            'lon_dir': data[5],  # Longitude direction (E = East, W = West)
            'quality': data[6],  # Signal Quality Metric
            'satCount': data[7],  # Number of satellites in use. May be different to the number in view
            'hdop': data[8],  # Horizontal dilution of precision
            'alt': data[9],  # Antenna altitude above/below mean sea level
            'a_units': data[10],  # Units of antenna altitude (M = metres)
            'undulation': data[11],  # Undulation - the relationship between the geoid and the WGS84 ellipsoid
            'u_units': data[12],  # Units of undulation (M = metres)
            'age': data[13],  # Age of correction data (in seconds)| The maximum age reported here is limited to 99 seconds.
            'stnID': data[14],  # Differential base station ID
        }
        rospy.logdebug("Received GPGGA Serial Data:\n{}".format(dataDict))
        # print("dataDict[\"lat\"]:{} \t dataDict[\"lon\"]:{}".format(dataDict["lat"], dataDict["lon"]))
        if dataDict["lat"] == "" and dataDict["lon"] == "":
            driver_pub_msg.header.frame_id = str(dataDict["HEADER"] + "@UTC TIME:" + dataDict["utc"])
            driver_pub_msg.Latitude = 0.
            driver_pub_msg.Longitude = 0.
            driver_pub_msg.Altitude = 0.
            driver_pub_msg.Utm_easting = 0.
            driver_pub_msg.Utm_northing = 0.
            driver_pub_msg.Zone = "NA"
            driver_pub_msg.ZoneLetter = "NA"
            rospy.logdebug("PUBLISHING MESSAGE:{}".format(driver_pub_msg))
            location_publisher.publish(driver_pub_msg)  # CHECK IF SCOPE IS GLOBAL ALREADY\
            # GPS_BAG.write(TOPIC_NAME, driver_pub_msg)
        else:
            print("CONVERTING FROM xDDMM.MMMM TO DD.DDDD format")
            LAT = decimal_degree_convertor(dataDict["lat"], dataDict["lat_dir"])
            LON = decimal_degree_convertor(dataDict["lon"], dataDict["lon_dir"])

            UTM = utm.from_latlon(LAT, LON)
            driver_pub_msg.header.frame_id = str(dataDict["HEADER"] + "| @UTC TIME:" + dataDict["utc"])
            driver_pub_msg.Latitude = LAT
            driver_pub_msg.Longitude = LON
            driver_pub_msg.Altitude = float(dataDict["alt"])
            driver_pub_msg.Utm_easting = UTM[0]
            driver_pub_msg.Utm_northing = UTM[1]
            driver_pub_msg.Zone = str(UTM[2])
            driver_pub_msg.ZoneLetter = str(UTM[3])
            rospy.logdebug("PUBLISHING MESSAGE:{}".format(driver_pub_msg))
            location_publisher.publish(driver_pub_msg)  # CHECK IF SCOPE IS GLOBAL ALREADY
            print("Writing {} to bag".format(driver_pub_msg))
            GPS_BAG.write(TOPIC_NAME, driver_pub_msg)
    driver_pub_msg.header.seq += 0


# DONE
def serial_read(port_, baudrate_):
    global SENSOR_NAME, GPS_BAG
    port = serial.Serial(port=port_, baudrate=baudrate_, timeout=3.)
    rospy.logdebug("Initializing {} sensor".format(SENSOR_NAME))
    rospy.logdebug("Using port: " + port_ + " | Baud Rate:" + str(baudrate_))
    while not rospy.is_shutdown():
        try:
            port = serial.Serial(port=port_, baudrate=baudrate_, timeout=3.)
            port.flushInput()
            data = str(port.readline())
            rospy.logdebug("Parsing data now {}".format(data))
            data_parser(data)
        # except ValueError:
        #     rospy.logdebug("Parameter Out of Range | Check Baud Rate")
        except serial.serialutil.SerialException as e:
            port.close()
            rospy.logdebug("Device Disconnected | Waiting for Reconnection")
        except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
            rospy.logdebug("Interrupted | Exiting Execution")
            GPS_BAG.close()
            port.close()
            sys.exit()
    rospy.on_shutdown(shutdownHook)


def shutdownHook():
    GPS_BAG.close()
    print("Shutdown time!")


def main():
    global SENSOR_NAME, TOPIC_NAME
    rospy.logdebug("Initializing {} data publisher node".format(SENSOR_NAME))
    rospy.init_node('gps_driver', log_level=rospy.DEBUG)
    PublishRate = rospy.get_param('PublishRate_', 1.0)
    sleep_time = 1 / PublishRate - 0.025
    rospy.Rate(PublishRate)
    rospy.logdebug("Node Publishing Rate:".format(PublishRate))

    serial_port = rospy.get_param('port_', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('baudrate_', 4800)
    rospy.logdebug("Initialization complete")

    rospy.logdebug("Publishing Data")
    serial_read(serial_port, serial_baud)


if __name__ == '__main__':
    main()
