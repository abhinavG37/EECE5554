import sys

import rospy
import serial
import utm
from Gps_driver.msg import driver_msgs

# https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm
SENSOR_NAME = "RTK_GPS"
driver_pub_msg = driver_msgs()


def data_parser(data):
    global SENSOR_NAME, driver_pub_msg
    location_publisher = rospy.Publisher(SENSOR_NAME + '/LocationData', driver_msgs, queue_size=5)

    driver_pub_msg.header.seq = 0
    data = data.split(',')
    dataDict = {}
    if data[0] == '$GPGGA':
        dataDict = {  # Ignoring the checksum and end portion for dataDict
            'HEADER': data[0],  #
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
        if dataDict["lat"] != "" and dataDict["lon"] != "":
            UTM = utm.from_latlon(dataDict["lat"], dataDict["lon"])
            driver_pub_msg.header.frame_id = data["HEADER"] + "@UTC TIME:" + data["utc"]
            driver_pub_msg.Latitude = dataDict["lat"]
            driver_pub_msg.Longitude = dataDict["lon"]
            driver_pub_msg.Utm_northing = UTM(0)
            driver_pub_msg.Utm_easting = UTM(1)
            location_publisher.publish(driver_pub_msg)  # CHECK IF SCOPE IS GLOBAL ALREADY
        else:
            driver_pub_msg.header.frame_id = data["HEADER"] + "@UTC TIME:" + data["utc"]
            driver_pub_msg.Latitude = ""
            driver_pub_msg.Longitude = ""
            driver_pub_msg.Utm_northing = ""
            driver_pub_msg.Utm_easting = ""
            location_publisher.publish(driver_pub_msg)  # CHECK IF SCOPE IS GLOBAL ALREADY
    driver_pub_msg.header.seq += 0


# DONE
def serial_read(port_, baudrate_):
    global SENSOR_NAME
    port = serial.Serial(port=port_, baudrate=baudrate_, timeout=3.)
    rospy.logdebug("Initializing {} sensor".format(SENSOR_NAME))
    rospy.logdebug("Using port: " + port_ + " | Baud Rate:" + str(baudrate_))
    while not rospy.is_shutdown():
        try:
            port = serial.Serial(port=port_, baudrate=baudrate_, timeout=3.)
            port.flushInput()
            data = port.readline()
            data_parser(data)
        except ValueError:
            rospy.logdebug("Parameter Out of Range | Check Baud Rate")
        except serial.serialutil.SerialException as e:
            port.close()
            rospy.logdebug("Device Disconnected | Waiting for Reconnection")
        except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
            rospy.logdebug("Interrupted | Exiting Execution")
            port.close()
            sys.exit()


def main():
    global SENSOR_NAME, driver_pub_msg
    rospy.init_node('gps_driver', log_level=rospy.DEBUG)
    location_publisher = rospy.Publisher(SENSOR_NAME + '/LocationData', driver_msgs, queue_size=5)
    rospy.logdebug("Initializing {} sensor".format(SENSOR_NAME))

    sampling_rate = rospy.get_param('sampling_rate_', 5.0)
    sleep_time = 1 / sampling_rate - 0.025
    rospy.Rate(sampling_rate)
    rospy.logdebug("Node Publishing Rate:".format(sampling_rate))

    serial_port = rospy.get_param('port_', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('baudrate_', 4800)
    rospy.logdebug("Initialization complete")

    rospy.logdebug("Publishing Data")
    # serial_read(serial_port, serial_baud)

    location_publisher = rospy.Publisher(SENSOR_NAME + '/LocationData', driver_msgs, queue_size=5)

    port = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=3.)
    rospy.logdebug("Initializing {} sensor".format(SENSOR_NAME))
    rospy.logdebug("Using port: " + serial_port + " | Baud Rate:" + str(serial_baud))
    while not rospy.is_shutdown():
        try:
            port = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=3.)
            port.flushInput()
            data = str(port.readline())
            rospy.logdebug("Parsing data now {}".format(data))
            data_parser(data)
        except ValueError:
            rospy.logdebug("Parameter Out of Range | Check Baud Rate")
        except serial.serialutil.SerialException as e:
            port.close()
            rospy.logdebug("Device Disconnected | Waiting for Reconnection")
        except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
            rospy.logdebug("Interrupted | Exiting Execution")
            port.close()
            sys.exit()
    location_publisher.publish(driver_pub_msg)


if __name__ == '__main__':
    main()
