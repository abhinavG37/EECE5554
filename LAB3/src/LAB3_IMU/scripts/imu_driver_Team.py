import os
import sys
from datetime import datetime

import rosbag
import rospy
import serial
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import *

current_time = datetime.now().strftime("%H_%M_%S")

SENSOR_NAME = "IMU"
ORI_TOPIC_NAME = str("/ImuLab3")
MAG_TOPIC_NAME = str("/MagLab3")
BAG_PATH = str(os.getcwd() + '/imu_data_{}.bag'.format(current_time))
BAG = rosbag.Bag(BAG_PATH, 'w')
seq = 0


# noinspection SpellCheckingInspection
def data_parser(data):
    """
    Parse a data input into a comma separated list and further into a VNYMR format dict
    Publishes a message on a topic for each line starting with $VNYMR
    """

    global SENSOR_NAME, ORI_TOPIC_NAME, MAG_TOPIC_NAME, seq
    imu_publisher = rospy.Publisher(ORI_TOPIC_NAME, Imu, queue_size=1)
    mag_publisher = rospy.Publisher(MAG_TOPIC_NAME, MagneticField, queue_size=1)

    imu_msg_pub = Imu()
    mag_msg_pub = MagneticField()
    data = data.split(',')
    if data[0][2:] == '$VNYMR':  # String contains b\$XXXXX
        dataDict = {  # Ignoring the checksum and end portion for dataDict
            'Yaw': float(data[1]),  # float	deg	   Calculated attitude heading angle in degrees.
            'Pitch': float(data[2]),  # float	deg	   Calculated attitude heading angle in degrees.
            'Roll': float(data[3]),  # float	deg	   Calculated attitude heading angle in degrees.
            'MagX': float(data[4]),  # float	Gauss  Compensated magnetometer measurement in x-axis.
            'MagY': float(data[5]),  # float	Gauss  Compensated magnetometer measurement in y-axis.
            'MagZ': float(data[6]),  # float	Gauss  Compensated magnetometer measurement in z-axis.
            'AccelX': float(data[7]),  # float	m/s2   Compensated Accelerometer measurement in x-axis.
            'AccelY': float(data[8]),  # float	m/s2   Compensated Accelerometer measurement in y-axis.
            'AccelZ': float(data[9]),  # float	m/s2   Compensated Accelerometer measurement in z-axis.
            'GyroX': float(data[10]),  # float	rad/s  Compensated Angular rate measurement in x-axis.
            'GyroY': float(data[11]),  # float	rad/s  Compensated Angular rate measurement in y-axis.
            'GyroZ': float(data[12].split('*')[0]),  # float	rad/s  Compensated Angular rate measurement in z-axis. [INCLUDES CHECKSUM]
        }
        rospy.logdebug("Received $VNYMR Serial Data:\n{}".format(dataDict))

        imu_msg_pub.header.frame_id = SENSOR_NAME + "/VN-100S/ORIENTATION"
        mag_msg_pub.header.frame_id = SENSOR_NAME + "/VN-100S/MAGFIELD"
        imu_msg_pub.header.seq = seq
        mag_msg_pub.header.seq = seq
        imu_msg_pub.header.stamp = rospy.Time.now()
        mag_msg_pub.header.stamp = rospy.Time.now()

        print("CONVERTING RPY to XYZW format")
        Quat = quaternion_from_euler(dataDict["Roll"], dataDict["Pitch"], dataDict["Yaw"])
        rospy.loginfo("Quaternion from orientation RPY Values:{}".format(Quat))

        [imu_msg_pub.orientation.x, imu_msg_pub.orientation.y,
         imu_msg_pub.orientation.z, imu_msg_pub.orientation.w] = Quat

        [imu_msg_pub.angular_velocity.x,
         imu_msg_pub.angular_velocity.y,
         imu_msg_pub.angular_velocity.x] = [dataDict["GyroX"], dataDict["GyroY"], dataDict["GyroZ"]]

        [imu_msg_pub.linear_acceleration.x,
         imu_msg_pub.linear_acceleration.y,
         imu_msg_pub.linear_acceleration.x] = [dataDict["AccelX"], dataDict["AccelY"], dataDict["AccelZ"]]

        [mag_msg_pub.magnetic_field.x,
         mag_msg_pub.magnetic_field.y,
         mag_msg_pub.magnetic_field.x] = [dataDict["MagX"], dataDict["MagY"], dataDict["MagZ"]]

        rospy.logdebug("PUBLISHING MESSAGE:\t\n{}\n{}".format(imu_msg_pub, mag_msg_pub))
        imu_publisher.publish(imu_msg_pub)
        mag_publisher.publish(mag_msg_pub)

        print("Writing IMU message: {} to bag".format(imu_msg_pub))
        BAG.write(ORI_TOPIC_NAME, imu_msg_pub)
        BAG.write(MAG_TOPIC_NAME, mag_msg_pub)

    seq = seq + 1


# DONE


def serial_read(port_, baudrate_):
    """
    Opens a port based on specified port name and baud rate and checks for incoming data
    Reconnection feature => Checks if device disconnected, then keeps listening at the specified port
      for incoming data but does not keep the port occupied after listening
    """
    global SENSOR_NAME, BAG
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
        except serial.serialutil.SerialException as e:
            port.close()
            rospy.logdebug("Device Disconnected | Waiting for Reconnection")
        except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
            rospy.logdebug("Interrupted | Exiting Execution")
            BAG.close()
            port.close()
            sys.exit()
    rospy.on_shutdown(shutdownHook)


def shutdownHook():  # Callback hook when Ctrl + C is pressed on the rosnode terminal
    BAG.close()
    print("Shutdown time!")


def main():
    global SENSOR_NAME
    rospy.logdebug("Initializing {} data publisher node".format(SENSOR_NAME))

    rospy.init_node('gps_driver', log_level=rospy.DEBUG)
    PublishRate = rospy.get_param('PublishRate_', 40.0)
    rospy.Rate(PublishRate)
    rospy.logdebug("Node Publishing Rate:".format(PublishRate))

    serial_port = rospy.get_param('port_', '/dev/pts/3')

    serial_baud = rospy.get_param('baudrate_', 115200)
    rospy.logdebug("Initialization complete")

    rospy.logdebug("Publishing Data")
    serial_read(serial_port, serial_baud)


if __name__ == '__main__':
    main()
