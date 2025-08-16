#!/usr/bin/env python3
import socket
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

posX = None
posY = None
compassHeading = None
socketClient = None

def sendData():
    global posX, posY, compassHeading, socketClient
    if posX is not None and posY is not None and compassHeading is not None and socketClient:
        msg = "{:.2f},{:.2f},{:.2f}\n".format(posX, posY, compassHeading)
        try:
            socketClient.sendall(msg.encode())
            rospy.loginfo(f"Sent: {msg.strip()}")
        except Exception as error:
            rospy.logerr(f"Socket send error: {error}")

def localPositionCallback(msg):
    global posX, posY
    posX = msg.pose.pose.position.x
    posY = msg.pose.pose.position.y
    sendData()

def headingCallback(msg):
    global compassHeading
    compassHeading = msg.data
    sendData()

def main():
    global socketClient
    rospy.init_node('gps_data_to_rover', anonymous=True)
    rospy.loginfo("Starting gps_data_to_rover node")

    ipAddress  = rospy.get_param('~ip_address', '172.20.10.10')
    portNumber = int(rospy.get_param('~port_number', 1299))

    while not rospy.is_shutdown():
        try:
            rospy.loginfo(f"Connecting to {ipAddress}:{portNumber} via TCP")
            socketClient = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            socketClient.settimeout(2.0)
            socketClient.connect((ipAddress, portNumber))
            socketClient.settimeout(None)
            rospy.loginfo("TCP connection established")
            break
        except Exception as error:
            rospy.logwarn(f"Failed to connect to TCP server: {error}. Retrying in 1s")
            rospy.sleep(1.0)

    rospy.Subscriber("/car/mavros/global_position/local", Odometry, localPositionCallback)
    rospy.Subscriber("/car/mavros/global_position/compass_hdg", Float64, headingCallback)
    rospy.spin()

    if socketClient:
        socketClient.close()
        rospy.loginfo("Socket closed")

if __name__ == '__main__':
    main()