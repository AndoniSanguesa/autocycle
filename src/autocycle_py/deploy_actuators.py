import serial
import rospy

baudrate = 115200
ser = serial.Serial("/dev/ttyACM0", baudrate=baudrate)



def main():
    rospy.init_node("deploy_actuators")
    ser.write(b"c1024")
