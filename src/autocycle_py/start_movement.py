import rospy
from autocycle_extras.srv import TCK

def dummy_fun(data):
    return []

def start():

    # Initializes the ROS node
    rospy.init_node("start_movement")

    # Creates Dummy Service to confirm subscriber is ready to go
    dummy = rospy.Service("start_movement", TCK, dummy_fun)

    rospy.spin()