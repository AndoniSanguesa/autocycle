import rospy
from autocycle_extras.srv import TCK

def dummy_fun(data):
    return []

def start():

    # Initializes the ROS node
    rospy.init_node("ready_to_start")

    # Creates Dummy Service to confirm subscriber is ready to go
    dummy = rospy.Service("ready_to_start", TCK, dummy_fun)

    rospy.spin()