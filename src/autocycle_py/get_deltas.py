import rospy
import re
from autocycle.srv import GetData, Action, GetCurve
import time
import numpy as np

id = -1
xs = []
deltas = []
length = -1
dist_travelled = 0


def update_distance(time):
    global dist_travelled, length

    ## Creates the Service Client that will get speed data
    data_getter = rospy.ServiceProxy("get_data", GetData)

    ## Updates the dist_travelled
    dist_travelled += (data_getter("vel").data*time)/length

    ## Closes the getter
    data_getter.close()

def find_x_ind(li, x):
    return li.index(min(li, key=lambda t:abs(t-x)))


def start():
    global param, dist_travelled, length, id, xs, deltas
    # Registers Node with the master
    rospy.init_node('get_deltas')

    # Waits for the data getter to be done setting up
    rospy.wait_for_service('get_data')

    # Waits for the send_action
    rospy.wait_for_service("send_action")
    
    # Waits for curve getter service
    rospy.wait_for_service("get_curve")

    # Creates the Service client that will send actions to the bike!
    action_sender = rospy.ServiceProxy("send_action", Action)
    
    # Creates the service cline that will get the curve
    curve_getter = rospy.ServiceProxy("get_curve", GetCurve)

    while id == -1:
        result = curve_getter(id)
        xs = result.xs
        deltas = result.deltas
        id = result.id
        length = result.length

    # Initial Time
    time_i = time.time()


    # Continues sending commands to bike and updating deltas
    while not rospy.is_shutdown():
        result = curve_getter(id)
        if result.id != -1:
            id = result.id
            length = result.length
            dist_travelled = 0
            update_distance(result.time)
            deltas = result.deltas
            time_i = time.time()
        
        # Time to adjust for
        time_f = time.time()

        # Adjusts for distance travelled
        update_distance(time_f-time_i)

        # Updates initial time
        time_i = time_f
        
        try:
            # Gets closest x to the distance travelled
            x_ind = find_x_ind(xs, dist_travelled)

            rospy.loginfo(f"Delta Sent: {deltas[1][x_ind]}")

            # Sends commands to the `action` node
            action_sender([True, True, False], deltas[1][x_ind], 4.5, "")
        except ValueError:
            pass
        



