import rospy
import re
from autocycle.msg import Curve
from autocycle.srv import GetData, Action, GetCurve
import time

param = ""
length = -1
dist_travelled = 0

def subs(eq, x):
    return eval(eq.replace("s", str(x)))


def derv2(eq, x, acc=0.000001):
    return (subs(eq, x) - 2*subs(eq, x) + subs(eq, x-acc))/(acc**2)


def derv(eq, x, acc=0.000001):
    return (subs(eq, x) - subs(eq, x+acc))/acc


def update_distance(time):
    global dist_travelled, length

    ## Creates the Service Client that will get speed data
    data_getter = rospy.ServiceProxy("get_data", GetData)

    ## Updates the dist_travelled
    dist_travelled += (data_getter("vel").data*time)/length

    ## Closes the getter
    data_getter.close()


def get_deltas():
    global param

    ## Creates the Service Client that will get phi data
    data_getter = rospy.ServiceProxy("get_data", GetData)

    ## The regex that will extract the info from the np matrix
    matchcase = re.match('Matrix\(\[\[([\S]*)\],\s\[([\S]*)\]\]\)', param)

    ## Splits the parametric curve into the x and y components
    x, y = [eval(matchcase.group(1)), eval(matchcase.group(2))]

    ## Calls the data getter to give us the latest roll
    resp = data_getter("roll")
    num = 1.02 * np.cos(resp.data)

    cosdelt = np.cos(0.08)
    step = 0.01  # decrease step size for greater precision
    curve_deltas = [[subs(x, i) for i in np.arange(step, 1 + step, step)], [(num / (cosdelt * (
            ((derv(x, i) ** 2 + derv(y, i) ** 2) ** (3 / 2)) / abs(
        derv(x, i) * derv2(y, i) - derv(y, i) * derv2(x, i))))) for i in np.arange(step, 1 + step, step)]]

    data_getter.close()
    return curve_deltas


def find_x_ind(li, x):
    return li.index(min(li, key=lambda t:abs(t-x)))


def start():
    global param, dist_travelled, length
    # Registers Node with the master
    rospy.init_node('get_deltas')

    # Creates subscriber option that waits for a new
    rospy.Subscriber("cycle/curve", Curve, update_param)

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

    while param == "":
        result = curve_getter(param)
        param = result.param
        length = result.length

    # Initial Time
    time_i = time.time()

    # Grabs the first deltas
    deltas = get_deltas()

    # Continues sending commands to bike and updating deltas
    while not rospy.is_shutdown():
        result = curve_getter(param)
        if result.param != "":
            param = result.param
            length = result.length
            dist_travelled = 0
            deltas = get_deltas()
            time_i = time.time()
        
        # Time to adjust for
        time_f = time.time()

        # Adjusts for distance travelled
        update_distance(time_f-time_i)

        # Updates initial time
        time_i = time_f

        # Gets closest x to the distance travelled
        x_ind = find_x_ind(deltas, dist_travelled)

        rospy.loginfo(f"Delta Sent: {deltas[x_ind][0]}")

        # Sends commands to the `action` node
        action_sender([True, True, False], deltas[x_ind][0], 4.5, "")
        



