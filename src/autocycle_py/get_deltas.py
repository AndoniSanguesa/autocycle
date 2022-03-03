import rospy
from autocycle_extras.srv import GetDelta, TCK
from scipy import interpolate as interp

tck = []
full_len = 0

def get_dervs(x, acc=0.000001):
    """Calculates the derivatives of the curve"""
    k = x / full_len
    subs = lambda x: interp.splev(x, tck)
    this_val = subs(k)
    next_val = subs(k + acc)
    return ((this_val[0] - next_val[0]) / acc,
            (this_val[1] - next_val[1]) / acc)
    


def get_delta(i):
    """Calculates steering angle"""
    global step
    ## Calls the data getter to give us the latest roll
    dervs = get_dervs(i)

    return np.arctan2(dervs[1], dervs[0])

    


def delta(data):
    print(f"RECV DELTA PARAMS: {data.x} DELTA CALCULATED: {get_delta(data.x)}")
    return get_delta(data.x, data.vel) if tck else -1


def update_deltas(data):
    global tck, full_len
    tck = (data.t, [data.c1, data.c2], data.k)
    full_len = data.full_len
    return []


def start():
    # Registers Node with the master
    rospy.init_node('get_deltas')

    # Service that will return the appropriate delta for a given x-value and roll
    fetch_delta = rospy.Service("get_delta", GetDelta, delta)

    # Service that will update the tck and full_len values
    new_deltas = rospy.Service("update_deltas", TCK, update_deltas)
        
    rospy.spin()


