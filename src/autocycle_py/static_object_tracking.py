import numpy as np
import rospy
from autocycle.msg import Object, ObjectList
from autocycle.srv import GetData, ObjectDetectionList

height = 100              # vertical dimension in millimeters
width = 200               # horizontal dimension in millimeters
object_length = 1000		# object length

new_objects = []
objects = []
heading = 0
time = 0

def static_object_tracking():
    global objects, heading, time, temp_objects
    # Registers the Node with the Master
    rospy.init_node("static_object_tracking")

    # Waits for data getter service
    rospy.wait_for_service('get_data')
    
    # Waits for the object list getter service
    rospy.wait_for_service("object_list_getter")

    # Creates the service client that will collect data
    data_getter = rospy.ServiceProxy("get_data", GetData)
    
    # Creates the service client that will get the object lists
    obj_lst_getter = rospy.ServiceProxy("object_list_getter")

    # Subscribes to the object topic
    rospy.Subscriber("cycle/objects", ObjectList, new_object)

    # Publishes objects to path topic
    pub = rospy.Publisher("cycle/object_frame", ObjectList, queue_size=1)

    heading = data_getter("heading").data
    time = data_getter("met").data

    while not new_objects:
        temp_objects = obj_lst_getter(objects).obj_lst
        
    objects = new_objects.copy()
        
    while not rospy.is_shutdown():
        temp_objects = obj_lst_getter(temp_objects).obj_lst
        if temp_objects:
            objects.extend(temp_objects)
        
        # Collects data
        new_heading = data_getter("heading").data
        delta_angle = new_heading - heading
        heading = new_heading

        new_time = data_getter("met").data
        delta_time = new_time - time
        time = new_time
		
        speed = data_getter("vel").data

        theta = delta_angle
        c, s = np.cos(theta), np.sin(theta)
        rotation_matrix = np.array([[c, -s], [s, c]])
        distance = speed * delta_time
        new = []
        for obj in objects:
            rotated = rotation_matrix @ np.array([[obj.x1, obj.x2], [obj.z1, obj.z2]])
            new.append(Object(rotated[0,0], rotated[1,0] - distance, rotated[0,1], rotated[1,1] - distance))
        objects = list(filter(lambda x: x.z1 >= 0 or x.z2 >= 0, new))

        pub.publish(objects)

