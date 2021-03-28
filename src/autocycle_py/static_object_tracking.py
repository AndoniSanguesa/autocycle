import numpy as np
import rospy
from autocycle_extras.msg import Object, ObjectList
from autocycle_extras.srv import GetData, ObjectDetectionList

height = 10000              # vertical dimension in millimeters
width = 20000               # horizontal dimension in millimeters
object_length = 1000		# object length

new_objects = []
objects = []
heading = 0
time = 0
iden = 0
iden2 = -1

def static_object_tracking():
    global objects, heading, time, new_objects, iden, iden2

    # Registers the Node with the Master
    rospy.init_node("static_object_tracking")

    # Waits for data getter service
    rospy.wait_for_service('get_data')
    
    # Waits for the object list getter service
    rospy.wait_for_service("object_list_getter")

    # Creates the service client that will collect data
    data_getter = rospy.ServiceProxy("get_data", GetData)
    
    # Creates the service client that will get the object lists
    obj_lst_getter = rospy.ServiceProxy("object_list_getter", ObjectDetectionList)

    # Publishes objects to path topic
    pub = rospy.Publisher("cycle/object_frame", ObjectList, queue_size=1)

    heading = data_getter("heading").data
    time = data_getter("met").data

    while not new_objects:
        new_objects = obj_lst_getter(iden2)
        print("I GOT THE OBJECTS")
        
    iden2 = new_objects.iden
    objects = new_objects.obj_lst.copy()
    while not rospy.is_shutdown():
        new_objects = obj_lst_getter(iden2)
        #for o in new_objects.obj_lst:
        #    print(f"OBJECT: ({o.x1}, {o.x2}, {o.z1}, {o.z2})")
        if new_objects.obj_lst:
            objects.extend(new_objects.obj_lst)
            iden2 = new_objects.iden
            print("I GOT THE OBJECTS")
        
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
            rotated = rotation_matrix @ np.array([[obj.x1, obj.z1], [obj.x2, obj.z2]])
            new.append(Object(rotated[0,0], rotated[1,0] - distance, rotated[0,1], rotated[1,1] - distance))
        objects = list(filter(lambda x: x.z1 >= 0 or x.z2 >= 0, new))
        for o in objects:
            print(f"OBJECT : ({o.x1}, {o.x2}, {o.z1}, {o.z2})")
        pub.publish(objects, iden)
        iden += 1


