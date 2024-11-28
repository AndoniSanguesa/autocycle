import rosbag


def start():
    for topic, msg, t in rosbag.Bag('input.bag').read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            print(topic, msg, msg.transforms[0].header.stamp)
        else:
            print(topic, msg, msg.header.stamp if msg._has_header else t)
