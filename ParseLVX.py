import struct

with open("lidar.lvx", "rb") as f:
    # Validation
    valid = ""
    for ind in range(10):
        valid += str(f.read(1))[2]
    if valid != "livox_tech":
        raise ValueError("Uh oh bad file")
    f.read(15)

    # Broadcast Code for our LiDAR Unit. Can be found on the back of Lidar
    broadcast_code = str(f.read(16))[16:-1]
    f.read(2)

    # Initial Sensor Readings
    init_roll = struct.unpack("f", f.read(4))[0]
    init_pitch = struct.unpack("f", f.read(4))[0]



