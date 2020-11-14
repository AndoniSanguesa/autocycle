import struct
import os

def check_status(status_code):
    print("---------------------------------------------")
    print("              STATUS REPORT                  ")
    print("---------------------------------------------")
    # Check Temperature
    if 2**0 & status_code:
        print("Temperature : HIGH")
    elif 2**1 & status_code:
        print("Temperature : EXTREMELY HIGH OR LOW")
    else:
        print("Temperature : GOOD")

    # Check Voltage
    if 2**2 & status_code:
        print("Voltage : HIGH")
    elif 2**3 & status_code:
        print("Voltage : EXTREMELY HIGH")
    else:
        print("Voltage : GOOD")

    # Motor Status
    if 2**4 & status_code:
        print("Motor : WARNING")
    elif 2**5 & status_code:
        print("Motor : CRITICAL ERROR. UNABLE TO OPERATE.")
    else:
        print("Motor : GOOD")

    # Dirty Warning
    if 2**6 & status_code:
        print("Sensor : DIRTY")
    else:
        print("Sensor : CLEAN")

    # Firmware Status
    if 2**8 & status_code:
        print("Firmware : Please Update")
    else:
        print("Firmware : OK")

    # PPS Status (Pulse Synchronization)
    if 2**9 & status_code:
        print("PPS : OK")
    else:
        print("PPS : NO PPS SIGNAL")

    # Device Status
    if 2**10 & status_code:
        print("Device : APPROACHING END OF SERVICE LIFE")
    else:
        print("Device : GOOD")

    # Fan Status
    if 2**11 & status_code:
        print("Fan : WARNING")
    else:
        print("Fan : GOOD")

    # Self Heating. LiDAR will heat itself under cold temperatures
    if 2**12 & status_code:
        print("Self Heating : OFF")
    else:
        print("Self Heating : ON")

    # PTP Status (Used for synchronization)
    if 2**13 & status_code:
        print("PTP : 1588 SIGNAL OK")
    else:
        print("PTP : NO 1588 SIGNAL")

    # Time Synchronization
    if 2**16 & status_code:
        print("Time Sync : ABNORMAL")
    elif (2**14 & status_code) and (2**15 & status_code):
        print("Time Sync : USING PPS SYNCHRONIZATION")
    elif 2**15 & status_code:
        print("Time Sync : USING GPS SYNCHRONIZATION")
    elif 2**14 & status_code:
        print("Time Sync : USING PTP 1588 SYNCHRONIZATION")
    else:
        print("Time Sync : SYSTEM DOES NOT START TIME SYNCHRONIZATION")

    # System Status Summary
    if 2**30 & status_code:
        print("System Summary : WARNING")
    elif 2**31 & status_code:
        print("System Summary : ERROR")
    else:
        print("System Summary : NORMAL")
    print("---------------------------------------------")


with open("lidar.lvx", "rb") as f:
    # Validation
    if str(struct.unpack("10s", f.read(10))[0])[2:-1] != "livox_tech":
        raise ValueError("Uh oh bad file")

    # Garbage validation stuff that we dont need
    f.read(14)

    # Duration of each frame in ms
    frame_dur = struct.unpack("I", f.read(4))

    # Number of devices
    device_count = struct.unpack("B", f.read(1))

    # Broadcast Code for our LiDAR Unit. Can be found on the back of Lidar
    broadcast_code = str(f.read(16))[16:-1]

    # Hub SN Code. Garbage value since we're not using a Hub
    f.read(16)

    # Device Information. Index should be 0. type should be 3.
    device_index = struct.unpack("B", f.read(1))[0]
    device_type = struct.unpack("B", f.read(1))[0]

    # Extrinsic Parameters Toggle
    ext_tog = struct.unpack("B", f.read(1))[0]

    # Initial Sensor Readings
    # Roll, Pitch, and Yaw --> Degrees
    # X, Y, Z --> Meters
    init_roll = struct.unpack("f", f.read(4))[0]
    init_pitch = struct.unpack("f", f.read(4))[0]
    init_yaw = struct.unpack("f", f.read(4))[0]
    init_x = struct.unpack("f", f.read(4))[0]
    init_y = struct.unpack("f", f.read(4))[0]
    init_z = struct.unpack("f", f.read(4))[0]

    # Grab the last location of the file to know when to stop reading
    cur_loc = f.tell()
    f.seek(0, os.SEEK_END)
    end_loc = f.tell()
    f.seek(cur_loc)

    # Reads all frames in the file
    while f.tell() < end_loc:
        # Absolute offset of the current frame in this file
        current_offset = struct.unpack("q", f.read(8))[0]

        # Absolute offset of the next frame in this file
        next_offset = struct.unpack("q", f.read(8))[0]

        # Index of frame
        frame_ind = struct.unpack("q", f.read(8))[0]

        # Index of device
        device_frame_index = struct.unpack("B", f.read(1))[0]

        # Package Protocol Version. Should be 5 (as of 11-14-20)
        ppv = struct.unpack("B", f.read(1))[0]

        # Garbage value. Slot ID used only in Mid-40. We're utilizing the Horizon
        f.read(1)

        # LiDAR ID. Should be 1
        lidar_id = struct.unpack("B", f.read(1))[0]

        # Absolute garbage value. Does not encode any data
        f.read(1)

        # Status Code. If you want to analyze the code, call `check_status()` with the code as the param.
        stat_code = struct.unpack("I", f.read(4))[0]

        # Time Stamp Type
        # 0 : No Sync Source (unit : ns)
        # 1 : PTP            (unit : ns)
        # 2 : Reserved
        # 3 : GPS            (UTC)
        # 4 : PPS            (unit : ns)
        time_stamp_type = struct.unpack("B", f.read(1))[0]

        # Data Type
        # 0 : Cartesian Coordinate System; Single Return; (Only for Livox Mid)
        # 1 : Spherical Coordinate System; Single Return; (Only for Livox Mid)
        # 2 : Cartesian Coordinate System; Single Return;
        # 3 : Spherical Coordinate System; Single Return;
        # 4 : Cartesian Coordinate System; Double Return;
        # 5 : Spherical Coordinate System; Double Return;
        # 6 : IMU Information
        data_type = struct.unpack("B", f.read(1))[0]

        # Time Stamp (Check value of `time_stamp_type` above for kind of time stamp)
        # Frankly how this time stamp works is confusing and I don't think we need it so I'm just gonna
        # skip the bytes
        f.read(8)  # time_stamp = list(struct.unpack("B", f.read(1))[0] for _ in range(8))

        # Gets Point Cloud/IMU Data
        # Cartesian Coordinate System; Single Return
        if data_type == 2:
            for _ in range(96):
                # (x, y, z)'s unit --> mm
                x = struct.unpack("i", f.read(4))[0]
                y = struct.unpack("i", f.read(4))[0]
                z = struct.unpack("i", f.read(4))[0]

                # Documentation does mention unit type
                reflectivity = struct.unpack("B", f.read(1))[0]

                # Used for determining noise in data. Don't think we need it, but can revisit later
                tag = struct.unpack("B", f.read(1))[0]

                """TODO: Store this information in a the data structure of your choosing"""
        # IMU Data
        elif data_type == 6:
            # (gyro_x, gyro_y, gyro_z)'s unit --> rad/s
            gyro_x = struct.unpack("f", f.read(4))[0]
            gyro_y = struct.unpack("f", f.read(4))[0]
            gyro_z = struct.unpack("f", f.read(4))[0]

            # (acc_x, acc_y, acc_z)'s unit --> g
            acc_x = struct.unpack("f", f.read(4))[0]
            acc_y = struct.unpack("f", f.read(4))[0]
            acc_z = struct.unpack("f", f.read(4))[0]
            """TODO: Store this information in a the data structure of your choosing"""
        f.seek(next_offset)

