dataFile = None

## INSTRUCTIONS ##

# 1. Each array below is a piece of input data for each obstacle.
# 2. Every array below should be a 2D array. Each subarray is a different frame.
#    Within each frame, the data of each object should be inputted starting from object 1 to object n
# 3. Each frame should have the same number of data from data type to data type
#    (i.e if the first frame in gamma has two pieces of data,
#    the first frame in edge_len should have two pieces of data)
# 4. Run this file and it will create the data in TestData
# 5. Run the bezier_visualization.py file

# You don't need to worry about anything past line 28

# The furthest distance from origin that the path is rendered to
resolution = 10
# Distances from the bike to the objects
dist_to_edge = [4, 6]
# The distance from the current path to edge of the obstacle most counter clockwise from the current path
edge_to_path = [1, 4]
# Lengths of the objects
edge_len = [6, 4]

if not (
        len(dist_to_edge) == len(edge_to_path) == len(edge_len)
):
    raise (Exception, "The number of items in all arrays must be the same!")

try:
    dataFile = open("TestData", "w+")
except FileNotFoundError:
    raise (Exception, "The file could not be found, check the name.")
except:
    raise (Exception, "It is likely that the file is open somewhere else. Close it and try again.")

dataFile.write(str(resolution) + "\n")
dataFile.write(str(len(edge_len)) + " ")
for datum in range(len(edge_len)):
    dataFile.write(
        str(dist_to_edge[datum])
        + " "
        + str(edge_to_path[datum])
        + " "
        + str(edge_len[datum])
        + " "
    )
dataFile.write("\n")

print("Data Recorded!")
