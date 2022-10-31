# Script to parse text files with data points to incorporate into the Phantom X movement
# Text file should have the format
# Target:= [x,y,z],
from fileinput import filename

def getTrajectoryFromTextFile(filename):
    with open(filename, 'r') as fobj:
        trajectory = []
        for line in fobj:
            coords_str = line.split()[-1][1:-2].split(",")
            coords = [float(n) for n in coords_str]
            trajectory.append(coords)

    return trajectory


# if __name__ == "__main__":
#     file_names = ["circle.txt", "custom_part.txt", "d_letter.txt",
#                     "inner_ring.txt", "line123.txt", "outter_ring.txt",
#                     "point1to5.txt", "s_letter.txt", "triangle.txt"]

#     trajectory = getTrajectoryFromTextFile(file_names[0])
#     print(trajectory)