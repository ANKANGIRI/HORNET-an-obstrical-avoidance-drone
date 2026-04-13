import numpy as np
import time
import math
import matplotlib.pyplot as plt


# Constants
BOARD_SIZE = 202
CENTER = BOARD_SIZE // 2
SECTOR_COUNT = 180  # Number of 2-degree sectors
C = 1  # Arbitrary constant for histogram calculation
K = 5  # Smoothing constant

# Initialize board and histogram
board = np.zeros((BOARD_SIZE, BOARD_SIZE), dtype=float)

def read_lidar_data(filename):
    """Reads LiDAR data from a file and returns a list of (angle, distance)."""
    lidar_data = []
    with open(filename, 'r') as file:
        for line in file:
            parts = line.split()
            if len(parts) < 2:
                continue  # Skip invalid lines
            angle = round(float(parts[0]), 1)  # Round to 1 decimal place
            distance = round(float(parts[1]), 1)  # Round to 1 decimal place
            distance= distance/10
            lidar_data.append((angle, distance))
            # print("angle : " ,angle ,"distance ",distance)
            #print(f"angle: {angle} distance: {distance}")
    return lidar_data

def update_certainity_grid(lidar_data):
    """Updates the certainty grid based on LiDAR readings."""
    global board
    board.fill(0)  # Reset board

    for angle, distance in lidar_data:
        theta = math.radians(angle)
        x = int(CENTER + distance * math.cos(theta))
        y = int(CENTER + distance * math.sin(theta))

        if 0 <= x < BOARD_SIZE and 0 <= y < BOARD_SIZE:
            board[x, y] += 2  # Increment certainty at obstacle location

        # Decrease certainty along the line towards the obstacle
        for i in range(1, int(distance)):
            xi = int(CENTER + i * math.cos(theta))
            yi = int(CENTER - i * math.sin(theta))
            if 0 <= xi < BOARD_SIZE and 0 <= yi < BOARD_SIZE:
                board[xi,yi] *= (i/distance)  # Reduce certainty of obstacles at a lesser distance for this angle(movement of obstacle taken in account)


def update_polar_histogram():

    """Computes and smooths the 1D polar histogram from the certainty grid."""
    histogram = np.zeros(SECTOR_COUNT, dtype=float)

    for i in range(BOARD_SIZE):
        for j in range(BOARD_SIZE):
            if board[i, j] > 0:
                x = i - CENTER
                y = j - CENTER
                distance = math.sqrt(x**2 + y**2)
                sector = int((math.degrees(math.atan2(y, x)) + 360) % 360 / (360 / SECTOR_COUNT))

                if distance > 0:
                    histogram[sector] += C * (board[i, j] ** 2) * (max(0,CENTER-distance)) # could have also divided by distance

    # Smooth the histogram
    smoothed_histogram = np.zeros(SECTOR_COUNT, dtype=float)
    for i in range(SECTOR_COUNT):
        total = 0
        weight_sum = 0
        for offset in range(-K, K + 1):
            index = (i + offset) % SECTOR_COUNT  # Wrap around
            weight = K + 1 - abs(offset)
            total += histogram[index] * weight
            weight_sum += weight
        smoothed_histogram[i] = total / weight_sum if weight_sum > 0 else 0
        
    #plot_histogram(smoothed_histogram)
    return smoothed_histogram

def find_force_vector(histogram):
    """Finds the net force vector based on the polar histogram."""
    net_x, net_y = 0, 0
    for i in range(SECTOR_COUNT):
        angle = math.radians(180 + (i * 360 / SECTOR_COUNT))
        magnitude = histogram[i]
        net_x += magnitude * math.cos(angle)
        net_y += magnitude * math.sin(angle)

    net_angle = math.degrees(math.atan2(net_y, net_x)) % 360
    net_magnitude = 0.01*math.sqrt(net_x**2 + net_y**2)
    return net_angle, net_magnitude

def plot_histogram(histogram,angle,magnitude):
    plt.figure(1)
    plt.clf()  # Clear previous plot
    x = np.arange(len(histogram))

    # Plot bar chart
    plt.bar(x*2, histogram)
    plt.bar(int(direction),magnitude,color='green')

    # Label axes
    plt.xlabel("Angle")
    plt.ylabel("Certainity of obstacle")
    plt.title("Smoothened Histogram")
    plt.pause(0.1)  # Pause to allow the plot to update

    # # Show plot
    # plt.show()


    """Plots a clear, bold arrow pointing in the given direction."""
    plt.figure(2)  # Create a separate figure
    plt.clf()

    
    ax = plt.gca()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_xticks([])  # Remove x-axis labels
    ax.set_yticks([])  # Remove y-axis labels
    ax.set_frame_on(False)  # Remove the border/frame
     # Convert angle to radians and calculate arrow components
    # angle_rad = np.radians(angle)
    # x = np.cos(angle_rad) * magnitude
    # y = np.sin(angle_rad) * magnitude

    # # Plot the arrow at the center with a larger head
    # plt.arrow(0, 0, x, y, head_width=0.2, head_length=0.2, fc='red', ec='red', linewidth=3, length_includes_head=True)
    
    angle_radd = np.radians(angle)  # Convert degrees to radians
    x_endd = magnitude* np.cos(angle_radd)
    y_endd = magnitude* np.sin(angle_radd)
    ang2 = np.radians((angle + 205)%360)
    ang3 = np.radians((angle + 155)%360)
    segments = [[0,0,magnitude,angle_radd],[x_endd,y_endd,magnitude/10,ang2],[x_endd,y_endd,magnitude/10,ang3]]
    for x_start, y_start, length, anglee in segments:
        
        x_end = x_start + length * np.cos(anglee)
        y_end = y_start + length * np.sin(anglee)

        plt.plot([x_start, x_end], [y_start, y_end], 'r-', linewidth=2)  # Red line
    # Redraw the plot

    plt.pause(0.1)

  


def print_board():
    """Prints the certainty grid."""
    print("\033c", end="")  # Clear terminal
    for row in board:
        print(" ".join(f"{int(cell):2d}" for cell in row))
    print()

def print_histogram(histogram):
    """Prints the histogram as a simple bar graph."""
    print("Histogram:")
    for i, value in enumerate(histogram):
        print(f"{i*2}: {int(value)}")

# Main Loop
i=2
plt.ion()
while i<1000:

    FILENAME = "lidar" + str(i%3 + 1) + ".txt" # Change this to your actual file
    lidar_data = read_lidar_data(FILENAME)

    update_certainity_grid(lidar_data)
    histogram = update_polar_histogram()
    direction, magnitude = find_force_vector(histogram)

    #print_board()
    print(i)
    print(f"Force Direction: {direction:.2f} degrees, Magnitude: {magnitude:.2f}")
    plot_histogram(histogram,direction,magnitude)
    #time.sleep(1)  # Adjust refresh rate
    i+=3

# Keep the final plot visible
plt.ioff()  # Turn off interactive mode
plt.show()
