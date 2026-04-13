import numpy as np
import threading
import time
import math
from rplidar import RPLidar
import matplotlib.pyplot as plt
import asyncio


BOARD_SIZE = 502 # 2.5 metre each side of drone
CENTER = BOARD_SIZE / 2
SECTOR_COUNT = 360 #updated to 1 degree per sector  
C = 100
K = 5 
V = 10 #constant magnitude velocity given for instructions
Hist_threshold = 0.2 #Threshold to determine if its safe to move in the sector or not
#Can put a dynamic threshold which is interdependent on velocity of drone
alt_change_sectors = 30 # if width of safe sectors less than this , we change altitude
critical_d = 75  #Critical protocol will be implemented if any detection less than this
#also later include a lower bound as to ignore drone parts if lidar gets tilted
min_critical_gap = 90 # min gap in degrees needed in critical protocol to steer otherwise altitude change
# Global variables to store previous GPS position
prev_lat, prev_lon,prev_alt_interval = None, None,None
# LiDAR device path
PORT_NAME = "/dev/tty.usbserial-0001"  # Update with actual port

# Global variables for threading
latest_scan = []
scan_lock = threading.Lock()
running = True  # Flag to stop the LiDAR properly
drone = None
# Initialize board
board = np.zeros((BOARD_SIZE, BOARD_SIZE), dtype=float)


def collect_lidar_data():
      pass


def update_certainity_grid(lidar_data):
    """Updates the certainty grid based on LiDAR readings."""
    global board
    #board.fill(0)  # Reset board
    critical_angles = []
    for _, angle, distance in lidar_data:
        with open("lidar_data.txt", "a") as file:
            file.write(f"({angle},{distance})\n")

        theta = math.radians(angle)
        distance = distance/10 #converting mm to cm
        if distance<=critical_d:
            critical_angles.append(angle)
        x = int(CENTER + distance * math.cos(theta))
        y = int(CENTER + distance * math.sin(theta))

        if 0 <= x < BOARD_SIZE and 0 <= y < BOARD_SIZE:
            
            board[x,y] += 2  
            if board[x,y]>250:
                 board[x,y]=250 # !! CREATING THIS CAP IS SOMEHOW MAKING THE OUTPUT FLICKER (but at 500 flicker reduced)

        for i in range(1, int(distance)):
            xi = int(CENTER + i * math.cos(theta))
            yi = int(CENTER + i * math.sin(theta))
            if 0 <= xi < BOARD_SIZE and 0 <= yi < BOARD_SIZE:
                # board[xi, yi] *= (i / distance)  
                board[xi,yi]=0 #trying to reduce obstacle certainity at closer points
    
    return critical_angles


def update_polar_histogram():
    """Computes and smooths the 1D polar histogram."""
    histogram = np.zeros(SECTOR_COUNT, dtype=float)

    for i in range(BOARD_SIZE):
        for j in range(BOARD_SIZE):
            if board[i, j] > 0:
                x = i - CENTER
                y = j - CENTER
                distance = math.sqrt(x**2 + y**2)
                sector = int((math.degrees(math.atan2(y, x)) + 360) % 360 / (360 / SECTOR_COUNT))

                if distance > 0:
                    #different variations to control dominance
                    # histogram[sector] += C * (board[i, j] ** 2) * (max(0, CENTER - distance))  
                    # histogram[sector] += C * (board[i, j] ** 2)/distance 
                    histogram[sector] += C * board[i, j]/(distance**2) #motivated by nature 


    smoothed_histogram = np.zeros(SECTOR_COUNT, dtype=float)
    for i in range(SECTOR_COUNT):
        total = 0
        weight_sum = 0
        for offset in range(-K, K + 1):
            index = (i + offset) % SECTOR_COUNT  
            weight = K + 1 - abs(offset)
            total += histogram[index] * weight
            weight_sum += weight
        smoothed_histogram[i] = total / weight_sum if weight_sum > 0 else 0

    return smoothed_histogram


# Force Vector Calculation Function
def find_force_vector(histogram):
    """Finds the net force vector based on the polar histogram."""
    net_x, net_y = 0, 0
    for i in range(SECTOR_COUNT):
        angle = math.radians(180 + (i * 360 / SECTOR_COUNT))
        magnitude = histogram[i]
        net_x += magnitude * math.cos(angle)
        net_y += magnitude * math.sin(angle)

    net_angle = math.degrees(math.atan2(net_y, net_x)) % 360
    net_magnitude = 0.01 * math.sqrt(net_x**2 + net_y**2)
    return net_angle, net_magnitude



def plot_histogram(histogram, angle, magnitude):
    plt.figure(1)
    plt.clf()  
    x = np.arange(len(histogram))
    plt.bar(x * 2, histogram)
    plt.bar(int(angle), magnitude, color='green')

    plt.xlabel("Angle")
    plt.ylabel("Certainty of Obstacle")
    plt.title("Smoothed Histogram")
    plt.pause(0.1)  

    plt.figure(2)
    plt.clf()
    
    ax = plt.gca()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_frame_on(False)  
    
    angle_rad = np.radians(angle)  
    x_end = magnitude * np.cos(angle_rad)
    y_end = magnitude * np.sin(angle_rad)

    segments = [[0, 0, magnitude, angle_rad], [x_end, y_end, magnitude / 10, np.radians((angle + 205) % 360)], [x_end, y_end, magnitude / 10, np.radians((angle + 155) % 360)]]
    
    for x_start, y_start, length, ang in segments:
        x_end = x_start + length * np.cos(ang)
        y_end = y_start + length * np.sin(ang)
        plt.plot([x_start, x_end], [y_start, y_end], 'r-', linewidth=2)  

    plt.pause(0.1)  

def make_binary_histogram(histogram):

    max_value = max(histogram)  
    threshold = Hist_threshold * max_value  

    binary_histogram = []
    for value in histogram:
        if value >= threshold:
            binary_histogram.append(1)
        else:
            binary_histogram.append(0)

    return np.array(binary_histogram)


def plot_binary_histogram(binary_histogram):

    binary_histogram = np.array(binary_histogram)
    angles = np.arange(len(binary_histogram))  # Angles from 0 to 359

    plt.figure(figsize=(10, 4))
    plt.bar(angles, binary_histogram, width=1.0, color=['green' if val == 1 else 'red' for val in binary_histogram])
    plt.xlabel("Angle (degrees)")
    plt.ylabel("Safty")
    plt.title("Binary Histogram")
    plt.ylim(0, 1.2)
    plt.grid(axis='y')
    plt.tight_layout()
    plt.show()





