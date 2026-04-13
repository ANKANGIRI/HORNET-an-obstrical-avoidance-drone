import numpy as np
import threading
import time
import math
from rplidar import RPLidar
import matplotlib.pyplot as plt

# Constants
BOARD_SIZE = 202
CENTER = BOARD_SIZE // 2
SECTOR_COUNT = 180  
C = 1  
K = 5  

# LiDAR device path
PORT_NAME = "/dev/tty.usbserial-0001"  # Update with actual port

# Global variables for threading
latest_scan = []
scan_lock = threading.Lock()
running = True  # Flag to stop the LiDAR properly

# Initialize board
board = np.zeros((BOARD_SIZE, BOARD_SIZE), dtype=float)

# LiDAR Data Collection Function
def collect_lidar_data():
    """Thread function to continuously collect LiDAR data."""
    global latest_scan, running

    lidar = RPLidar(PORT_NAME, baudrate=256000)
    
    try:
        temp_scan = []
        for scan in lidar.iter_scans():
            for (strength, angle, distance) in scan:
                temp_scan.append((strength,angle, distance))
                
            # Once a full revolution is collected, update shared variable
            with scan_lock:
                latest_scan = temp_scan.copy()  # Store only latest revolution

            temp_scan = []  # Reset for next revolution

            if not running:
                break  # Stop loop when flag is False
    
    except Exception as e:
        print(f"LiDAR Error: {e}")
        lidar.stop_motor()
        lidar.stop()
        lidar.disconnect()
    
    finally:
        print("Stopping LiDAR...")
        lidar.stop_motor()
        lidar.stop()
        lidar.disconnect()

# Certainty Grid Update Function
def update_certainity_grid(lidar_data):
    """Updates the certainty grid based on LiDAR readings."""
    global board
    #board.fill(0)  # Reset board

    for _, angle, distance in lidar_data:
        theta = math.radians(angle)
        distance = distance/10
        x = int(CENTER + distance * math.cos(theta))
        y = int(CENTER + distance * math.sin(theta))

        if 0 <= x < BOARD_SIZE and 0 <= y < BOARD_SIZE:
            
            board[x, y] += 2  
            if board[x,y]>50:
                board[x,y]=50

        for i in range(1, int(distance)):
            xi = int(CENTER + i * math.cos(theta))
            yi = int(CENTER + i * math.sin(theta))
            if 0 <= xi < BOARD_SIZE and 0 <= yi < BOARD_SIZE:
                board[xi, yi] *= (i / distance)  

# Polar Histogram Update Function
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
                    histogram[sector] += C * (board[i, j] ** 2) * (max(0, CENTER - distance))  

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

# Histogram Plotting Function
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

# Main Processing Function
def process_lidar_data():
    """Processes the LiDAR data and runs the algorithm."""
    global running
    try:
        while running:
            with scan_lock:
                scan_data = latest_scan.copy()  

            if scan_data:
                print(f"Processing {len(scan_data)} points")  
                update_certainity_grid(scan_data)
                histogram = update_polar_histogram()
                direction, magnitude = find_force_vector(histogram)
                print(f"Force Direction: {direction:.2f} degrees, Magnitude: {magnitude:.2f}")
                plot_histogram(histogram, direction, magnitude)

            time.sleep(0.1)  

    except KeyboardInterrupt:
        print("\nCTRL+C detected. Stopping LiDAR processing...")
        running = False  

# Start the LiDAR collection thread
lidar_thread = threading.Thread(target=collect_lidar_data)
lidar_thread.start()

# Run the main processing function
process_lidar_data()

# Clean up
lidar_thread.join()
plt.ioff()
plt.show()
