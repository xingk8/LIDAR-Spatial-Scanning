import serial                 # Handles serial communication with microcontroller
import math                   # Used to convert angles from degrees to radians and perform trig
import numpy as np            # For handling point arrays
import open3d as o3d          # For 3D point cloud visualization

#-- Discuss this for showing how UART connects to the python script--
# === Serial Setup ===
s = serial.Serial(port='COM5', baudrate=115200, timeout=10)  # Set up serial connection (make sure COM port matches)
print("Opening: " + s.name)

# Reset any leftover data in UART buffers
s.reset_output_buffer()
s.reset_input_buffer()

# Wait for user input to begin scanning
input("Press Enter to start communication...")
s.write(b's')  # Tell microcontroller to begin sending distance data

#-------------------------------------------------------------------------
# === Scan Parameters ===
angle = 0              # Current angle in degrees (starts at 0°)
z = 0                  # Z-layer height (in mm or units)
numScans = 2           # Number of vertical layers (i.e., how many times the sensor completes a circle while moving up) TO CHANGE SCANS
numSteps = 32          # Number of angle steps per full rotation (i.e., 360° / 11.25° = 32 steps)
valid_points = []      # Stores final computed XYZ points

# Open output file to save coordinates for external use (e.g., MeshLab, CloudCompare)
with open("coordinates.xyz", "w") as f:
    i = 0
    started = False  # Don't increment index until the first valid distance reading

    while i < numScans * numSteps:
        x = s.readline().decode().strip()  # Read a line from the serial port and decode it
        print("RAW:", x)

        if not x.isdigit():
            print("Skipping non-distance value:", x)  # Skip any invalid messages (e.g., boot info, END)
            continue

        distance = int(x)

        if not started:
            print("=== First valid reading detected. Starting scan indexing ===")
            started = True

        # Convert polar to Cartesian coordinates
        angle_rad = math.radians(angle)
        x_val = round(distance * math.cos(angle_rad), 4)
        y_val = round(distance * math.sin(angle_rad), 4)
        z_val = z  # Z height stays the same for the current rotation

        # Store the 3D point
        valid_points.append([x_val, y_val, z_val])
        f.write(f"{x_val} {y_val} {z_val}\n")  # Write to file

        # Update angle and vertical position
        angle += 11.25  # Each step is 11.25°
        if angle >= 360:
            angle = 0  # Reset for new circle

        if (i + 1) % numSteps == 0:  # After 1 full circle
            z += 250  # Move up 250 units

        print("Distance:", distance, "Index:", i)
        i += 1

# Done receiving data
s.close()

# === Visualization using Open3D ===
pts = np.array(valid_points)  # Convert list to NumPy array
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts)  # Load points into the Open3D point cloud

# === Connecting points with lines ===
lines = []

# Connect each scan "slice" in a circular fashion
for z in range(numScans):
    offset = numSteps * z
    for i in range(numSteps):
        next_i = i + 1 if i < numSteps - 1 else 0  # Wrap around
        lines.append([i + offset, next_i + offset])

# Connect the same points across different layers vertically
for z in range(numScans - 1):
    base = z * numSteps
    for i in range(numSteps):
        lines.append([base + i, base + i + numSteps])

# Create a line set object for rendering
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(pts),
    lines=o3d.utility.Vector2iVector(lines),
)

# Display the point cloud in a window
print("Rendering point cloud...")
o3d.visualization.draw_geometries([line_set], width=1280, height=720)
