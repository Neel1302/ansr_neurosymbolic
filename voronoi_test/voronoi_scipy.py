import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
from shapely.geometry import LineString
import os
import cv2

import generate_points

#path
image_path = os.path.join(os.getcwd(), 'images' ,'voronoi_d0.png')
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
#from gerate_points.py
points = generate_points.extract_corners_canny(image_path)
points = np.vstack((points, generate_points.extrat_border_countors(image_path)))
# Generate Voronoi diagram
vor = Voronoi(points)

# Extract Voronoi lines
lines = []
for ridge in vor.ridge_vertices:
    if -1 not in ridge:
        start_point = vor.vertices[ridge[0]]
        end_point = vor.vertices[ridge[1]]
        lines.append((start_point, end_point))

# Plot Voronoi diagram and lines
fig, ax = plt.subplots()
voronoi_plot_2d(vor, ax=ax, show_vertices=False, line_colors='blue')

# Plot the extracted lines
for line in lines:
    line = LineString(line)
    x, y = line.xy
    ax.plot(x, y, 'r-')

# Plot the original points
ax.plot(points[:, 0], points[:, 1], 'ko')
ax.imshow(image, cmap='gray')


plt.show()
