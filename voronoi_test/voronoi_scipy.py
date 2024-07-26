import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Polygon
import os
import cv2
import generate_points

# Path to the image
image_path = os.path.join(os.getcwd(), 'images', 'voronoi_d0.png')
#image_path = os.path.join(os.getcwd(), 'images', 'city_map.png')

image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)


points = generate_points.extract_corners_canny(image_path)

all_contours = generate_points.extract_contours(image_path)



vor = Voronoi(points)

#print("Points:\n", vor.points)
#print("Vertices:\n", vor.vertices)
#print("Ridge Points:\n", vor.ridge_points)
#print("Ridge Vertices:\n", vor.ridge_vertices)
#print("Regions:\n", vor.regions)
#print("Point Region:\n", vor.point_region)

# Extract the boundary of the objects (assuming objects are non-zero pixels in the image)
contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

image_contours = image.copy()
cv2.drawContours(image_contours, contours, -1, (0, 255, 0), 2)

# Display the result
cv2.imshow('Detected Edges', image_contours)
cv2.waitKey(0)
cv2.destroyAllWindows()

boundary_polygons = [Polygon(cnt[:, 0, :]) for cnt in all_contours]

# Extract Voronoi lines and filter those that do not intersect with the boundary
lines = []
for ridge in vor.ridge_vertices:
    if -1 not in ridge:
        start_point = vor.vertices[ridge[0]]
        end_point = vor.vertices[ridge[1]]
        line = LineString([start_point, end_point])
        
        #intersects = any(line.intersects(bound) for bound in boundary_polygons)
        #if not intersects:
        #    lines.append((start_point, end_point))
        lines.append((start_point, end_point))



# Plot Voronoi diagram and lines
fig, ax = plt.subplots()
#voronoi_plot_2d(vor, ax=ax, show_vertices=False, line_colors='blue')

# Plot the extracted lines that do not intersect with the boundary
for line in lines:
    line = LineString(line)
    x, y = line.xy
    ax.plot(x, y, 'r-')

# Plot the original points
ax.plot(points[:, 0], points[:, 1], 'ko')

# Optionally, plot the boundary for visualization
for bound in boundary_polygons:
    x, y = bound.exterior.xy
    ax.plot(x, y, 'g-')

ax.imshow(image, cmap='gray')
plt.show()

