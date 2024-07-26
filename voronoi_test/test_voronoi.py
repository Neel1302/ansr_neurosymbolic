import cv2
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import os

def extract_points_from_image(image_path, max_points=10000):
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    edges = cv2.Canny(image, 100, 200)
    corners = cv2.goodFeaturesToTrack(edges, maxCorners=0, qualityLevel=0.5, minDistance=10)
    corners = np.int0(corners)
    points = np.column_stack(np.nonzero(edges))

    image_color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

    for corner in corners:
        x, y = corner.ravel()
        cv2.circle(image_color, (x, y), 5, (0, 255, 0), -1)

    # Plot results
    plt.figure(figsize=(15, 5))

    plt.subplot(1, 3, 1)
    plt.imshow(cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB))
    plt.title('Original Image')
    plt.axis('off')

    plt.subplot(1, 3, 2)
    plt.imshow(cv2.cvtColor(edges_colored, cv2.COLOR_BGR2RGB))
    plt.title('Edges')
    plt.axis('off')

    plt.subplot(1, 3, 3)
    plt.imshow(cv2.cvtColor(image_color, cv2.COLOR_BGR2RGB))
    plt.title('Corners')
    plt.axis('off')

    plt.show()
    
    if len(points) > max_points:
        indices = np.random.choice(points.shape[0], max_points, replace=False)
        points = points[indices]
    
    return points

def plot_voronoi_on_image(image_path, points):
    vor = Voronoi(points)
    image = cv2.imread(image_path)
    
    for simplex in vor.ridge_vertices:
        simplex = np.asarray(simplex)
        if np.all(simplex >= 0):
            cv2.line(image, tuple(vor.vertices[simplex[0]].astype(int)),
                     tuple(vor.vertices[simplex[1]].astype(int)), (0, 255, 0), 1)
    
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.show()

def main(image_path):
    points = extract_points_from_image(image_path)
    print(f'Number of points: {len(points)}')
    print(f'type:"{type(points)}"')
    #plot_voronoi_on_image(image_path, points)

# Usage
image_path = os.path.join(os.getcwd(), 'images' ,'city_map.png')
#image_path = os.path.join(os.getcwd(), 'images' ,'voronoi_d0.png')
main(image_path)
