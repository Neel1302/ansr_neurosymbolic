import cv2
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import os

def extract_corners_canny(image_path, max_points=10000):
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    edges = cv2.Canny(image, 100, 200)
    corners = cv2.goodFeaturesToTrack(edges, maxCorners=0, qualityLevel=0.1, minDistance=5)
    edge_points = cv2.findNonZero(edges)

    print("Corners shape: ", corners.shape)
    print("Edge points shape: ", edge_points.shape)
    
    output_corners = corners.reshape(corners.shape[0], 2)
    output_edge_points = edge_points.reshape(edge_points.shape[0], 2)

    # Draw points
    plt.figure(figsize=(10, 10))
    plt.imshow(image, cmap='gray')
    #plt.scatter(output_corners[:, 0], output_corners[:, 1], c='r', s=10)
    plt.scatter(output_edge_points[:, 0], output_edge_points[:, 1], c='r', s=10)
    plt.title('Edges detected by Canny')
    plt.axis('off')
    plt.show()
    
    
    return output_edge_points

def extract_contours(image_path):
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, threshold1=100, threshold2=200)
    _, thresholded = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    bound_contours, _ = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    all_contours = contours + bound_contours
    valid_contours = [contour for contour in all_contours if len(contour) >= 4]

    points_list = []
    for contour in all_contours:
        for point in contour:
            points_list.append(point[0])

    points_array = np.array(points_list)

    print(points_array)

    image_contours = image.copy()
    cv2.drawContours(image_contours, valid_contours, -1, (0, 255, 0), 2)
    

    # Display the result
    cv2.imshow('Detected Edges', image_contours)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return valid_contours

def extract_border_contours(image_path):
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply binary threshold to get binary image
    _, thresholded = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # Find external contours
    external_contours, _ = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find internal contours
    internal_contours, _ = cv2.findContours(cv2.bitwise_not(thresholded), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Extract points of all contours
    boundary_points = []
    object_points = []
    for contour in external_contours:
        for point in contour:
            boundary_points.append(point[0])
    
    for contour in internal_contours:
        for point in contour:
            object_points.append(point[0])
    
    boundary_points = np.array(boundary_points)
    object_points = np.array(object_points)

    # Draw contours for visualization
    contour_image = image.copy()
    cv2.drawContours(contour_image, external_contours, -1, (0, 255, 0), 3)  # Green for boundary
    cv2.drawContours(contour_image, internal_contours, -1, (255, 0, 0), 3)  # Blue for objects

    # Visualize the contours
    plt.figure(figsize=(10, 10))
    plt.imshow(cv2.cvtColor(contour_image, cv2.COLOR_BGR2RGB))
    plt.title('Image with Contours')
    plt.axis('off')
    plt.show()

    # Print the shapes and points for debugging
    print("Boundary Points Shape:", boundary_points.shape)
    print("Boundary Points:\n", boundary_points)
    print("Object Points Shape:", object_points.shape)
    print("Object Points:\n", object_points)

    # Return the list of boundary points and object points
    return boundary_points, object_points

def main(image_path):
    points = extract_corners_canny(image_path)
    print(f'Number of points: {len(points)}')
    print(f'type:"{type(points)}"')
    #plot_voronoi_on_image(image_path, points)

# Usage
#image_path = os.path.join(os.getcwd(), 'images' ,'voronoi_d0.png')
image_path = os.path.join(os.getcwd(), 'images' ,'city_map.png')
#main(image_path)
#extract_corners_canny(image_path)
extract_contours(image_path)
#extract_border_contours(image_path)