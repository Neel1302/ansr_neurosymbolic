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

def extract_corners_contours(image_path):
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, threshold1=100, threshold2=200)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    points_list = []
    for contour in contours:
        for point in contour:
            points_list.append(point[0])

    points_array = np.array(points_list)

    print(points_array)

    image_contours = image.copy()
    cv2.drawContours(image_contours, contours, -1, (0, 255, 0), 2)

    # Display the result
    cv2.imshow('Detected Edges', image_contours)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return points_array

def extrat_border_countors(image_path):
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, tresholded = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(tresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour_points = []
    for contour in contours:
        for point in contour:
            contour_points.append(point[0])
    
    contour_points = np.array(contour_points)
    
    #return 2D array of points
    #output_border = contours.reshape(contours.shape[0], 2)
    contour_image = image.copy()
    cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 3)

    plt.figure(figsize=(10, 10))
    plt.imshow(cv2.cvtColor(contour_image, cv2.COLOR_BGR2RGB))
    plt.title('Image with Contours')
    plt.axis('off')
    plt.show()

    #returns list of border points
    print(contour_points.shape)
    print(contour_points)
    return np.array(contour_points)

def main(image_path):
    points = extract_corners_canny(image_path)
    print(f'Number of points: {len(points)}')
    print(f'type:"{type(points)}"')
    #plot_voronoi_on_image(image_path, points)

# Usage
image_path = os.path.join(os.getcwd(), 'images' ,'voronoi_d0.png')
#main(image_path
#extract_corners_canny(image_path)
#extract_corners_contours(image_path)
extrat_border_countors(image_path)