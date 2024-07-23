from voronoi.image import PolygonDetector
from voronoi.geometry import *

import pyvisgraph as vg
import cv2
import os

image_path = os.path.join(os.getcwd(), 'images', 'voronoi_d0.png')

#get polygons
def get_polygons(image_path):
    # Load the image    
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    # Create a PolygonDetector object
    detector = PolygonDetector(image)
    # Detect polygons
    polygons = detector.detect_polygons()
    return polygons