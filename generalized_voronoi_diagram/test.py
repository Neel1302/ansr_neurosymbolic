from voronoi.voronoi import GeneralizedVoronoi
from voronoi.geometry import *
from voronoi.astar import Astar
from voronoi.image import PolygonDetector
import cv2
import matplotlib.pyplot as plt
from datetime import datetime


def print_result(file, start, end, result, show=True):
    print('|----------------------------------------------------------|')
    print('| ' + str(file + ' - ' + str(start) + ' > ' + str(end)).ljust(56) + ' |')
    print('|----------------------------------------------------------|')
    print('| Fig | epsilon | seconds | vertices | distance | obstacle |')
    print('|-----|---------|---------|----------|----------|----------|')
    for i in range(len(result)):
        ele = result[i]
        print('| ' + str(i+1).rjust(3) + ' |  '
                    + '{:1.4f}'.format(ele[0]) + ' | ' 
                    + str(ele[1].seconds).rjust(3) + "." 
                    + str(int(ele[1].microseconds / 1000)).zfill(3) + ' | '
                    + str(ele[2]).rjust(8) + ' | '
                    + '{:1.6f}'.format(ele[3]) + ' | '
                    + '{:1.6f}'.format(ele[4]) + ' | ')
    print('|----------------------------------------------------------|')
    if show:
        plt.show()


def image_detect(path, start, end, show=True):
    # boundary
    b1 = Line([[0.0, 0.0], [1.0, 0.0]])
    b2 = Line([[1.0, 0.0], [1.0, 1.0]])
    b3 = Line([[1.0, 1.0], [0.0, 1.0]])
    b4 = Line([[0.0, 0.0], [0.0, 1.0]])

    # polygon detector
    pd = PolygonDetector(path, [214, 255])
    polygons = pd.run(bound=[1.0, 1.0])
    print("Deteced polygons #", len(polygons))
    #for polygon in polygons:
        #print(polygon)
#        area = cv2.contourArea(polygon)
#        print(area)
    if show:
        pd.generate_plot()
        pd.show()
    pd.save()

    # voronoi
    vor = GeneralizedVoronoi()
    vor.add_polygons(polygons)
    #vor.add_polygon(polygons[0])
    #vor.add_polygon(polygons[1])
    #vor.add_polygon(polygons[4])
    #vor.add_polygon(polygons[6])
    vor.add_boundaries([b1, b2, b3, b4])
    #vor.add_points([start, end])

    return vor


def voronoi(epsilon, vor, start, end, result, show=True):
    GeneralizedVoronoi.rdp_epsilon = epsilon
    start_t = datetime.now()

    # voronoi
    vor_result = vor.run_optimized()
    #liuc
    vor.generate_plot()
    vor.show()
#    exit()

    # astar
    astar = Astar(vor_result, start, end)
    astar_result = astar.run()
    
    end_t = datetime.now()
    if show:
        astar.generate_plot()
    vertices_count = len(vor_result.vertices)

    # distance
    dist = total_distance(astar_result)
    dist_obt = min_distance_from_obstacle(vor_result)

    result.append((epsilon, end_t - start_t, vertices_count, dist, dist_obt))
    print('epsilon ' + str(epsilon) + ' done.')


if __name__ == '__main__':
    Line.point_distance = 0.015
    Triangle.distance_trash = 0.01

    map = 7 

    if map == 0:
        PolygonDetector.rdp_epsilon = 0.01
        PolygonDetector.area_threshold = 400
        PolygonDetector.gray_thresh_boundary = 3
    elif map == 1:
        PolygonDetector.rdp_epsilon = 0.025
        PolygonDetector.area_threshold = 400
        PolygonDetector.gray_thresh_boundary = 5
    elif map == 2:
        PolygonDetector.rdp_epsilon = 0.025
        PolygonDetector.area_threshold = 400
        PolygonDetector.gray_thresh_boundary = 5
    elif map == 3:
        PolygonDetector.rdp_epsilon = 0.01
        PolygonDetector.area_threshold = 400
        PolygonDetector.gray_thresh_boundary = 5
    elif map == 4:
        PolygonDetector.rdp_epsilon = 0.002
        PolygonDetector.area_threshold = 400
        PolygonDetector.gray_thresh_boundary = 5
    elif map == 5:
        PolygonDetector.rdp_epsilon = 0.005
        PolygonDetector.area_threshold = 400
        PolygonDetector.gray_thresh_boundary = 5
    elif map == 6:
        PolygonDetector.rdp_epsilon = 0.001
        PolygonDetector.area_threshold = 400
        PolygonDetector.gray_thresh_boundary = 5
    elif map == 7:
        PolygonDetector.rdp_epsilon = 0.001
        PolygonDetector.area_threshold = 800
        PolygonDetector.gray_thresh_boundary = 5

    path = './testdata/'
    #file = 'map' + str(map) + '.png'
    file = 'city_map.png'
    start_points = [[0.05, 0.05], [0.05, 0.95], [0.55, 0.05], [0.05, 0.6]]
    end_points = [[0.9, 0.9], [0.95, 0.05], [0.4, 0.95], [0.95, 0.45]]

    for i in range(len(start_points)):
        if map == 4:
            start = [0.1, 0.075]
            end = [0.1, 0.9]
        elif map == 5:
            start = [0.05, 0.05]
            end = [0.5, 0.5]
        elif map == 6:
            start = [0.05, 0.95]
            end = [0.95, 0.05]
        else:
            start = start_points[i]
            end = end_points[i]

        vor = image_detect(path+file, start, end, True)
        result = []

        #voronoi(0.0000, vor, start, end, result, True)
        #voronoi(0.0002, vor, start, end, result, True)
        #voronoi(0.0004, vor, start, end, result, True)
        #voronoi(0.0008, vor, start, end, result, True)
        #voronoi(0.0016, vor, start, end, result, True)
        #voronoi(0.0032, vor, start, end, result, True)
        voronoi(0.0064, vor, start, end, result, True)
        #voronoi(0.0128, vor, start, end, result, True)
        #voronoi(0.0256, vor, start, end, result, True)
        #voronoi(0.0500, vor, start, end, result, True)

        #vor.generate_plot_only_points()
        print_result(file, start, end, result, False)
        plt.show()

        #if map > 3: break
    
        break

    plt.show()
    plt.savefig('plot.png')

