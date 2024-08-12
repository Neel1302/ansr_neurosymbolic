import numpy as np
import os
import matplotlib.pyplot as plt

def read_pgm(filename):
    with open(filename, 'rb') as f:
        # Read the header
        header = f.readline().decode('ascii')
        if header.strip() != 'P5':
            raise ValueError('File is not a valid PGM (P5) file.')

        # Skip comments
        line = f.readline().decode('ascii')
        while line.startswith('#'):
            line = f.readline().decode('ascii')

        # Read width, height
        width, height = map(int, line.split())
        max_val = int(f.readline().decode('ascii'))

        # Read the pixel data
        img_data = np.fromfile(f, dtype=np.uint8 if max_val < 256 else np.uint16, count=width*height)
        img_data = img_data.reshape((height, width))

    return img_data, width, height

def get_pixel_coordinates(img_data, value):
    coordinates = np.argwhere(img_data <= value)
    return coordinates

def get_points_from_pgm(filename, value_to_find):
    image_data, width, height = read_pgm(filename)
    pixel_coordinates = get_pixel_coordinates(image_data, value_to_find)
    return pixel_coordinates, width, height


# Example usage:
filename = os.path.join('pgm','my_map_200.pgm')
value_to_find = 210  # For example, find coordinates of white pixels in a grayscale image
image_data, width, height = read_pgm(filename)
print("Image dimensions are:", width, "x", height)
print("Image data is:")
print(image_data)

#output all pixel values to text
np.savetxt('my_map_200.txt', image_data, fmt='%d')
pixel_coordinates = get_pixel_coordinates(image_data, value_to_find)
np.savetxt('my_map_coord_200.txt', pixel_coordinates, fmt='%d')

print("Pixel coordinates for value", value_to_find, "are:")
print(pixel_coordinates)

x_coords = [coord[0] for coord in pixel_coordinates]
y_coords = [coord[1] for coord in pixel_coordinates]

# Create a scatter plot
plt.scatter(x_coords, y_coords, s = 1, color='blue', marker='o')

# Optionally, you can add labels to the axes
plt.xlabel('X Coordinates')
plt.ylabel('Y Coordinates')

# Set the title of the plot
plt.title('Graph of Coordinates')

# Show grid
plt.grid(True)

# Show the plot
plt.show()
