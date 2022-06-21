'''
find skeletonization intersections code based from this github link:
https://gist.github.com/jamjar919/05a76cf21035cef3cc86ac1979588d6d
'''

import colors
import cv2 as cv
import itertools
import numpy as np
from skimage.morphology import skeletonize


# get image coords
def img_coords(img):
    rows, cols, _ = img.shape
    rows_list = np.linspace(0, rows - 1, cols)
    cols_list = np.linspace(0, cols - 1, cols)

    map_coords = list(itertools.product(rows_list, cols_list))
    map_coords = np.asarray(map_coords)  # [rows, cols]

    return map_coords.astype(int)


# color given coordinates of an image
def color_coords(img, coords, color):

    img[coords[:, 0], coords[:, 1]] = color  # replace colors


# binary color coords, white on black
def bin_color_coords(img, coords):
    img[coords[:, 0], coords[:, 1]] = 1  # replace colors


# get color coordinates in image
def get_color_coords(img, coords, color):
    # indices of where current color exists in image
    color_idx = np.where((img[coords[:, 0], coords[:, 1], :] == color).all(axis=1))[0]
    color_coords = coords[color_idx]

    return color_coords


# get approx color coordinates
def get_color_coords_approx(img, coords, color, thresh=30):
    # indices of where current color approximately exists in image
    color_idx = np.where((np.linalg.norm(img[coords[:, 0], coords[:, 1], :] -
                                         np.asarray(color), axis=1) < thresh))[0]
    color_coords = coords[color_idx]

    return color_coords


# get non color coordinates in image
def get_non_color_coords(img, coords, color):
    # indices of where current color exists in image
    color_idx = np.where((img[coords[:, 0], coords[:, 1], :] != color).all(axis=1))[0]
    color_coords = coords[color_idx]

    return color_coords


# get image coords
def img_coords(img):
    rows, cols, _ = img.shape
    rows_list = np.linspace(0, rows - 1, cols)
    cols_list = np.linspace(0, cols - 1, cols)

    map_coords = list(itertools.product(rows_list, cols_list))
    map_coords = np.asarray(map_coords)  # [rows, cols]

    return map_coords.astype(int)


# get nodes from GT image
def get_GT_nodes(full_img, scale_percent=100, node_radius=2, node_thickness=2, color_thresh=30):

    full_img_coords = img_coords(full_img)
    # road_coords = get_color_coords(full_img, full_img_coords, colors.road_color)
    road_coords = get_color_coords_approx(full_img, full_img_coords, colors.road_color, thresh=color_thresh)

    # map image
    map_img = np.zeros((full_img.shape[0], full_img.shape[1]))
    bin_color_coords(map_img, road_coords)

    # resize map image
    width = int(map_img.shape[1] * scale_percent / 100)
    height = int(map_img.shape[0] * scale_percent / 100)
    dim = (width, height)
    map_img = cv.resize(map_img, dim, interpolation=cv.INTER_AREA)
    full_img = cv.resize(full_img, dim, interpolation=cv.INTER_AREA)

    # perform skeletonization
    skeleton = skeletonize(map_img)

    # get skeleton intersections
    waypoints = getSkeletonIntersection(skeleton)

    # resize skeleton to 3 channel image (rgb)
    skeleton = np.repeat(skeleton[:, :, np.newaxis], 3, axis=2)

    # plot intersections on the image
    for waypoint in waypoints:
        skeleton = cv.circle(np.float32(skeleton), waypoint, node_radius,
                             np.asarray(colors.blue) / 255, node_thickness)

    # convert to format for visualization
    full_img = np.float32(full_img) / 255

    return full_img, skeleton, waypoints


# get nodes from velo image
def get_velo_nodes(velo_img, scale_percent=100, kernel_size_d=25, kernel_size_e=20,
                   node_radius=2, node_thickness=2, color_thresh=30):
    # read image
    # velo_img = cv.imread(img_path)

    velo_img_coords = img_coords(velo_img)
    # road_coords = get_color_coords(velo_img, velo_img_coords, colors.road_color)
    road_coords = get_color_coords_approx(velo_img, velo_img_coords, colors.road_color, thresh=color_thresh)

    # map image
    map_img = np.zeros((velo_img.shape[0], velo_img.shape[1]))
    bin_color_coords(map_img, road_coords)

    # resize map image
    width = int(map_img.shape[1] * scale_percent / 100)
    height = int(map_img.shape[0] * scale_percent / 100)
    dim = (width, height)
    map_img = cv.resize(map_img, dim, interpolation=cv.INTER_AREA)
    velo_img = cv.resize(velo_img, dim, interpolation=cv.INTER_AREA)

    # visualize.imshow_wait(map_img)

    # dilate and then erode
    kernel = np.ones((kernel_size_d, kernel_size_d), np.uint8)
    dilate_img = cv.dilate(map_img, kernel, iterations=1)

    # visualize.imshow_wait(dilate_img, name="dilate")

    kernel = np.ones((kernel_size_e, kernel_size_e), np.uint8)
    erode_img = cv.erode(dilate_img, kernel, iterations=1)

    # visualize.imshow_wait(erode_img, name="erode")

    # perform skeletonization
    skeleton = skeletonize(erode_img)

    # get skeleton intersections
    waypoints = getSkeletonIntersection(skeleton) # waypoints [col, rows]

    # resize skeleton to 3 channel image (rgb)
    skeleton = np.repeat(skeleton[:, :, np.newaxis], 3, axis=2)

    # plot intersections on the image
    for waypoint in waypoints:
        skeleton = cv.circle(np.float32(skeleton), waypoint, node_radius,
                             np.asarray(colors.blue) / 255, node_thickness)

    # convert to format for visualization
    velo_img = np.float32(velo_img) / 255

    return velo_img, skeleton, waypoints


''' test code from github: get intersections of skeletonized image '''


def get_neighbours(x, y, image):
    """Return 8-neighbours of image point P1(x,y), in a clockwise order"""
    img = image
    x_1, y_1, x1, y1 = x - 1, y - 1, x + 1, y + 1;
    return [img[x_1][y], img[x_1][y1], img[x][y1], img[x1][y1], img[x1][y], img[x1][y_1], img[x][y_1], img[x_1][y_1]]


def getSkeletonIntersection(skeleton):
    """ Given a skeletonised image, it will give the coordinates of the intersections of the skeleton.

    Keyword arguments:
    skeleton -- the skeletonised image to detect the intersections of

    Returns:
    List of 2-tuples (x,y) containing the intersection coordinates
    """
    # A biiiiiig list of valid intersections             2 3 4
    # These are in the format shown to the right         1 C 5
    #                                                    8 7 6
    validIntersection = [[0, 1, 0, 1, 0, 0, 1, 0], [0, 0, 1, 0, 1, 0, 0, 1], [1, 0, 0, 1, 0, 1, 0, 0],
                         [0, 1, 0, 0, 1, 0, 1, 0], [0, 0, 1, 0, 0, 1, 0, 1], [1, 0, 0, 1, 0, 0, 1, 0],
                         [0, 1, 0, 0, 1, 0, 0, 1], [1, 0, 1, 0, 0, 1, 0, 0], [0, 1, 0, 0, 0, 1, 0, 1],
                         [0, 1, 0, 1, 0, 0, 0, 1], [0, 1, 0, 1, 0, 1, 0, 0], [0, 0, 0, 1, 0, 1, 0, 1],
                         [1, 0, 1, 0, 0, 0, 1, 0], [1, 0, 1, 0, 1, 0, 0, 0], [0, 0, 1, 0, 1, 0, 1, 0],
                         [1, 0, 0, 0, 1, 0, 1, 0], [1, 0, 0, 1, 1, 1, 0, 0], [0, 0, 1, 0, 0, 1, 1, 1],
                         [1, 1, 0, 0, 1, 0, 0, 1], [0, 1, 1, 1, 0, 0, 1, 0], [1, 0, 1, 1, 0, 0, 1, 0],
                         [1, 0, 1, 0, 0, 1, 1, 0], [1, 0, 1, 1, 0, 1, 1, 0], [0, 1, 1, 0, 1, 0, 1, 1],
                         [1, 1, 0, 1, 1, 0, 1, 0], [1, 1, 0, 0, 1, 0, 1, 0], [0, 1, 1, 0, 1, 0, 1, 0],
                         [0, 0, 1, 0, 1, 0, 1, 1], [1, 0, 0, 1, 1, 0, 1, 0], [1, 0, 1, 0, 1, 1, 0, 1],
                         [1, 0, 1, 0, 1, 1, 0, 0], [1, 0, 1, 0, 1, 0, 0, 1], [0, 1, 0, 0, 1, 0, 1, 1],
                         [0, 1, 1, 0, 1, 0, 0, 1], [1, 1, 0, 1, 0, 0, 1, 0], [0, 1, 0, 1, 1, 0, 1, 0],
                         [0, 0, 1, 0, 1, 1, 0, 1], [1, 0, 1, 0, 0, 1, 0, 1], [1, 0, 0, 1, 0, 1, 1, 0],
                         [1, 0, 1, 1, 0, 1, 0, 0],
                         [1, 0, 0, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0, 0, 0],
                         [0, 0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0],
                         [0, 0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 0, 1]]

    image = skeleton.copy()
    # image = image/255;
    intersections = list()
    for x in range(1, len(image) - 1):
        for y in range(1, len(image[x]) - 1):
            # If we have a white pixel
            if image[x][y] == 1:
                neighbours = get_neighbours(x, y, image)
                valid = True
                if neighbours in validIntersection:
                    intersections.append((y, x))
    # Filter intersections to make sure we don't count them twice or ones that are very close together
    for point1 in intersections:
        for point2 in intersections:
            if (((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) < 10 ** 2) and (point1 != point2):
                intersections.remove(point2)
    # Remove duplicates
    intersections = list(set(intersections))
    return intersections
