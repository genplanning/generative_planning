import colors
import cv2 as cv
import heapq as hq
import math
import matplotlib.pyplot as plt
import numpy as np
import os
import planner_utils as plutils
import utils

'''
Node Class, holds coordinate, cost parameters, and parent
'''


class Node:
    def __init__(self):
        self.coord = None  # coordinate (x, y, theta)
        self.cost = 0  # cost for astar planner, equivalent of f in astar: f = g + h
        self.dist = math.inf  # distance from the start to the current node, equivalent to g
        self.parent = None  # parent node

    def __eq__(self, other):
        if other is None:
            return False
        if not isinstance(other, Node):
            return False
        return self.dist == other.dist


'''
Hybrid Astar Planner: Discrete
'''


class HybridAstar:
    """
    Initialize class
    map_img: nxmx3 image of the binarized map
    obstacles: kx2 sized array of obstacle coordinates, [[row, col], ...]
    start: (col, row, theta)
    goal: (col, row, theta)
    vehicle length:
    velocity:
    """

    def __init__(self, map_img, obstacles, vehicle_length=1, goal_thresh=25, velocity=6.0):
        rows, cols, _ = map_img.shape  # map shape

        # initialize self variables
        self.goal_thresh = goal_thresh
        self.map_img = map_img
        self.min_x = 0  # bounds on map size
        self.max_x = cols - 1
        self.min_y = 0
        self.max_y = rows - 1
        self.obstacles = [tuple(coord) for coord in obstacles]  # convert to tuples for hashing
        self.vehicle_length = vehicle_length  # length of the vehicle
        self.velocity = velocity  # velocity

        self.obstacles = set(self.obstacles)  # hash obstacles to set

        # set of possible inputs
        self.steering_inputs = [-1.2, -1, 0, 1, 1.2]  # possible steering inputs
        # self.steering_inputs = [-1, 0, 1]

    """
    backtrack astar
    """

    def backtrack(self, current_node):
        path_coords = []
        while current_node.parent is not None:  # while current node has a parent
            coords = current_node.coord

            path_coords.append([coords[0], self.max_y - coords[1], coords[2]])  # append coordinates to path (c, r,
            # theta)

            parent = current_node.parent
            current_node = parent

        return path_coords

    """
    check astar neighbor conditions
    """

    def check_conditions(self, neighbor_coords, visited_list):

        neighbor_x_d = neighbor_coords[0]
        neighbor_y_d = neighbor_coords[1]
        neighbor_theta_d = neighbor_coords[2]

        pass_conditions = 1  # pass conditions

        # check if neighbor coordinates are in bound
        if neighbor_x_d <= self.min_x or neighbor_x_d >= self.max_x \
                or neighbor_y_d <= self.min_y or neighbor_y_d >= self.max_y:
            pass_conditions = 0

        # check if neighbor is an obstacle node
        # obstacles in [row, col] -> [max_y - row, x]
        if (self.max_y - int(neighbor_y_d), int(neighbor_x_d)) in self.obstacles:
            pass_conditions = 0

        # check if neighbor has already been visited
        if (neighbor_x_d, neighbor_y_d, neighbor_theta_d) in visited_list:
            pass_conditions = 0

        return pass_conditions

    """
    for tuples: l2 norm between position and target for entire state vector
    """

    @staticmethod
    def euc_dist(position, target):
        output = np.sqrt(((position[0] - target[0]) ** 2) + ((position[1] - target[1]) ** 2) +
                         (math.radians(position[2]) - math.radians(target[2])) ** 2)
        return float(output)

    @staticmethod
    def euc_dist_2(position, target):
        output = np.sqrt(((position[0] - target[0]) ** 2) + ((position[1] - target[1]) ** 2))
        return float(output)

    """
    find hybrid astar path
    """

    def find_path(self, start, goal):
        """
        convert start and goal to float
        start and goal are in discrete coordinates
        img to dynamics coordinate conversion: col, row, theta <--> x, max_y - row, theta
        """
        start = (start[0], self.max_y - start[1], start[2])  # (x, y, theta)
        goal = (goal[0], self.max_y - goal[1], goal[2])

        # initial node
        initial_node = Node()
        initial_node.coord = start  # coordinate of initial node (x, y, theta)
        initial_node.dist = 0  # distance to start equals zero

        """
        heap to hold open nodes
        elements in the heap are (node.cost, node.coord)
        """
        open_heap = []
        hq.heappush(open_heap, (initial_node.cost, initial_node))

        # list of visited nodes
        visited_list = []  # (x, y, theta)
        open_list = []  # (x, y, theta)

        debug_idx = 0
        # hybrid astar loop
        while len(open_heap) != 0:  # while priority queue is not empty
            #
            # debug_idx += 1
            # if debug_idx > 6:
            #     break

            current_element = hq.heappop(open_heap)  # pop top element from heap with the lowest cost
            # current_cost = current_element[0]  # the current cost
            current_node = current_element[1]  # get the current node
            if current_node.coord in visited_list:
                continue

            visited_list.append(current_node.coord)  # mark the node as visited, same as removing from open list

            if len(visited_list) > 10000: # here no path can be found
                current_node = None
                return current_node

            # if the current node is close enough to the goal then break
            # if self.euc_dist(current_node.coord, goal) < self.goal_thresh:
            #     print("we have reached the goal")
            #     path_coords = self.backtrack(current_node)  # [c, r, theta]
            #     return path_coords
            # only account for x, y, coordinates
            if self.euc_dist_2((current_node.coord[0], current_node.coord[1]), (goal[0], goal[1])) \
                    < self.goal_thresh:
                # print(current_node.coord)
                print("we have reached the goal")
                path_coords = self.backtrack(current_node)  # [c, r, theta]
                return path_coords

            # current coordinates
            current_coord = current_node.coord
            current_x = current_coord[0]
            current_y = current_coord[1]
            current_theta = current_coord[2]

            # debug map
            map_debug = self.map_img.copy()
            map_debug = cv.circle(map_debug, (int(current_x), self.max_y - int(current_y)),
                                  colors.radius, colors.green, colors.thickness)  # column, row

            for delta in self.steering_inputs:

                # new continuous state after applying neighbor deltas
                neighbor_theta_cts = math.radians(current_theta) + (
                        self.velocity * math.tan(math.radians(delta)) / (float(self.vehicle_length)))
                neighbor_x_cts = current_x + (self.velocity * math.cos(neighbor_theta_cts))
                neighbor_y_cts = current_y + (self.velocity * math.sin(neighbor_theta_cts))

                # print(current_theta, current_x, current_y,
                #       delta, self.vehicle_length, self.velocity)
                # print(neighbor_theta_cts, neighbor_x_cts, neighbor_y_cts)

                # convert theta to degrees
                neighbor_theta_cts = math.degrees(neighbor_theta_cts)

                # round to discrete
                neighbor_x_d = round(neighbor_x_cts)
                neighbor_y_d = round(neighbor_y_cts)
                neighbor_theta_d = round(neighbor_theta_cts)

                # check if neighbor coords passes neighbor conditions
                neighbor_coords = (neighbor_x_d, neighbor_y_d, neighbor_theta_d)
                pass_conditions = self.check_conditions(neighbor_coords, visited_list)
                if pass_conditions == 0:  # if neighbor have fails to pass conditions
                    continue

                # plot neighbors on map debug
                map_debug = cv.circle(map_debug, (int(neighbor_x_d), self.max_y - int(neighbor_y_d)),
                                      colors.radius, colors.blue, colors.thickness)  # column, row
                '''
                calculate the new distance:
                the new distance is the current nodes distance plus the distance between current
                node and the neighbor
                '''
                # trav_dist = np.sqrt(((neighbor_x_d - current_x) ** 2) + ((neighbor_y_d - current_y) ** 2))
                neighbor_dist = current_node.dist + self.velocity

                # calculate heuristic for astar
                heuristic = self.euc_dist((neighbor_x_d, neighbor_y_d, neighbor_theta_d), goal)
                # print((neighbor_x_d, neighbor_y_d, neighbor_theta_d))
                # print(neighbor_dist, heuristic)

                # calculate total cost f = g + h
                neighbor_cost = neighbor_dist + heuristic
                # print(neighbor_cost)

                # if the neighbor is already in the open list,
                # if the neighbor g is greater than open list neighbor g, then continue
                for open_node in open_list:
                    if (neighbor_x_d, neighbor_y_d, neighbor_theta_d) == open_node.coord:
                        if neighbor_dist > open_node.dist:
                            continue

                # add neighbor to the open list
                neighbor_node = Node()
                neighbor_node.coord = (neighbor_x_d, neighbor_y_d, neighbor_theta_d)  # coordinate (x, y, theta)
                neighbor_node.cost = neighbor_cost  # cost for astar planner, equivalent of f in astar: f = g + h
                neighbor_node.dist = neighbor_dist  # distance from the start to the current node, equivalent to g
                neighbor_node.parent = current_node  # parent node

                open_list.append(neighbor_node)

                # add neighbor to open heap
                hq.heappush(open_heap, (neighbor_cost, neighbor_node))

            # utils.imshow_wait(map_debug)
            # break

        if len(open_heap) == 0:  # if no path is found (heap has no more elements in it)
            # print("no path can be found")
            current_node = None
            return current_node


def main():
    """load images"""
    # define paths
    data_root = '/home/Dataset/KITTI-360'
    # directories to load
    data_dir = os.path.join(data_root, 'test_planner')
    data_full = os.path.join(data_dir, 'full')

    # get list of data files
    full_names = os.listdir(data_full)
    full_name = full_names[0]
    full_name = '0000010400.png'

    # read image
    full_img_path = os.path.join(data_full, full_name)
    full_img = cv.imread(full_img_path)

    img_coords = plutils.img_coords(full_img)  # image coords of full image
    road_coords = plutils.get_color_coords(full_img, img_coords, colors.road_color)  # coordinates of road pixels
    # map image, binarize road coordinates to black and white image for planning
    map_img = np.zeros((full_img.shape[0], full_img.shape[1]))
    plutils.bin_color_coords(map_img, road_coords)  # binarize road coordinates

    # resize map image
    map_img = utils.resize_image(map_img, scale_percent=35)

    # recalculate for resized image
    map_img = np.repeat(map_img[:, :, np.newaxis], 3, axis=2)  # shape to n x m x 3
    img_coords = plutils.img_coords(map_img)  # image coords of full image
    # coordinates of road pixels, [[row, col], ...]
    obs_coords = plutils.get_color_coords(map_img, img_coords, colors.black)

    # define start and goal locations
    start = (210, 120, -15)  # col, row, theta <--> x, max_y - row, theta (degrees)
    goal = (300, 85, 65)
    map_img = cv.circle(map_img, (start[0], start[1]), colors.radius, colors.blue, colors.thickness)
    map_img = cv.circle(map_img, (goal[0], goal[1]), colors.radius, colors.red, colors.thickness)
    # utils.imshow_wait(map_img)
    print(map_img.shape)
    # instantiate class
    hybrid_astar = HybridAstar(map_img, obs_coords, goal_thresh=15, velocity=16.0)
    path_coords = hybrid_astar.find_path(start, goal)

    # plot path
    r = 5  # arrow radius
    rows, cols, _ = map_img.shape
    for coord in path_coords:
        col = coord[0]
        row = coord[1]
        theta = coord[2]
        print(coord)
        map_img = cv.circle(map_img, (col, row),
                            colors.radius, colors.green, colors.thickness)
        plt.arrow(col, row, r * math.cos(math.radians(-theta)),
                  r * math.sin(math.radians(-theta)), head_width=5)

    plt.arrow(start[0], start[1], r * math.cos(math.radians(-start[2])),
              r * math.sin(math.radians(-start[2])), head_width=5)
    plt.arrow(goal[0], goal[1], r * math.cos(math.radians(-goal[2])),
              r * math.sin(math.radians(-goal[2])), head_width=5)

    # utils.imshow_wait(map_img)
    plt.style.use('dark_background')
    plt.imshow(map_img)
    plt.show()

    print(map_img.shape)


if __name__ == "__main__":
    main()
