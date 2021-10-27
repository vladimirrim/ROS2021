#! /usr/bin/python3

import numpy as np

import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan


class GridCallback:
    def __init__(self):
        self.pub_markers = rospy.Publisher('/markers', Marker, queue_size=10)
        self.pub_grid = rospy.Publisher('/grid', OccupancyGrid, queue_size=10)

    def publish_markers(self, xs, ys):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.color.a = 1.
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.type = marker.POINTS
        marker.points = [Point(x, y, 0.) for x, y in zip(xs, ys)]
        self.pub_markers.publish(marker)

    def publish_grid(self, xs, ys, res, size=10):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = res
        grid.info.width = 2 * int(size / res) + 3
        grid.info.height = 2 * int(size / res) + 3
        grid_data = np.zeros((grid.info.width, grid.info.height)) - 1
        for x, y in zip(xs, ys):
            if abs(x) < size and abs(y) < size:
                i = int((x + size) // res)
                j = int((y + size) // res)
                grid_data[i][j] = min(grid_data[i][j] + 10, 100)
        grid.info.origin.position.x = -size
        grid.info.origin.position.y = -size
        grid.data = list(grid_data.transpose(1, 0).reshape(-1).astype(int))
        self.pub_grid.publish(grid)

    def __call__(self, msg, threshold=0.1):
        ranges = np.array(msg.ranges)
        rho = msg.angle_min + msg.angle_increment * np.arange(ranges.size)
        mask = np.abs(ranges[1:] - ranges[:-1]) < threshold
        mask = np.concatenate([mask, [True]])
        mask[0] = True
        xs = ranges[mask] * np.cos(rho[mask])
        ys = ranges[mask] * np.sin(rho[mask])
        self.publish_markers(xs, ys)
        self.publish_grid(xs, ys, threshold)


rospy.init_node('map')
rospy.Subscriber('/base_scan', LaserScan, GridCallback())
rospy.spin()
