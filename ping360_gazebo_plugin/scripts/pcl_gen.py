#!/usr/bin/env python3

from math import cos, pi, sin

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from geometry_msgs.msg import Point32   
from ping360_sonar.msg import SonarEcho
import numpy_pc2

sonarRange = 20
angle = 0
intensities = [0,0,0,0] 
points = [] 
ranges = 0
last_data = []
last_points = []

first_loop = True

def echo_callBack(echo_msg):

    global imgSize, angle, ranges, intensities, center

    angle = echo_msg.angle
    ranges = echo_msg.range
    intensities = echo_msg.intensities
    #get_cloud()
    #print(format(ranges))
    #get_cloud()





points = []

modified_indices = []

points = []


def get_cloud():
    global intensities, angle, ranges, points, first_loop
    
    data = intensities
    
    imgSize = int(rospy.get_param('~imgSize', 200))    
    image = np.zeros((imgSize, imgSize, 1), np.uint8)             
    step = int(rospy.get_param('~step', 1))              
    center = (float(imgSize / 2), float(imgSize / 2))
    linear_factor = float(len(data)) / float(center[0])

    try:

        new_points = []


        old_coordinates = []

        for i in range(int(center[0])):
            if i < center[0]:
                pointColor = data[int(i * linear_factor - 1)]
            else:
                pointColor = 0
            for k in np.linspace(0, step, 1 * step): 
                theta = 2 * np.pi * (angle*200/180 + k)/400
                x = float(i) * np.cos(theta)
                y = float(i) * np.sin(theta)
                image[int(center[0] + x)][int(center[1] + y)][0] = pointColor
                x_world = x / float(center[0]) * float(sonarRange)
                y_world = -1 * y / float(center[0]) * float(sonarRange)
                z_world = float(pointColor) / 255.0 * float(sonarRange)
                old_coordinates.append((x_world, y_world))
                new_points.append([x_world, y_world, z_world, pointColor])


        points = [p for p in points if (p[0], p[1]) not in old_coordinates]


        if angle >= 355 and first_loop:
            points = [point for point in points if np.arctan2(point[1], point[0]) * 180 / np.pi < 355]
            first_loop = False


        points += new_points


        pc2 = numpy_pc2.array_to_xyzi_pointcloud2f(points, rospy.Time.now(), "sonar_frame")
        pclPub.publish(pc2)

        pcl1 = PointCloud()
        pcl1.header.stamp = rospy.Time.now()
        pcl1.header.frame_id = "sonar_frame"
        pcl1.points = [Point32(x=p[0], y=p[1], z=p[2]) for p in points]
        pcl1_pub.publish(pcl1)

    except IndexError:
        rospy.logwarn("IndexError: data response was empty, skipping this iteration..")



def main():

    global pclPub
    global pcl1_pub
    global sonarRange

    rospy.init_node('pcl_gen', anonymous=True)
    sub_echo = rospy.Subscriber("/ping360_node/sonar/data", SonarEcho, echo_callBack)
    pclPub = rospy.Publisher("/ping360_node/sonar/point_cloud2", PointCloud2, queue_size=10)
    pcl1_pub = rospy.Publisher("/ping360_node/sonar/point_cloud", PointCloud, queue_size=10)

    while not rospy.is_shutdown():
        rate = rospy.Rate(100)
        get_cloud()
        rate.sleep()



if __name__ == '__main__':
    
    main()

