#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from scipy import interpolate
import tf2_ros
from scipy.spatial.transform import Rotation as R
import csv
import cv2
import numpy as np
from pupil_apriltags import Detector

class PurePursuit(Node):
    """
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.pose_sub = self.create_subscription(Odometry, '/pf/pose/odom', self.pose_callback, 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.waypoint_pub1 = self.create_publisher(MarkerArray, '/waypoints1', 10)
        self.merge_pub = self.create_publisher(MarkerArray, '/merge', 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.opp_pose_sub = self.create_subscription(Float32MultiArray, '/cam', self.opp_pose_callback, 10)
        
        self.ego_pose = None
        self.opp_pose = None
        self.opp_pose_last = None
        self.distance_threshold = -110  # Define a distance threshold for switching waypoints


        self.target = [0.0, 0.0, 1]
        self.target1 = [0.0, 0.0, 1]
        self.skel = []
        self.L = 3.0
        self.Kp = 1.0
        self.Kd = 0.05
        self.my_line = 1

        self.i = 0

        self.merge = []
        self.merge_fin = []
        self.create_waypoints()
        self.publish_waypoints()
        self.create_waypoints1()
        self.publish_waypoints1()
        self.publish_merge()

        self.waypoints_og = self.waypoints.copy()
        self.waypoints1_og = self.waypoints1.copy()
        
        self.merging = False

        self.ay_max = 6.65
        self.delta_max = 1.5
        self.last_ang = 0.0
        

    def create_waypoints(self):
        #Define skeleton waypoints to interpolate between
        x_skel = []
        y_skel = []
        sect_skel = []
        skel = []
        with open('/home/nvidia/f1tenth_ws/src/pure_pursuit/data/finalprojraceline1.csv', newline='') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in spamreader:
                x_skel.append(row[0])
                y_skel.append(row[1])
                sect_skel.append(row[2])
                skel.append([row[0], row[1]])

        x_skel = np.array(x_skel, dtype=np.float64)
        y_skel = np.array(y_skel, dtype=np.float64)
        self.skel = np.array(skel, dtype=np.float64)
        self.waypoints = []

        tck, u = interpolate.splprep([x_skel, y_skel], s=0, per=True) #Interpolate the spline
        x, y = interpolate.splev(np.linspace(0, 1, 250), tck)

        for i in range(len(x)):
            self.waypoints.append([x[i], y[i], 1])
        


    def opp_pose_callback(self, msg):
        self.opp_pose_last = self.opp_pose
        self.opp_pose = msg.data
        self.i += 1


    def get_distance(self, x, y):  # input pixel x,y
        Hmount = 8.8
        intrinsic = np.array(
            [[694.71543085, 0, 449.37540776], [0, 695.54961208, 258.64705743], [0, 0, 1]])  # From calibration

        x_car = intrinsic[1][1] * Hmount / (y - intrinsic[1][2])
        y_car = - x_car * (x - intrinsic[0][2]) / intrinsic[0][0]
        return x_car, y_car


    def create_waypoints1(self):
        #Define skeleton waypoints to interpolate between
        x_skel1 = []
        y_skel1 = []
        sect_skel1 = []
        skel1 = []
        with open('/home/nvidia/f1tenth_ws/src/pure_pursuit/data/finalprojraceline2.csv', newline='') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in spamreader:
                x_skel1.append(row[0])
                y_skel1.append(row[1])
                sect_skel1.append(row[2])
                skel1.append([row[0], row[1]])

        x_skel1 = np.array(x_skel1, dtype=np.float64)
        y_skel1 = np.array(y_skel1, dtype=np.float64)
        self.skel1 = np.array(skel1, dtype=np.float64)
        self.waypoints1 = []

        tck1, u1 = interpolate.splprep([x_skel1, y_skel1], s=0, per=True) #Interpolate the spline
        x1, y1 = interpolate.splev(np.linspace(0, 1, 200), tck1)

        for i in range(len(x1)):
            self.waypoints1.append([x1[i], y1[i], 1])


    def publish_waypoints1(self):
        marker_s11 = Marker()
        marker_s11.header.frame_id = 'map'
        marker_s11.type = Marker.POINTS
        marker_s11.scale.x = 0.1
        marker_s11.scale.y = 0.1
        marker_s11.color.r = 1.0
        marker_s11.color.g = 1.0
        marker_s11.color.b = 1.0
        marker_s11.color.a = 1.0
        marker_s11.id = 6

        marker_s11.points = [Point(x=x1, y=y1, z=0.0) for x1,y1,s in self.waypoints1 if s == 1]

        marker_s21 = Marker()
        marker_s21.header.frame_id = 'map'
        marker_s21.type = Marker.POINTS
        marker_s21.scale.x = 0.1
        marker_s21.scale.y = 0.1
        marker_s21.color.r = 1.0
        marker_s21.color.g = 0.5
        marker_s21.color.b = 1.0
        marker_s21.color.a = 1.0
        marker_s21.id = 7

        marker_s21.points = [Point(x=x1, y=y1, z=0.0) for x1,y1,s in self.waypoints1 if s == 2]

        marker_s31 = Marker()
        marker_s31.header.frame_id = 'map'
        marker_s31.type = Marker.POINTS
        marker_s31.scale.x = 0.1
        marker_s31.scale.y = 0.1
        marker_s31.color.r = 1.0
        marker_s31.color.g = 1.0
        marker_s31.color.b = 0.5
        marker_s31.color.a = 1.0
        marker_s31.id = 8

        marker_s31.points = [Point(x=x1, y=y1, z=0.0) for x1,y1,s in self.waypoints1 if s == 3]

        marker_s41 = Marker()
        marker_s41.header.frame_id = 'map'
        marker_s41.type = Marker.POINTS
        marker_s41.scale.x = 0.1
        marker_s41.scale.y = 0.1
        marker_s41.color.r = 0.5
        marker_s41.color.g = 1.0
        marker_s41.color.b = 1.0
        marker_s41.color.a = 1.0
        marker_s41.id = 9

        marker_s41.points = [Point(x=x1, y=y1, z=0.0) for x1,y1,s in self.waypoints1 if s == 4]


        target1 = Marker()
        target1.header.frame_id = 'map'
        target1.type = Marker.POINTS
        target1.scale.x = 0.2
        target1.scale.y = 0.2
        target1.color.r = 0.0
        target1.color.g = 0.0
        target1.color.b = 1.0
        target1.color.a = 1.0
        target1.id = 10

        target1.points = [Point(x=self.target1[0], y=self.target1[1], z=0.0)]


        skel_marker1 = Marker()
        skel_marker1.header.frame_id = 'map'
        skel_marker1.type = Marker.POINTS
        skel_marker1.scale.x = 0.1
        skel_marker1.scale.y = 0.1
        skel_marker1.color.r = 1.0
        skel_marker1.color.g = 0.0
        skel_marker1.color.b = 0.0
        skel_marker1.color.a = 0.0
        skel_marker1.id = 11

        skel_marker1.points = [Point(x=x1, y=y1, z=0.0) for x1,y1 in self.skel1]

        marker_array1 = MarkerArray()
        marker_array1.markers = [marker_s11, marker_s21, marker_s31, marker_s41, target1, skel_marker1]

        self.waypoint_pub1.publish(marker_array1)

    def publish_merge(self):
        marker_s1 = Marker()
        marker_s1.header.frame_id = 'map'
        marker_s1.type = Marker.POINTS
        marker_s1.scale.x = 0.1
        marker_s1.scale.y = 0.1
        marker_s1.color.r = 1.0
        marker_s1.color.g = 0.0
        marker_s1.color.b = 0.0
        marker_s1.color.a = 1.0
        marker_s1.id = 0

        marker_s1.points = [Point(x=x, y=y, z=0.0) for x,y,s in self.merge]

        marker_array = MarkerArray()
        marker_array.markers = [marker_s1]

        self.merge_pub.publish(marker_array)

    def publish_waypoints(self):
        marker_s1 = Marker()
        marker_s1.header.frame_id = 'map'
        marker_s1.type = Marker.POINTS
        marker_s1.scale.x = 0.1
        marker_s1.scale.y = 0.1
        marker_s1.color.r = 0.0
        marker_s1.color.g = 1.0
        marker_s1.color.b = 0.0
        marker_s1.color.a = 1.0
        marker_s1.id = 0

        marker_s1.points = [Point(x=x, y=y, z=0.0) for x,y,s in self.waypoints if s == 1]

        marker_s2 = Marker()
        marker_s2.header.frame_id = 'map'
        marker_s2.type = Marker.POINTS
        marker_s2.scale.x = 0.1
        marker_s2.scale.y = 0.1
        marker_s2.color.r = 1.0
        marker_s2.color.g = 0.0
        marker_s2.color.b = 0.0
        marker_s2.color.a = 1.0
        marker_s2.id = 1

        marker_s2.points = [Point(x=x, y=y, z=0.0) for x,y,s in self.waypoints if s == 2]

        marker_s3 = Marker()
        marker_s3.header.frame_id = 'map'
        marker_s3.type = Marker.POINTS
        marker_s3.scale.x = 0.1
        marker_s3.scale.y = 0.1
        marker_s3.color.r = 1.0
        marker_s3.color.g = 1.0
        marker_s3.color.b = 0.0
        marker_s3.color.a = 1.0
        marker_s3.id = 2

        marker_s3.points = [Point(x=x, y=y, z=0.0) for x,y,s in self.waypoints if s == 3]

        marker_s4 = Marker()
        marker_s4.header.frame_id = 'map'
        marker_s4.type = Marker.POINTS
        marker_s4.scale.x = 0.1
        marker_s4.scale.y = 0.1
        marker_s4.color.r = 1.0
        marker_s4.color.g = 1.0
        marker_s4.color.b = 1.0
        marker_s4.color.a = 1.0
        marker_s4.id = 3

        marker_s4.points = [Point(x=x, y=y, z=0.0) for x,y,s in self.waypoints if s == 4]


        target = Marker()
        target.header.frame_id = 'map'
        target.type = Marker.POINTS
        target.scale.x = 0.2
        target.scale.y = 0.2
        target.color.r = 0.0
        target.color.g = 0.0
        target.color.b = 1.0
        target.color.a = 1.0
        target.id = 4

        target.points = [Point(x=self.target[0], y=self.target[1], z=0.0)]


        skel_marker = Marker()
        skel_marker.header.frame_id = 'map'
        skel_marker.type = Marker.POINTS
        skel_marker.scale.x = 0.1
        skel_marker.scale.y = 0.1
        skel_marker.color.r = 0.0
        skel_marker.color.g = 0.0
        skel_marker.color.b = 0.0
        skel_marker.color.a = 1.0
        skel_marker.id = 5

        skel_marker.points = [Point(x=x, y=y, z=0.0) for x,y in self.skel]

        marker_array = MarkerArray()
        marker_array.markers = [marker_s1, marker_s2, marker_s3, marker_s4, target, skel_marker]

        self.waypoint_pub.publish(marker_array)


    def find_target_waypoint(self, pose_msg, L):
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y

        if self.merging:
            arr = self.merge.copy()
        elif self.my_line == 1:
            arr = self.waypoints.copy()
        else:
            arr = self.waypoints1.copy()
        arr += arr
        found = False
        for i in range(len(arr)): #Iterate forwards so we choose a point ahead of us
            p = arr[i]
            if (p[0] - x)**2 + (p[1] - y)**2 <= 3:
                found = True
            if (i+1 == len(arr)):
                break
            next_p = arr[i+1]

            if found and (p[0] - x)**2 + (p[1] - y)**2 <= L**2 and (next_p[0] - x)**2 + (next_p[1] - y)**2 > L**2:
                return [(p[0] + next_p[0]) / 2, (p[1] + next_p[1]) / 2, next_p[2]]

        return self.target

    def transform_target(self, pose_msg, x, y):
        car_x = pose_msg.pose.pose.position.x
        car_y = pose_msg.pose.pose.position.y
        car_z = 0
        r = R.from_quat([pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y,
                         pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w])
        rotation_matrix = r.as_matrix()
        h_matrix = np.array([[rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], car_x],
                             [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], car_y],
                             [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], car_z],
                             [0, 0, 0, 1]])
        point = np.array([x, y, 0, 1])
        new_point = np.linalg.inv(h_matrix) @ point #Point in car's reference frame

        return new_point[0], new_point[1]



    def get_v(self):
        vel = 2.0
        if vel < 1.0:
            self.delta_max = 1.5
        else:
            self.delta_max = np.arctan2(13 / (vel**2 / self.ay_max))
        return vel


    def pose_callback(self, pose_msg):
        self.ego_pose = pose_msg.pose.pose.position
        
        if self.ego_pose is not None and self.opp_pose is not None and self.opp_pose_last is not None:    
            dx = self.opp_pose[0]
            dx2 = self.opp_pose[1]
            dy = self.opp_pose[2]
            print(f"{dx} {dx2} {dy}")

            opp_vel_x = dx2 - self.opp_pose_last[1]
            infered_pose_x = dx2 + opp_vel_x
            
            if infered_pose_x >= self.distance_threshold:
                min_l = 2.0
                max_l = 5.0
                v_opp_max = 100 #cm/tick
                
                l = (max_l - min_l) * (opp_vel_x / v_opp_max) + min_l

                print(opp_vel_x)

                if (dy < -15) and self.my_line == 1 and self.i > 15: #Opp car to my right
                    self.i = 0
                    print("track change to 2")
                    self.my_line = 2
                    self.merge_fin = self.find_target_waypoint(pose_msg, l)

                    tck, u = interpolate.splprep([[self.ego_pose.x, self.merge_fin[0]], [self.ego_pose.y, self.merge_fin[1]]], s=0, k=1) #Interpolate the spline
                    x, y = interpolate.splev(np.linspace(0, 1, 5), tck)

                    self.merging = True
                    for i in range(len(x)):
                        self.merge.append([x[i], y[i], 1])

                elif (dy > 15) and self.my_line == 2 and self.i > 15: #Opp car to my left
                    self.i = 0
                    print("track change to 1")
                    self.my_line = 1

                    self.merge_fin = self.find_target_waypoint(pose_msg, l)

                    tck, u = interpolate.splprep([[self.ego_pose.x, self.merge_fin[0]], [self.ego_pose.y, self.merge_fin[1]]], s=0, k=1) #Interpolate the spline
                    x, y = interpolate.splev(np.linspace(0, 1, 5), tck)

                    self.merging = True
                    for i in range(len(x)):
                        self.merge.append([x[i], y[i], 1])
        
        if self.merging and np.abs(self.ego_pose.x - self.merge_fin[0]) <= 2.5 and np.abs(self.ego_pose.y - self.merge_fin[1]) <= 2.5:
            self.merging = False

        #find the current waypoint to track using methods mentioned in lecture
        self.target = self.find_target_waypoint(pose_msg, self.L)

        #transform goal point to vehicle frame of reference
        car_targetx, car_targety = self.transform_target(pose_msg, self.target[0], self.target[1])
        vel = self.get_v()

        #calculate curvature/steering angle
        if car_targety >= 0: #TURN LEFT
            steering_angle = 2.0*np.abs(car_targety)/(self.L**2) * self.Kp
            steering_angle = steering_angle + (steering_angle - self.last_ang) * self.Kd
        else: #TURN RIGHT
            steering_angle = -2.0*np.abs(car_targety)/(self.L**2) * self.Kp
            steering_angle = steering_angle + (steering_angle - self.last_ang) * self.Kd
        self.last_ang = steering_angle
        #print(steering_angle_vel)

        #publish drive message, don't forget to limit the steering angle.
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = np.clip(steering_angle, -self.delta_max, self.delta_max)
        msg.drive.speed = vel
        self.drive_pub.publish(msg)
        
        self.publish_waypoints()
        self.publish_waypoints1()
        self.publish_merge()
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
