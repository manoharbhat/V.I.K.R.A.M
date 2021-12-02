#! /usr/bin/env python

#importing everything necessary
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

#global variables for callback functions
global robot_pose
global laser_data

#callback functions
def callback_odom(data):
    """returns robot position as a list containing [x_position, y_position, z_orientation]"""
    global robot_pose
    orientation = euler_from_quaternion([data.pose.pose.orientation.x, \
      data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
    robot_pose = [data.pose.pose.position.x, data.pose.pose.position.y, orientation]
    return robot_pose

def callback_laser(data):
    """converts range data from lidar into 5 sections and
    gives out a list containing minimum distance in those sections"""
    global laser_data
    laser_data[0] = float(min(min(data.ranges[0:143]), 10))
    laser_data[1] = float(min(min(data.ranges[144:287]), 10))
    laser_data[2] = float(min(min(data.ranges[288:431]), 10))
    laser_data[3] = float(min(min(data.ranges[432:575]), 10))
    laser_data[4] = float(min(min(data.ranges[576:720]), 10))
    return laser_data

#other functions
def on_line(p1, p2, current): #returns 1 if current lies on line passing p1 and p2
    """returns true if 'current' point lies on the line passing through 'p1' and 'p2'
    this is used to come out of wall following mode, according to bug2 algorithm"""
    #giving range, as there might be some error in computations
    return current[1]-p1[1]-0.075 < ((p2[1]-p1[1]) / (p2[0]-p1[0])) * (current[0]-p1[0]) < current[1]-p1[1]+0.075


def main_loop():
    """main function"""
    #creating waypoints to follow from the equation given
    num_waypoints = 14
    waypoints = []
    xx = 0
    waypoints.append([6, 7])     #appending final goal

    #global variables for callbacks
    global laser_data
    laser_data = [10.0, 10.0, 10.0, 10.0, 10.0]
    global robot_pose
    robot_pose = [0, 0, 0]

    #other variables
    Kp = 1    #Kp for line tracing
    cmd_vel_data = Twist()
    ii = 0  #variable to keep track of next waypoint goal
    obstacle_present = 0

    #initializing node, publisher and subscribers
    rospy.init_node('vikram_dis_description', anonymous=False)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.Subscriber('/odom', Odometry, callback_odom)
    rospy.Subscriber('/laser/scan', LaserScan, callback_laser)
    rate = rospy.Rate(40)

    #control loop
    while not rospy.is_shutdown():
        #check if obstacle is present
        if (min(laser_data[0:5]) < 4) and (obstacle_present == 0):
            obstacle_present = 1

            #note down present position and next goal, to get out of bug2
            present = [robot_pose[0], robot_pose[1]]
            goal = [waypoints[ii][0], waypoints[ii][1]]

            #rotate so the wall is on the right
            while (laser_data[4] > laser_data[0] and laser_data[4] > laser_data[1] and \
              laser_data[4] > laser_data[2] and laser_data[4] > laser_data[3]):
                cmd_vel_data.linear.x = 0.2
                cmd_vel_data.angular.z = 3.0
                pub_cmd_vel.publish(cmd_vel_data)

        #if obstacle is present, follow the wall
        if obstacle_present:
            if not on_line(present, goal, robot_pose): #follow wall until not on line (bug2)
                if laser_data[2] <= 1.5:
                    if laser_data[3] > 0.8:
                        cmd_vel_data.linear.x = 0.3
                        cmd_vel_data.angular.z = 3.0
                        pub_cmd_vel.publish(cmd_vel_data)
                    elif laser_data[3] < 0.8:
                        cmd_vel_data.linear.x = 0.3
                        cmd_vel_data.angular.z = -3.0
                        pub_cmd_vel.publish(cmd_vel_data)
                    else:
                        cmd_vel_data.linear.x = 0.3
                        cmd_vel_data.angular.z = 0
                        pub_cmd_vel.publish(cmd_vel_data)

                elif laser_data[2] > 1.5:
                    if laser_data[0] <= 1.2 and laser_data[1] <= 1 and laser_data[0] >= 0.8 and laser_data[1] >= 0.8:
                        cmd_vel_data.linear.x = 0.3
                        cmd_vel_data.angular.z = -0.3
                        pub_cmd_vel.publish(cmd_vel_data)
                    elif laser_data[1] < 0.8 or  laser_data[0] < 0.8:
                        cmd_vel_data.linear.x = 0.3
                        cmd_vel_data.angular.z = 0.3
                    else:
                        cmd_vel_data.linear.x = 0.3
                        cmd_vel_data.angular.z = -1
                        pub_cmd_vel.publish(cmd_vel_data)

            else:   #the robot is now on line, exit wall following
                obstacle_present = 0

        #no obstacle present, move to next waypoint
        if not obstacle_present:
            #calculate angle to goal
            theta_goal = math.atan2(waypoints[ii][1]-robot_pose[1], waypoints[ii][0]-robot_pose[0])

            #calculate error
            e_theta = theta_goal - robot_pose[2]
            if e_theta > math.pi:
                e_theta = math.pi
            elif e_theta < -math.pi:
                e_theta = -math.pi

            #calculate PID value and saturate it
            pid_val = Kp*e_theta
            if pid_val > math.pi:
                pid_val = math.pi
            elif pid_val < -math.pi:
                pid_val = -math.pi

            #publish linear and angular velocities
            cmd_vel_data.angular.z = pid_val
            cmd_vel_data.linear.x = 0.3
            pub_cmd_vel.publish(cmd_vel_data)


        if robot_pose[0] > waypoints[ii][0]:    #give next waypoint goal
            ii += 1

        if ii > num_waypoints+1:  #reached final goal, stop here
            cmd_vel_data.linear.x = 0
            cmd_vel_data.angular.z = 0
            pub_cmd_vel.publish(cmd_vel_data)
            rospy.signal_shutdown("Goal Reached")

        rate.sleep()

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass