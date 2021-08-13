#!/usr/bin/env python3

import sys
print(sys.path)

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import math

import tf2_ros
import sensor_msgs.msg
import nav_msgs.msg
import std_msgs.msg
import rosgraph_msgs.msg
#from sensor_msgs import point_cloud2 as pc2
import dots_interfaces.msg
import geometry_msgs.msg
import numpy as np
import random

import struct
import operator
import py_trees
import py_trees_ros



# The behaviour of sequence without memory in the current py-trees is not 
# semantically correct, it should be the inverse of sel without mem. 
# Subclassing and adding inverters makes a fake version with the correct semantics
class FakeSequence(py_trees.decorators.Inverter):
    def __init__(self, **kwargs):
        super(FakeSequence, self).__init__(child=py_trees.composites.Selector(**kwargs))
    def add_child(self, child):
        self.decorated.add_child(py_trees.decorators.Inverter(child))

class G:
    # Global values
    max_linear_velocity     = 0.5
    max_angular_velocity    = 3.0
    finish_time             = 0

    # obstacle avoidance parameter
    R_rob                   = 1000
    R_r                     = 0.1
    N_r                     = 1

    # Last direction and motion
    Last_direction          = random.uniform(-math.pi, math.pi)
    Last_motion             = geometry_msgs.msg.Twist()
    Last_motion.linear.x    = max_linear_velocity * math.cos(Last_direction)
    Last_motion.linear.y    = max_linear_velocity * math.sin(Last_direction)
    Last_motion.angular.z   = 0.0

    # task list
    carriers                = [100, 101, 102, 103, 104, 105, 106, 107, 108, 109]
    found_product           = []
    Global_information      = []

    # Perimeter amera angles
    camera_angles           = [0.2967, 2.251, -2.251, -0.2967]


class Pick_random_direction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Pick_random_direction, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(key='random_cmd_vel', access=py_trees.common.Access.WRITE)
    def update(self):
        cv              = geometry_msgs.msg.Twist()
        angle           = random.uniform(-math.pi, math.pi)
        cv.linear.x     = G.max_linear_velocity * math.cos(angle)
        cv.linear.y     = G.max_linear_velocity * math.sin(angle)
        cv.angular.z    = 0.0
        self.blackboard.random_cmd_vel = cv
        return py_trees.common.Status.SUCCESS

class Pick_last_direction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Pick_last_direction, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(key='last_cmd_vel', access=py_trees.common.Access.WRITE)
    def update(self):
        cv              = G.Last_motion
        cv.angular.z    = 0.0
        self.blackboard.last_cmd_vel = cv
        return py_trees.common.Status.SUCCESS
        
        
class Process_irtof(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Process_irtof, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(key='irtof', access=py_trees.common.Access.READ)
        self.blackboard.register_key(key='obstacle', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='collision', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='avoid_cmd_vel', access=py_trees.common.Access.WRITE)

        self.max_range          = 2.0
        self.min_range          = 0.13
        self.collision_range    = 0.3
        self.speed              = 0.5


    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            self.node = None
            print(e)

    def update(self):
        # Return running if the blackboard entry does not yet exist
        try:
            irtof = self.blackboard.irtof
        except KeyError:
            return py_trees.common.Status.RUNNING

        # There are 16 sensors, make somewhere to store data from each one. The Gazebo
        # version of the laser scanner doesn't send data for points with no return detected
        # but the hardware does, so fill in the maximum range for each entry to start with.
        msg = irtof
        data        = np.zeros((16))
        data[:]     = self.max_range
        step        = math.pi / 8.0
        prox        = np.zeros(2)
        collision   = False
        for i in range(msg.width):
            # Points within the pointcloud are actually locations in 3D space in the scanner
            # frame. Each is a float32 taking 12 bytes. 
            # Extract point
            [x, y, z]   = struct.unpack('fff', bytes(msg.data[i * msg.point_step : i * msg.point_step + 12]))
            # Get angle and see which sensor it was
            th          = math.atan2(y, x)
            if th < 0.0:
                th += 2 * math.pi
            idx         = int(round(th / step))
            # Get the distance and save it in the array
            dist        = math.sqrt(x**2 + y**2)
            data[idx]   = dist
            # Check if there is a collision
            if dist < self.collision_range:
                collision = True
            # Calculate a vector pointing towards the nearest obstacle
            nm          = np.array([x, y]) / dist
            nm_inv_len  = G.R_rob * G.R_r * dist * math.exp(-dist / G.R_r) / G.N_r
            nm          = nm * nm_inv_len
            prox        += nm

        random_angle = random.uniform(-math.pi, math.pi)
        bias         = np.array([math.cos(random_angle), math.sin(random_angle)])
        prox         = prox + bias

        self.blackboard.obstacle    = prox
        self.blackboard.collision   = collision

        cv = geometry_msgs.msg.Twist()

        coll_vector     = -prox / np.linalg.norm(prox)
        cv.linear.x     = coll_vector[0] * G.max_linear_velocity
        cv.linear.y     = coll_vector[1] * G.max_linear_velocity
        cv.angular.z    = random.uniform(-G.max_angular_velocity / 2.0, G.max_angular_velocity / 2.0)
        self.blackboard.avoid_cmd_vel = cv

        if collision == True:
            G.Last_motion = self.blackboard.avoid_cmd_vel
        #self.node.get_logger().info('%s' % irtof)
        return py_trees.common.Status.SUCCESS

        
class Process_vision(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Process_vision, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(key='cam0_tags', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='zero_cmd_vel', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='found_or_not', access=py_trees.common.Access.WRITE)


    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            self.node = None
            print('Should have node! %s' % e)
            sys.exit(1)

        # The robot name is always used for the namespace, get the name by removing the 
        # leading '/'
        self.robot_name     = self.node.get_namespace()[1:]
        self.information_task = std_msgs.msg.Int16MultiArray()
        # Listen to the lists of tags seen by each camera
        self.cam0_tags_sub  = self.node.create_subscription(dots_interfaces.msg.TagArray, 
                            'cam0_tags', self.cam0_tags_callback, qos_profile_system_default)
        self.cam1_tags_sub  = self.node.create_subscription(dots_interfaces.msg.TagArray, 
                            'cam1_tags', self.cam1_tags_callback, qos_profile_system_default)
        self.cam2_tags_sub  = self.node.create_subscription(dots_interfaces.msg.TagArray, 
                            'cam2_tags', self.cam2_tags_callback, qos_profile_system_default)
        self.cam3_tags_sub  = self.node.create_subscription(dots_interfaces.msg.TagArray, 
                            'cam3_tags', self.cam3_tags_callback, qos_profile_system_default)
        self.cam4_tags_sub  = self.node.create_subscription(dots_interfaces.msg.TagArray, 
                            'cam4_tags', self.cam4_tags_callback, qos_profile_system_default)

        self.tasks_pub = self.node.create_publisher(std_msgs.msg.Int16MultiArray, '/robot_tasks', 10)
        self.tasks_sub = self.node.create_subscription(std_msgs.msg.Int16MultiArray, '/robot_tasks', self.information_callback, qos_profile=qos_profile_system_default)

        # Transform listener and broadcaster
        self.tfbuffer   = tf2_ros.Buffer()
        self.listener   = tf2_ros.TransformListener(self.tfbuffer, self.node)
        self.br         = tf2_ros.TransformBroadcaster(self.node)


        # Local transform
        self.bc = tf2_ros.BufferCore(rclpy.duration.Duration(seconds=10.0))
        self.blackboard.zero_cmd_vel    = geometry_msgs.msg.Twist()
        self.timer                      = 0
        self.blackboard.found_or_not    = False

    def get_past(self, delta):
        # Look slightly into the past for the transforms. This seems to fail sometimes
        # with a negative time. Presumably a callback arrives before the node has got
        # a time from /clock or from the system (only seen happen in simulation)
        now = self.node.get_clock().now()
        d = rclpy.duration.Duration(seconds=delta)
        try:
            past = now - d
        except ValueError as e:
            past = now
        return past

    def cam_tags_callback(self, msg, cam):
        # We can only dock using a single camera. If we are in the process of docking, ignore
        # all other cameras
        for tag in msg.data:
            if tag.id in G.carriers:
                if tag.id in G.found_product:
                    continue
                else:
                    G.found_product.append(tag.id)
                if len(list(set(G.found_product).intersection(set(G.carriers)))) == len(G.carriers):
                    self.blackboard.found_or_not = True
            else:
                if tag.id in G.found_product:
                    continue
                else:
                    G.found_product.append(tag.id)

    def cam0_tags_callback(self, msg):
        self.cam_tags_callback(msg, 0)

    def cam1_tags_callback(self, msg):
        self.cam_tags_callback(msg, 1)

    def cam2_tags_callback(self, msg):
        self.cam_tags_callback(msg, 2)

    def cam3_tags_callback(self, msg):
        self.cam_tags_callback(msg, 3)

    def cam4_tags_callback(self, msg):
        self.cam_tags_callback(msg, 4)


    def information_callback(self, msg):
        G.Global_information = G.found_product
        if len(msg.data) == 0:
            self.information_task.data = G.Global_information
        else:
            for i in range(len(msg.data)):
                if msg.data[i] in G.Global_information:
                    continue
                else:
                    G.Global_information.append(msg.data[i])
            self.information_task.data = G.Global_information

    def update(self):

        self.timer = self.node.get_clock().now()
        if len(list(set(G.Global_information).intersection(set(G.carriers)))) == len(G.carriers):
            self.blackboard.found_or_not = True
            G.finish_time = self.timer
        
        self.tasks_pub.publish(self.information_task)
        self.node.get_logger().info('%s %s %s' % (G.finish_time, self.blackboard.found_or_not, self.information_task.data))
            
        return py_trees.common.Status.SUCCESS





def create_root():
    root = py_trees.composites.Parallel(
        name    = 'Simple BT controller',
        policy  = py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )

    topics2bb   = py_trees.composites.Sequence(
        name    = 'Topics2BB',
        memory  = True
    )

    irtof2bb    = py_trees_ros.subscribers.ToBlackboard(
        name                    = 'irtof2bb',
        topic_name              = 'sensor/scan',
        topic_type              = sensor_msgs.msg.PointCloud2,
        qos_profile             = qos_profile_sensor_data,
        blackboard_variables    = {'irtof' : None}
    )

    odom2bb     = py_trees_ros.subscribers.ToBlackboard(
        name                    = 'odom2bb',
        topic_name              = 'odom',
        topic_type              = nav_msgs.msg.Odometry,
        qos_profile             = qos_profile_system_default,
        blackboard_variables    = {'odom' : None}
    )

    #priorities      = py_trees.composites.Sequence(name='Priorities', memory=False)
    priorities      = FakeSequence(name='FS Priorities', memory=False)



    proc_irtof      = Process_irtof(name='Proc irtof')
    proc_vision     = Process_vision(name='Proc vision')

    find_carrier    = py_trees.composites.Selector(name='Find carrier', memory=False)

    avoid_collision  = py_trees.composites.Sequence('Coll avoid', memory=True)
    avoid_collision_sir = py_trees.decorators.SuccessIsRunning(avoid_collision)

    coll_check  = py_trees.behaviours.CheckBlackboardVariableValue(
        name    = 'Obstacles?',
        check   = py_trees.common.ComparisonExpression(
            variable    = 'collision', 
            value       = True, 
            operator    = operator.eq)
    )
    avoid_cmd_vel = py_trees_ros.publishers.FromBlackboard(
        name                = 'avoid cmd_vel',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'avoid_cmd_vel'
    )

    random_wander   = py_trees.composites.Sequence('Wander', memory=True)
    random_wander_sir = py_trees.decorators.SuccessIsRunning(random_wander)

    pick_direction  = Pick_random_direction('Pick direction')
    random_cmd_vel  = py_trees_ros.publishers.FromBlackboard(
        name                = 'random cmd_vel',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'random_cmd_vel'
    )
    wander_delay    = py_trees.behaviours.TickCounter(name='Wander delay', duration=30)


    Last_wander   = py_trees.composites.Sequence('Last', memory=True)
    Last_wander_sir = py_trees.decorators.SuccessIsRunning(Last_wander)

    pick_last_direction  = Pick_last_direction('Pick last direction')
    last_cmd_vel  = py_trees_ros.publishers.FromBlackboard(
        name                = 'last cmd_vel',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'last_cmd_vel'
    )



    fetch           = py_trees.composites.Sequence('Fetch', memory=True)

    product_check  = py_trees.behaviours.CheckBlackboardVariableValue(
        name    = 'found_all?',
        check   = py_trees.common.ComparisonExpression(
            variable    = 'found_or_not', 
            value       = True, 
            operator    = operator.eq)
    )

    zero_cmd_vel  = py_trees_ros.publishers.FromBlackboard(
        name                = 'zero cmd_vel',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'zero_cmd_vel'
    )

    idle = py_trees.behaviours.Success(name='Idle')



    root.add_child(topics2bb)
    topics2bb.add_child(irtof2bb)
    topics2bb.add_child(odom2bb)
    root.add_child(priorities)
    priorities.add_child(proc_irtof)
    priorities.add_child(proc_vision)
    priorities.add_child(fetch)
    priorities.add_child(idle)

    fetch.add_child(find_carrier)

    find_carrier.add_child(product_check)

    find_carrier.add_child(avoid_collision_sir)
    avoid_collision.add_child(coll_check)
    avoid_collision.add_child(avoid_cmd_vel)

    find_carrier.add_child(Last_wander_sir)
    Last_wander.add_child(pick_last_direction)
    Last_wander.add_child(last_cmd_vel)

    fetch.add_child(zero_cmd_vel)


    return root




        

def main():
    rclpy.init()

    #py_trees.logging.level = py_trees.logging.level.DEBUG
    root = create_root()
    #tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.setup()
    tree.tick_tock(period_ms=100.0)
    rclpy.spin(tree.node)



if __name__ == '__main__':
    main()