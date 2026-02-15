#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

import rclpy
import numpy as np
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

                # QoS profiles
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(     
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile_sub)
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile_sub)
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            'fmu/in/offboard_control_mode', 
            qos_profile_pub)
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, 
            'fmu/in/trajectory_setpoint', 
            qos_profile_pub)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period
        
        self.declare_parameter('radius', 15.0)
        self.declare_parameter('omega', 35)
        self.declare_parameter('altitude', 5.0)
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        # Note: no parameter callbacks are used to prevent sudden inflight changes of radii and omega
        # which would result in large discontinuities in setpoints
        self.theta = 0.0
        self.offset = 0.0
        self.yoffset = 0.0
        self.d = 0.0
        self.state = 1
        self.center1 = [35, 45]
        self.center2 = [0, 0]
        self.center3 = [0, 0]
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value
        

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            pi = 3.14
            trajectory_msg = TrajectorySetpoint()
            if(self.state == 1):
                if(self.theta <= 100):
                    trajectory_msg.position[0] = 45
                    trajectory_msg.position[1] = 20
                    trajectory_msg.position[2] = -self.altitude
                    self.theta = self.theta + 1
                else:
                    self.theta = 180
                    self.state = self.state + 1
                    self.radius = 17
                    
            if(self.state == 2):    #ring_1
                if(self.theta <= 590):
                    trajectory_msg.position[1] = self.radius * np.cos(pi * self.theta / 180) + self.center1[0] #x
                    trajectory_msg.position[0] = self.radius * np.sin(pi * self.theta / 180) + self.center1[1] #y
                    trajectory_msg.position[2] = -self.altitude
                    self.theta = self.theta + self.omega * self.dt
                else:
                    x = self.radius * np.cos(pi * self.theta / 180) + self.center1[0]
                    y = self.radius * np.sin(pi * self.theta / 180) + self.center1[1]
                    self.radius = 20
                    self.center2 = [x + self.radius * 0.866, y - self.radius / 2]
                    self.theta = 150
                    self.state = self.state + 1
                    
            if(self.state == 3):    #ring_2
                if(self.theta <= 800):
                    trajectory_msg.position[1] = self.radius * np.cos(pi * self.theta / 180) + self.center2[0] #x
                    trajectory_msg.position[0] = self.radius * np.sin(pi * self.theta / 180) + self.center2[1] #y
                    trajectory_msg.position[2] = -self.altitude
                    self.theta = self.theta + self.omega * self.dt
                else:
                    x = self.radius * np.cos(pi * self.theta / 180) + self.center1[0]
                    y = self.radius * np.sin(pi * self.theta / 180) + self.center1[1]
                    self.radius = 13
                    self.center3 = [x + self.radius, y - self.radius * 2]
                    self.theta = 180
                    self.offset = 0.0
                    self.omega = 40
                    self.state = self.state + 1
                    
            if(self.state == 4):    #stem
                if(self.theta <= 3960):
                    trajectory_msg.position[1] = self.radius * np.cos(pi * self.theta / 180) + self.center3[0] + self.offset     #x
                    trajectory_msg.position[0] = self.radius * np.sin(pi * self.theta / 180) + self.center3[1] + self.yoffset  #y
                    trajectory_msg.position[2] = -self.altitude
                    self.theta = self.theta + self.omega * self.dt
                    self.offset = self.offset + 0.07
                    self.yoffset = self.yoffset + self.d
                    self.d = self.d + 0.0001
                else:
                    self.state = self.state + 1
            
            if(self.state == 5):    #stem2
                trajectory_msg.position[1] = self.radius * np.cos(pi * self.theta / 180) + self.center3[0] + self.offset     #x
                trajectory_msg.position[0] = self.radius * np.sin(pi * self.theta / 180) + self.center3[1] + self.yoffset  #y
                trajectory_msg.position[2] = -self.altitude
                self.theta = self.theta + self.omega * self.dt
                 
            self.publisher_trajectory.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
