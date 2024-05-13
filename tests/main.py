#!/usr/bin/env python3

"""Test tello."""

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__authors__ = 'Daniel Fernández Sánchez'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'

from time import sleep

from as2_python_api.drone_interface import DroneInterface
import rclpy


def arm_test(uav: DroneInterface):
    """Arm test."""
    uav.offboard()
    print('Offboard')
    sleep(1)

    uav.arm()
    print('Armed')
    sleep(1)


def takeoff_land_test(uav: DroneInterface):
    """Takeoff and land."""
    uav.takeoff()
    sleep(5)

    uav.land()


def command_pose_test(uav: DroneInterface):
    """Pose test."""
    uav.takeoff()
    sleep(2)

    uav.send_motion_reference_pose(position=[0, 0, 0.5])
    sleep(5)

    uav.send_motion_reference_pose(position=[0, 0, 1.0])
    sleep(5)

    uav.send_motion_reference_pose(position=[0, 0, 0.3])
    sleep(5)
    uav.land()


if __name__ == '__main__':
    rclpy.init()

    drone_id = 'tello'
    print('Connecting to: ', drone_id)
    drone = DroneInterface(drone_id, True)

    takeoff_land_test(drone)

    drone.shutdown()

    print('Bye')

    rclpy.shutdown()
