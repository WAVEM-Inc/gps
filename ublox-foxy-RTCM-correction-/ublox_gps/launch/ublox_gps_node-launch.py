# Copyright 2020 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
# * Neither the name of {copyright_holder} nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch the ublox gps node with c94-m8p configuration."""

import os

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    config_directory = os.path.join(
            ament_index_python.packages.get_package_share_directory('ublox_gps'),
            'config')
    params = os.path.join(config_directory, 'zed_f9p.yaml')
    ublox_gps_node = launch_ros.actions.Node(package='ublox_gps',
            executable='ublox_gps_node',
            output='both',
            parameters=[params])

    ntrip_client_node = launch_ros.actions.Node(
            name='ntrip_client_node',
            namespace='ntrip_client',
            package='ntrip_client',
            executable='ntrip_ros.py',
            output='both',
            parameters=[
                {
                    # Required parameters used to connect to the NTRIP server
                    'host': 'RTS1.ngii.go.kr',
                    'port': 2101,
                    'mountpoint': 'RTK-RTCM32',

                    # Optional parameter that will set the NTRIP version in the initial HTTP request to the NTRIP caster.
                    'ntrip_version': 'None',

                    # If this is set to true, we will read the username and password and attempt to authenticate. If not, we will attempt to connect unauthenticated
                    'authenticate': True,

                    # If authenticate is set the true, we will use these to authenticate with the server
                    'username': 'robotgps23',
                    'password': 'ngii',

                    # Whether to connect with SSL. cert, key, and ca_cert options will only take effect if this is true
                    'ssl': False,

                    # If the NTRIP caster uses cert based authentication, you can specify the cert and keys to use with these options
                    'cert': 'None',
                    'key':  'None',

                    # If the NTRIP caster uses self signed certs, or you need to use a different CA chain, specify the path to the file here
                    'ca_cert': 'None',

                    # Not sure if this will be looked at by other ndoes, but this frame ID will be added to the RTCM messages published by this node
                    'rtcm_frame_id': 'odom',

                    # Optional parameters that will allow for longer or shorter NMEA messages. Standard max length for NMEA is 82
                    'nmea_max_length': 82,
                    'nmea_min_length': 3,

                    # Use this parameter to change the type of RTCM message published by the node. Defaults to "mavros_msgs", but we also support "rtcm_msgs"
                    'rtcm_message_package': 'mavros_msgs',

                    # Will affect how many times the node will attempt to reconnect before exiting, and how long it will wait in between attempts when a reconnect occurs
                    'reconnect_attempt_max': 10,
                    'reconnect_attempt_wait_seconds': 5,

                    # How many seconds is acceptable in between receiving RTCM. If RTCM is not received for this duration, the node will attempt to reconnect
                    'rtcm_timeout_seconds': 4
                    }
                ],
                remappings=[
                  ("/ntrip_client/nmea", "/nmea"),
                  ("/ntrip_client/rtcm", "/rtcm")
                ],
            )

    return launch.LaunchDescription([ublox_gps_node,ntrip_client_node,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=ublox_gps_node,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
                )),
            ])
