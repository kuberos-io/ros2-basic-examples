# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.declare_parameter('init_topic', 'world - init')
        self.declare_parameter('topic', 'world')
        self.declare_parameter('talker', 'none')
        self.declare_parameter('content', 'content')
        
        # get initial parameters 
        self.init_topic = self.get_parameter('init_topic').get_parameter_value().string_value
        
        self.talker = self.get_parameter('talker').get_parameter_value().string_value
        
        # get content from parameter server
        self.content = self.get_parameter('content').get_parameter_value().string_value
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        
        self.content = self.get_parameter('content').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        
        if self.i < 10:
            topic = self.init_topic
        else:
            topic = self.topic
        
        msg.data = 'Hello - {} from <{}> - [{}]: {}'.format(topic, 
                                                            self.talker, 
                                                            self.i, 
                                                            self.content)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
