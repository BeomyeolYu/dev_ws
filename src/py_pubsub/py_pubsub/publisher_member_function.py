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


# The following lines represent the node’s dependencies. 
# Recall that dependencies have to be added to package.xml, 
# which you’ll do in the next section.
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # built-in string message type.


class MinimalPublisher(Node): # which inherits from (or is a subclass of) ``Node``.

    # Following is the definition of the class’s constructor.
    def __init__(self):
        # ``super().__init__`` calls the Node class’s constructor and
        # gives it your node name, in this case minimal_publisher.
        super().__init__('minimal_publisher') 
        # ``create_publisher`` declares that the node publishes messages of type ``String``,
        # over a topic named ``topic``, and that the “queue size” is 10.
        queue_size = 10
        self.publisher_ = self.create_publisher(
            String, 
            'topic', 
            queue_size)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # ``timer_callback`` creates a message with the counter value appended, and
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        # publishes it to the console with ``get_logger().info``.
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args) # First the ``rclpy`` library is initialized,

    minimal_publisher = MinimalPublisher() # then the node is created,

    rclpy.spin(minimal_publisher) # and then it “spins” the node so its callbacks are called.

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
