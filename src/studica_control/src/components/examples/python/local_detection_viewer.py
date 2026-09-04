#!/usr/bin/env python3
"""Display the compressed detection overlay on the Raspberry Pi desktop."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class LocalDetectionViewer(Node):
    def __init__(self):
        super().__init__('local_detection_viewer')
        self.window = 'Studica RGB-D Object Tracking'
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.create_subscription(
            CompressedImage,
            '/object_detection/debug_image/compressed',
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info('Local viewer ready; press Q or Esc in the window to close')

    def image_callback(self, message):
        frame = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        cv2.imshow(self.window, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            rclpy.shutdown()

    def close(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = LocalDetectionViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
