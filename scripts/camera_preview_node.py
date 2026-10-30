"""Publish a live (downscaled) camera feed to ROS for rqt_image_view, so you
can see where to hold the checkerboard without the lag of cv2's own GUI loop
doing per-frame chessboard detection.

  /usr/bin/python3 scripts/camera_preview_node.py
  rqt_image_view   # in another terminal, pick /camera/image_raw

Not a camera driver -- just enough to look at while positioning the board.
Actual calibration photos are still taken separately (capture_photos.py or
capture_calibration_photos.py), at full resolution.
"""
import argparse

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', type=int, default=0)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--topic', default='/camera/image_raw')
    parser.add_argument('--rate', type=float, default=15.0)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    for _ in range(5):
        cap.read()
    if not cap.isOpened():
        raise SystemExit(f'could not open camera /dev/video{args.device}')

    rclpy.init()
    node = Node('camera_preview')
    pub = node.create_publisher(Image, args.topic, 1)
    print(f'publishing {args.topic} -- open rqt_image_view and select it. Ctrl+C to stop.')

    msg = Image()
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    try:
        while rclpy.ok():
            ok, frame = cap.read()
            if not ok:
                rclpy.spin_once(node, timeout_sec=0.01)
                continue
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.height, msg.width = frame.shape[:2]
            msg.step = msg.width * 3
            msg.data = frame.tobytes()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=max(0.0, 1.0 / args.rate))
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
