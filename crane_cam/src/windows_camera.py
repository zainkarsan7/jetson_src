#!/usr/bin/env python3
"""Publish a Windows DirectShow MJPEG camera from WSL."""
import math
import subprocess

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def camera_info(path, width, height):
    info = CameraInfo(width=width, height=height)
    if not path:
        return info
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if (data["image_width"], data["image_height"]) != (width, height):
        raise ValueError("Calibration dimensions must match the requested camera mode")
    info.distortion_model = data["distortion_model"]
    for field, key in (("k", "camera_matrix"), ("d", "distortion_coefficients"),
                       ("r", "rectification_matrix"), ("p", "projection_matrix")):
        values = [float(v) for v in data[key]["data"]]
        if not all(math.isfinite(v) for v in values):
            raise ValueError(f"Non-finite calibration values in {key}")
        setattr(info, field, values)
    models = {"plumb_bob": 5, "rational_polynomial": 8}
    if (models.get(info.distortion_model) != len(info.d) or
            min(info.k[0], info.k[4]) <= 0 or info.k[8] != 1):
        raise ValueError("Expected calibrated plumb_bob or rational_polynomial camera info")
    return info


def jpeg_packets(stream):
    # FFmpeg's mpjpeg muxer prefixes each JPEG with its byte length.
    while True:
        line = stream.readline(1024)
        if not line:
            return
        if line.lower().startswith(b"content-length:"):
            size = int(line.split(b":", 1)[1])
            if not 0 < size <= 64 * 1024 * 1024 or stream.readline(4).strip():
                raise ValueError("Invalid MJPEG packet header")
            packet = stream.read(size)
            if len(packet) != size:
                raise EOFError("Truncated camera frame")
            yield packet


class WindowsCamera(Node):
    def __init__(self):
        super().__init__("windows_camera")
        param = lambda name, default: self.declare_parameter(name, default).value
        device = param("device", "")
        width = param("width", 1920)
        height = param("height", 1080)
        fps = param("fps", 30)
        self.frame = param("frame_id", "innomaker_optical_frame")
        if not device:
            raise ValueError("Set device to the Windows DirectShow camera name")
        self.info = camera_info(param("calibration", ""), width, height)
        self.bridge = CvBridge()
        self.images = self.create_publisher(Image, "image_raw", 1)
        self.infos = self.create_publisher(CameraInfo, "camera_info", 1)
        self.proc = subprocess.Popen([
            "ffmpeg.exe", "-hide_banner", "-loglevel", "error", "-f", "dshow",
            "-video_size", f"{width}x{height}", "-framerate", str(fps),
            "-vcodec", "mjpeg", "-i", f"video={device}", "-an",
            "-c:v", "copy", "-f", "mpjpeg", "pipe:1",
        ], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        if not self.info.k[0]:
            self.get_logger().warn("Detection only until a matching calibration YAML is supplied")

    def capture(self):
        for packet in jpeg_packets(self.proc.stdout):
            if not rclpy.ok():
                return
            stamp = self.get_clock().now().to_msg()
            frame = cv2.imdecode(np.frombuffer(packet, np.uint8), cv2.IMREAD_COLOR)
            if frame is None or frame.shape[:2] != (self.info.height, self.info.width):
                raise ValueError("Invalid camera frame or unexpected capture dimensions")
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame
            self.info.header = msg.header
            self.infos.publish(self.info)
            self.images.publish(msg)
        if rclpy.ok():
            raise RuntimeError("Windows camera stream ended. Check the DirectShow device name and MJPEG mode.")

    def close(self):
        try:
            with self.proc.stdin as controls:
                controls.write(b"q\n")
        except BrokenPipeError:
            pass
        self.proc.stdout.close()
        try:
            self.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()


def main():
    rclpy.init()
    node = WindowsCamera()
    try:
        node.capture()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
