#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import time

import cv2
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image


class UsbCameraNode(object):

    def __init__(self):
        self.device = str(rospy.get_param("~device", "/dev/video0"))
        self.width = int(rospy.get_param("~width", 640))
        self.height = int(rospy.get_param("~height", 480))
        self.fps = float(rospy.get_param("~fps", 30.0))
        # MJPG / YUYV, "" = формат драйвера по умолчанию
        self.fourcc = str(rospy.get_param("~fourcc", "MJPG"))
        self.grayscale = bool(rospy.get_param("~grayscale", True))
        self.frame_id = rospy.get_param("~frame_id", "camera_link")
        # Задержка между экспозицией и возвратом grab(), с
        self.time_offset = float(rospy.get_param("~time_offset", 0.0))
        self.reconnect_delay = float(rospy.get_param("~reconnect_delay", 2.0))
        self.max_grab_failures = int(rospy.get_param("~max_grab_failures", 10))
        self.stats_period = float(rospy.get_param("~stats_period", 5.0))
        image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        info_topic = rospy.get_param("~camera_info_topic", "/camera/camera_info")

        self.camera_info = self.load_camera_info(rospy.get_param("~camera_info_file", ""))
        self.image_pub = rospy.Publisher(image_topic, Image, queue_size=1)
        self.info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=1)

        self.bridge = CvBridge()
        self.cap = None
        self.frame_count = 0
        self.last_stamp = None

    def load_camera_info(self, path):
        """Читает yaml в формате camera_calibration."""
        if not path:
            rospy.logwarn("camera_info_file not set, camera_info will not be published")
            return None
        try:
            with open(path) as f:
                calib = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr("Failed to read %s: %s", path, e)
            return None

        info = CameraInfo()
        info.width = int(calib["image_width"])
        info.height = int(calib["image_height"])
        info.distortion_model = calib.get("distortion_model", "plumb_bob")
        info.K = calib["camera_matrix"]["data"]
        info.D = calib["distortion_coefficients"]["data"]
        info.R = calib["rectification_matrix"]["data"]
        info.P = calib["projection_matrix"]["data"]

        if (info.width, info.height) != (self.width, self.height):
            rospy.logwarn(
                "Calibration size %dx%d != capture size %dx%d",
                info.width,
                info.height,
                self.width,
                self.height,
            )
        rospy.loginfo("Loaded calibration %s", path)
        return info

    def open_camera(self):
        # V4L2 backend в OpenCV 4.1 надёжнее открывает устройство по индексу
        m = re.match(r"^/dev/video(\d+)$", self.device)
        cap = cv2.VideoCapture(int(m.group(1)) if m else self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            rospy.logerr("Failed to open camera %s", self.device)
            return None

        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # не копим кадры в драйвере, нужен самый свежий

        code = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc = "".join(chr((code >> 8 * i) & 0xFF) for i in range(4))
        rospy.loginfo(
            "Camera %s opened: %dx%d @ %.1f fps, %s",
            self.device,
            cap.get(cv2.CAP_PROP_FRAME_WIDTH),
            cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
            cap.get(cv2.CAP_PROP_FPS),
            fourcc,
        )
        return cap

    def close_camera(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = None

    def publish(self, frame, stamp):
        if self.grayscale and frame.ndim == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        msg = self.bridge.cv2_to_imgmsg(frame, "mono8" if frame.ndim == 2 else "bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.header.seq = self.frame_count
        self.image_pub.publish(msg)

        if self.camera_info is not None:
            self.camera_info.header = msg.header
            self.info_pub.publish(self.camera_info)

    def run(self):
        failures = 0
        stats_count, stats_start = 0, time.time()

        while not rospy.is_shutdown():
            if self.cap is None:
                self.cap = self.open_camera()
                if self.cap is None:
                    rospy.sleep(self.reconnect_delay)
                    continue
                failures = 0

            # stamp сразу после grab(), до декодирования кадра
            ok = self.cap.grab()
            stamp = rospy.Time.now() - rospy.Duration.from_sec(self.time_offset)
            frame = self.cap.retrieve()[1] if ok else None

            if frame is None:
                failures += 1
                if failures >= self.max_grab_failures:
                    rospy.logerr("Camera %s: %d failed grabs, reopening", self.device, failures)
                    self.close_camera()
                    rospy.sleep(self.reconnect_delay)
                continue
            failures = 0

            if self.last_stamp is not None and stamp <= self.last_stamp:
                rospy.logwarn("Non-monotonic camera stamp, frame dropped")
                continue
            self.last_stamp = stamp

            self.publish(frame, stamp)
            self.frame_count += 1
            stats_count += 1

            elapsed = time.time() - stats_start
            if elapsed >= self.stats_period:
                rate = stats_count / elapsed
                log = rospy.logwarn if rate < 20.0 else rospy.loginfo  # ТЗ: VO >= 20 Гц
                log("Camera rate %.1f Hz, frames=%d", rate, self.frame_count)
                stats_count, stats_start = 0, time.time()


def main():
    rospy.init_node("usb_camera")
    node = UsbCameraNode()
    try:
        node.run()
    finally:
        node.close_camera()


if __name__ == "__main__":
    main()
