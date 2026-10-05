"""
OAK-D Pro driver — RGB frames + onboard IMU for the real car.
=============================================================

Publishes from a Luxonis OAK-D Pro over the `depthai` Python API (pure pip,
works on Humble — no depthai-ros packages needed on the Jetson):

  /oakd/rgb          sensor_msgs/Image  (bgr8)   -> camera_perception YOLO
  /oakd/camera_info  sensor_msgs/CameraInfo      -> pinhole back-projection
  /oakd/imu          sensor_msgs/Imu (accel+gyro)-> raceline_mpc traction governor
  /oakd/video        sensor_msgs/CompressedImage (format "h264" or "h265", one Annex-B
                     access unit per message) when video_kbps > 0 -> web_pilot's /vstream. Encoded
                     on the camera's own chip, so the Jetson only forwards bytes.
  /oakd/opponents_rel PoseArray (optional)       -> on-device VPU YOLO: set
                     `yolo_blob` to a compiled .blob and detection runs on the
                     camera's Myriad X — zero host CPU/GPU cost, boxes
                     back-projected to (x fwd, y left) with the factory fx

Design notes:
  - The RGB preview stream is configured interleaved-BGR at the requested size
    so frames go straight into an Image message as raw bytes — no cv_bridge,
    no per-frame colour conversion on the Jetson CPU.
  - Intrinsics come from the device's factory calibration (readCalibration),
    scaled to the published resolution, so camera_perception gets a correct
    fx/cx without hand-tuning.
  - IMU runs at `imu_hz` (accel + gyro, BMI270/BNO086 depending on unit) and is
    drained on a fast timer; orientation is not estimated (covariance[0] = -1).
    Mount note: the governor downstream uses |gyro z|, so any flat (z-up or
    z-down) mounting works without sign config.
  - Sensor-data QoS (best-effort, depth 1): a late frame is a useless frame.

Run:
    ros2 run f1tenth_gym_ros oakd_camera --ros-args --params-file config/hardware.yaml
"""

import time

try:
    import depthai as dai
except Exception:                       # pragma: no cover — not on dev machines
    dai = None


def build_pipeline(width, height, fps, imu_hz, yolo_blob='', yolo_conf=0.4,
                   yolo_size=416, video_kbps=0, video_keyframe=15,
                   video_codec='h264'):
    """depthai pipeline: RGB preview (interleaved BGR) + raw accel/gyro IMU,
    plus optional on-device YOLO on the Myriad X VPU when `yolo_blob` set,
    plus optional on-device H.264/H.265 for the pilot page when `video_kbps` > 0."""
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setPreviewSize(int(width), int(height))
    cam.setInterleaved(True)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setFps(float(fps))
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName('rgb')
    cam.preview.link(xout_rgb.input)

    if video_kbps:
        # H.264 (or H.265) for the pilot page, encoded by the camera's own video encoder. Baseline
        # profile (no B-frames, so nothing waits for a future frame), constant bitrate so
        # the stream fits the radio link, and a keyframe every `video_keyframe` frames so a
        # lost or skipped frame heals within half a second at 30 fps.
        w, h = int(width), int(height)
        k = 1920 // w
        if 1920 % w == 0 and 1080 % h == 0 and 1080 // h == k:
            cam.setIspScale(1, k)            # 1080p / 3 = 640x360: same picture as the preview
            cam.setVideoSize(w, h)
            enc_in = cam.video
        else:                                # other sizes: resize the video stream on the camera
            manip_v = pipeline.create(dai.node.ImageManip)
            manip_v.initialConfig.setResize(w, h)
            manip_v.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
            manip_v.setMaxOutputFrameSize(w * h * 3 // 2)
            cam.video.link(manip_v.inputImage)
            enc_in = manip_v.out
        enc = pipeline.create(dai.node.VideoEncoder)
        prof = (dai.VideoEncoderProperties.Profile.H265_MAIN if video_codec == 'h265'
                else dai.VideoEncoderProperties.Profile.H264_BASELINE)
        enc.setDefaultProfilePreset(float(fps), prof)
        enc.setRateControlMode(dai.VideoEncoderProperties.RateControlMode.CBR)
        enc.setBitrateKbps(int(video_kbps))
        enc.setKeyframeFrequency(int(video_keyframe))
        enc.setNumBFrames(0)
        enc_in.link(enc.input)
        xout_video = pipeline.create(dai.node.XLinkOut)
        xout_video.setStreamName('video')
        enc.bitstream.link(xout_video.input)

    if yolo_blob:
        # NN wants planar input at its own size: convert on-device, then run
        # the detector entirely on the camera's VPU (no host involvement)
        manip = pipeline.create(dai.node.ImageManip)
        manip.initialConfig.setResize(int(yolo_size), int(yolo_size))
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        manip.setMaxOutputFrameSize(int(yolo_size) * int(yolo_size) * 3)
        cam.preview.link(manip.inputImage)
        nn = pipeline.create(dai.node.YoloDetectionNetwork)
        nn.setBlobPath(yolo_blob)
        nn.setConfidenceThreshold(float(yolo_conf))
        nn.setNumClasses(1)                       # car detector
        nn.setCoordinateSize(4)
        nn.setIouThreshold(0.45)
        nn.input.setBlocking(False)
        manip.out.link(nn.input)
        xout_det = pipeline.create(dai.node.XLinkOut)
        xout_det.setStreamName('det')
        nn.out.link(xout_det.input)

    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(
        [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW],
        int(imu_hz))
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(20)
    xout_imu = pipeline.create(dai.node.XLinkOut)
    xout_imu.setStreamName('imu')
    imu.out.link(xout_imu.input)
    return pipeline


def _make_node():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo, Imu, CompressedImage
    from geometry_msgs.msg import PoseArray, Pose

    class OakDCamera(Node):
        def __init__(self):
            super().__init__('oakd_camera')
            self.declare_parameter('width', 640)
            self.declare_parameter('height', 360)
            self.declare_parameter('fps', 30.0)
            self.declare_parameter('imu_hz', 200)
            self.declare_parameter('rgb_topic', '/oakd/rgb')
            self.declare_parameter('info_topic', '/oakd/camera_info')
            self.declare_parameter('imu_topic', '/oakd/imu')
            self.declare_parameter('camera_frame', 'oakd_rgb')
            self.declare_parameter('imu_frame', 'oakd_imu')
            self.declare_parameter('yolo_blob', '')    # on-device VPU detector
            self.declare_parameter('yolo_conf', 0.4)
            self.declare_parameter('yolo_size', 416)
            self.declare_parameter('car_width', 0.30)  # m, for back-projection
            self.declare_parameter('video_kbps', 0)     # >0: on-camera video for the pilot page
            self.declare_parameter('video_codec', 'h264')  # h264 or h265 (the encoder has no AV1)
            self.declare_parameter('video_keyframe', 15)
            self.declare_parameter('video_topic', '/oakd/video')
            p = lambda n: self.get_parameter(n).value   # noqa: E731
            self.w, self.h = int(p('width')), int(p('height'))
            self.cam_frame = p('camera_frame')
            self.imu_frame = p('imu_frame')
            self.car_w = float(p('car_width'))

            if dai is None:
                raise RuntimeError('depthai not installed — pip3 install depthai')

            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.pub_rgb = self.create_publisher(Image, p('rgb_topic'), qos)
            self.pub_info = self.create_publisher(CameraInfo, p('info_topic'), qos)
            self.pub_imu = self.create_publisher(Imu, p('imu_topic'),
                                                 QoSProfile(depth=50))

            self.video_kbps = int(p('video_kbps'))
            self.codec = 'h265' if str(p('video_codec')).lower() in ('h265', 'hevc') else 'h264'
            self.device = dai.Device(build_pipeline(
                self.w, self.h, p('fps'), p('imu_hz'),
                yolo_blob=p('yolo_blob'), yolo_conf=float(p('yolo_conf')),
                yolo_size=int(p('yolo_size')), video_kbps=self.video_kbps,
                video_keyframe=int(p('video_keyframe')), video_codec=self.codec))
            self.q_rgb = self.device.getOutputQueue('rgb', maxSize=2, blocking=False)
            # capture-to-host latency and video bitrate, logged every 10 s
            self.stats = {'rgb_lat': [], 'h264_lat': [], 'h264_bytes': 0, 'h264_n': 0,
                          'h264_keys': 0, 't0': time.monotonic()}
            self.q_h264 = None
            if self.video_kbps:
                self.q_h264 = self.device.getOutputQueue('video', maxSize=30, blocking=False)
                # reliable and deep: a dropped delta frame breaks decoding until the next keyframe
                self.pub_h264 = self.create_publisher(
                    CompressedImage, p('video_topic'),
                    QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE))
                self.create_timer(1.0 / (2.0 * float(p('fps'))), self._poll_h264)
                self.get_logger().info(
                    f"on-camera {self.codec.upper()} {self.video_kbps} kbit/s, keyframe every "
                    f"{int(p('video_keyframe'))} frames -> {p('video_topic')}")
            self.create_timer(10.0, self._log_stats)
            self.q_imu = self.device.getOutputQueue('imu', maxSize=50, blocking=False)
            self.info = self._camera_info()
            self.q_det = None
            if p('yolo_blob'):
                self.q_det = self.device.getOutputQueue('det', maxSize=2,
                                                        blocking=False)
                self.pub_det = self.create_publisher(
                    PoseArray, '/oakd/opponents_rel', qos)
                self.create_timer(1.0 / (2.0 * float(p('fps'))), self._poll_det)
                self.get_logger().info(
                    'on-device YOLO active (Myriad X VPU) -> /oakd/opponents_rel')
            self.create_timer(1.0 / (2.0 * float(p('fps'))), self._poll_rgb)
            self.create_timer(0.002, self._poll_imu)
            self.get_logger().info(
                f'oakd_camera ready — rgb {self.w}x{self.h}@{p("fps"):.0f} '
                f'imu @{p("imu_hz")}Hz ({self.device.getMxId()})')

        def _camera_info(self):
            info = CameraInfo()
            info.header.frame_id = self.cam_frame
            info.width, info.height = self.w, self.h
            try:
                calib = self.device.readCalibration()
                K = calib.getCameraIntrinsics(
                    dai.CameraBoardSocket.RGB, self.w, self.h)
                info.k = [float(v) for row in K for v in row]
                info.p = [info.k[0], info.k[1], info.k[2], 0.0,
                          info.k[3], info.k[4], info.k[5], 0.0,
                          info.k[6], info.k[7], info.k[8], 0.0]
                info.distortion_model = 'plumb_bob'
                d = calib.getDistortionCoefficients(dai.CameraBoardSocket.RGB)
                info.d = [float(v) for v in d[:5]]
            except Exception as e:                      # uncalibrated dev unit
                self.get_logger().warning(f'no factory calibration: {e}')
            return info

        def _age_ms(self, frame):
            """Capture-to-now in ms: depthai timestamps run on the host's steady clock."""
            try:
                return (dai.Clock.now() - frame.getTimestamp()).total_seconds() * 1000.0
            except Exception:
                return None

        def _poll_h264(self):
            for pkt in self.q_h264.tryGetAll():
                data = pkt.getData().tobytes()
                lat = self._age_ms(pkt)
                if lat is not None:
                    self.stats['h264_lat'].append(lat)
                self.stats['h264_bytes'] += len(data)
                self.stats['h264_n'] += 1
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.cam_frame
                msg.format = self.codec
                msg.data = data
                self.pub_h264.publish(msg)

        def _log_stats(self):
            s, now = self.stats, time.monotonic()
            dt = max(1e-3, now - s['t0'])

            def p50_p95(v):
                v = sorted(v)
                return (v[len(v) // 2], v[int(len(v) * 0.95)]) if v else (float('nan'),) * 2
            r50, r95 = p50_p95(s['rgb_lat'])
            line = f'capture->host: rgb p50 {r50:.1f} ms p95 {r95:.1f} ms'
            if self.q_h264 is not None:
                h50, h95 = p50_p95(s['h264_lat'])
                line += (f' | {self.codec} p50 {h50:.1f} ms p95 {h95:.1f} ms, '
                         f"{s['h264_bytes'] * 8 / dt / 1000:.0f} kbit/s, {s['h264_n'] / dt:.1f} fps")
            self.get_logger().info(line)
            self.stats = {'rgb_lat': [], 'h264_lat': [], 'h264_bytes': 0, 'h264_n': 0,
                          'h264_keys': 0, 't0': now}

        def _poll_rgb(self):
            frame = self.q_rgb.tryGet()
            if frame is None:
                return
            lat = self._age_ms(frame)
            if lat is not None:
                self.stats['rgb_lat'].append(lat)
            stamp = self.get_clock().now().to_msg()
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.cam_frame
            msg.height, msg.width = self.h, self.w
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = self.w * 3
            msg.data = frame.getData().tobytes()
            self.pub_rgb.publish(msg)
            self.info.header.stamp = stamp
            self.pub_info.publish(self.info)

        def _poll_det(self):
            """On-device detections -> relative poses (x fwd, y left, base frame).

            Same pinhole back-projection camera_perception uses: depth from
            the known car width and the factory fx, scaled to preview size.
            """
            det = self.q_det.tryGet()
            if det is None:
                return
            fx = self.info.k[0] if self.info.k[0] else 600.0
            cx = self.info.k[2] if self.info.k[2] else self.w / 2.0
            pa = PoseArray()
            pa.header.stamp = self.get_clock().now().to_msg()
            pa.header.frame_id = self.cam_frame
            for d in det.detections:
                w_px = max((d.xmax - d.xmin) * self.w, 1.0)
                u_center = (d.xmin + d.xmax) / 2.0 * self.w
                depth = self.car_w * fx / w_px
                y_left = -(u_center - cx) * depth / fx
                pose = Pose()
                pose.position.x = float(depth)
                pose.position.y = float(y_left)
                pose.orientation.w = float(d.confidence)
                pa.poses.append(pose)
            self.pub_det.publish(pa)

        def _poll_imu(self):
            data = self.q_imu.tryGet()
            if data is None:
                return
            for pkt in data.packets:
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.imu_frame
                acc, gyr = pkt.acceleroMeter, pkt.gyroscope
                msg.linear_acceleration.x = float(acc.x)
                msg.linear_acceleration.y = float(acc.y)
                msg.linear_acceleration.z = float(acc.z)
                msg.angular_velocity.x = float(gyr.x)
                msg.angular_velocity.y = float(gyr.y)
                msg.angular_velocity.z = float(gyr.z)
                msg.orientation_covariance[0] = -1.0    # orientation not provided
                self.pub_imu.publish(msg)

        def shutdown(self):
            try:
                self.device.close()
            except Exception:
                pass

    return rclpy, OakDCamera


def main(args=None):
    rclpy, NodeCls = _make_node()
    rclpy.init(args=args)
    node = None
    try:
        node = NodeCls()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if node is not None:
            node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
