"""While the car is being driven: does /scan turn the same way as the gyro, and move the same
way as /odom? Aligns scans 0.5 s apart (ICP) and compares with the gyro and odometry over the
same interval. Uses /scan as published (no flip)."""
import importlib.util, math, time, sys
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
spec = importlib.util.spec_from_file_location('ac', __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), 'auto_calibrate.py'))
ac = importlib.util.module_from_spec(spec); spec.loader.exec_module(ac)
ac.LIDAR_FLIP = False
rclpy.init(); n = Node('lidar_live_check'); q = qos_profile_sensor_data
scans, imu, odom = [], [], []
n.create_subscription(LaserScan, '/scan', lambda m: scans.append((time.monotonic(), m)), q)
n.create_subscription(Imu, '/oakd/imu', lambda m: imu.append((time.monotonic(), m.angular_velocity.x,
    m.angular_velocity.y, m.angular_velocity.z, m.linear_acceleration.x, m.linear_acceleration.y,
    m.linear_acceleration.z)), q)
n.create_subscription(Odometry, '/odom', lambda m: odom.append((time.monotonic(), m.twist.twist.linear.x)), q)
T = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
t0 = time.monotonic()
while time.monotonic() - t0 < T: rclpy.spin_once(n, timeout_sec=0.01)
I = np.array(imu); O = np.array(odom)
up = I[:, 4:7].mean(0); up /= np.linalg.norm(up)
yaw = -(I[:, 1:4] @ up)              # sign as calibrated today: left turn positive
yaw -= np.median(yaw[np.abs(yaw) < 0.02]) if (np.abs(yaw) < 0.02).sum() > 50 else 0.0
rot = [0, 0, 0]; mov = [0, 0, 0]; rows = []
for i in range(0, len(scans) - 5, 5):
    (ta, a), (tb, b) = scans[i], scans[i + 5]
    sel = (I[:, 0] >= ta) & (I[:, 0] <= tb); so = (O[:, 0] >= ta) & (O[:, 0] <= tb)
    if sel.sum() < 20 or so.sum() < 5: continue
    dpsi = float(ac.trapz(yaw[sel], I[sel, 0])); d = float(ac.trapz(O[so, 1], O[so, 0]))
    if abs(dpsi) < 0.05 and abs(d) < 0.15: continue          # not moving enough to tell
    r = ac.icp(ac.points(b, None), ac.points(a, None), (d * math.cos(dpsi / 2), d * math.sin(dpsi / 2), dpsi))
    if not r or r['inl'] < 0.6: continue
    rows.append((dpsi, r['th'], d, r['x']))
    if abs(dpsi) >= 0.05: rot[0 if np.sign(r['th']) == np.sign(dpsi) else 1] += 1
    if abs(d) >= 0.15: mov[0 if np.sign(r['x']) == np.sign(d) else 1] += 1
print('pairs used: %d' % len(rows))
print('rotation: lidar agrees with gyro %d, disagrees %d' % (rot[0], rot[1]))
print('travel:   lidar agrees with odom %d, disagrees %d' % (mov[0], mov[1]))
for r in rows[:12]: print('  gyro %+6.1f deg  lidar %+6.1f deg | odom %+.2f m  lidar %+.2f m'
                          % (math.degrees(r[0]), math.degrees(r[1]), r[2], r[3]))
