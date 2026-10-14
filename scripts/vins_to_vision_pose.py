import math
import time
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState

rospy.init_node('vins_to_vision_pose')
latest, received = {}, {}

def receive(msg, key):
    latest[key] = msg
    received[key] = time.monotonic()

subs = [
    rospy.Subscriber('/vins_estimator/imu_propagate',
                     Odometry, receive, callback_args='vins',
                     queue_size=1, tcp_nodelay=True),
    rospy.Subscriber('/mavros/imu/data',
                     Imu, receive, callback_args='attitude',
                     queue_size=1),
    rospy.Subscriber('/mavros/state',
                     State, receive, callback_args='state'),
    rospy.Subscriber('/mavros/extended_state',
                     ExtendedState, receive, callback_args='land')
]

def normalized(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(x) for x in values):
        raise ValueError('四元数含非有限值')
    norm = math.sqrt(sum(x*x for x in values))
    if norm < 0.5 or norm > 1.5:
        raise ValueError('四元数模长异常')
    return tuple(x/norm for x in values)

def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

def body_position(msg, q):
    p = msg.pose.pose.position
    x, y, z, w = q
    # IMU位于机体中心上方0.02m，换算为机体中心位置。
    position = (
        p.x-0.02*2*(x*z+w*y),
        p.y-0.02*2*(y*z-w*x),
        p.z-0.02*(1-2*(x*x+y*y))
    )
    if not all(math.isfinite(v) for v in position):
        raise ValueError('位置含非有限值')
    return position

print('等待VINS、飞控姿态和地面状态，最多20秒……', flush=True)
deadline = time.monotonic()+20
while not rospy.is_shutdown():
    ready = all(
        key in latest and time.monotonic()-received[key] < 1.5
        for key in ('vins', 'attitude', 'state', 'land')
    )
    if ready:
        break
    if time.monotonic() > deadline:
        raise RuntimeError('缺少输入数据，请发送本终端输出')
    time.sleep(0.1)

if not latest['state'].connected:
    raise RuntimeError('MAVROS未连接')
if latest['state'].armed or latest['land'].landed_state != 1:
    raise RuntimeError('请在着地且上锁状态下启动转发')

initial = latest['vins']
q0 = normalized(initial.pose.pose.orientation)
fcu_q = normalized(latest['attitude'].orientation)
origin = body_position(initial, q0)
angle = yaw(fcu_q)-yaw(q0)
c, s = math.cos(angle), math.sin(angle)
qc, qs = math.cos(angle/2), math.sin(angle/2)

publisher = rospy.Publisher('/mavros/vision_pose/pose',
                            PoseStamped, queue_size=10)
last_stamp = [None]
count = [0]

def publish(event):
    msg = latest.get('vins')
    if msg is None:
        return

    age = rospy.Time.now().to_sec()-msg.header.stamp.to_sec()
    if (time.monotonic()-received['vins'] > 1.0
            or age > 0.1 or age < -0.05):
        rospy.logwarn_throttle(5, 'VINS数据过期或时间戳异常，暂停发送')
        return

    stamp = msg.header.stamp.to_nsec()
    if last_stamp[0] is not None and stamp <= last_stamp[0]:
        return

    try:
        q = normalized(msg.pose.pose.orientation)
        p = body_position(msg, q)
        dx, dy, dz = [p[i]-origin[i] for i in range(3)]
        position = (c*dx-s*dy, s*dx+c*dy, dz)
        if math.sqrt(sum(v*v for v in position)) > 10:
            raise ValueError('相对位置超过本次实验的10米范围')
    except ValueError as error:
        rospy.logwarn_throttle(5, '暂停发送：%s' % error)
        return

    x, y, z, w = q
    out = PoseStamped()
    out.header.stamp = msg.header.stamp
    out.header.frame_id = 'map'
    out.pose.position.x = position[0]
    out.pose.position.y = position[1]
    out.pose.position.z = position[2]
    out.pose.orientation.x = qc*x-qs*y
    out.pose.orientation.y = qc*y+qs*x
    out.pose.orientation.z = qc*z+qs*w
    out.pose.orientation.w = qc*w-qs*z

    publisher.publish(out)
    last_stamp[0] = stamp
    count[0] += 1
    rospy.loginfo_throttle(
        5, '外部视觉转发运行：累计%d条，数据年龄%.3fs' %
        (count[0], age))

print('航向对齐角：%.2f度' % math.degrees(angle), flush=True)
print('转发已启动：imu_propagate → vision_pose/pose，目标50Hz',
      flush=True)
timer = rospy.Timer(rospy.Duration(0.02), publish)
rospy.spin()
