#!/usr/bin/env python3
"""PX4 v1.14.3 / ROS Noetic / MAVROS 1.20.1: 首次无GPS POSCTL测试。

仅用于当前Gazebo仿真。MAVROS 1.20.1 manual_control/send 使用原始MAVLink
数值：z=0最低、500居中、650上升、350下降。Gazebo真值仅记录进bag，
不参与控制。运行前停止原来的最低油门发布程序，保留仿真、VINS和视觉转发。
--motion追加机头方向的前后左右小幅移动、松杆保持，以及升降保持测试。
"""
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, ManualControl, State
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode
from mavros_msgs.msg import ParamValue
from nav_msgs.msg import Odometry


def xyz(msg, odometry=False):
    p = msg.pose.pose.position if odometry else msg.pose.position
    return (p.x, p.y, p.z)


def validate(data, now_wall, now_sim, base=None, flight=False):
    """只检查飞控、VINS及转发数据；不读取Gazebo真值。"""
    for name in ('state', 'land', 'pose', 'vins', 'vision'):
        if name not in data:
            raise RuntimeError('尚未收到 ' + name)
        if now_wall - data[name][1] > (3.0 if name in ('state', 'land') else 1.0):
            raise RuntimeError(name + ' 超过接收超时')
    if not data['state'][0].connected:
        raise RuntimeError('飞控连接已断开')
    for name in ('pose', 'vins', 'vision'):
        msg = data[name][0]
        age = now_sim - msg.header.stamp.to_sec()
        if not -0.05 <= age <= (0.3 if name == 'pose' else 0.25):
            raise RuntimeError('%s 时间戳异常：数据年龄 %.3fs' % (name, age))
        p = xyz(msg, name == 'vins')
        if not all(math.isfinite(v) for v in p):
            raise RuntimeError(name + ' 位置含NaN/Inf')
        q = msg.pose.pose.orientation if name == 'vins' else msg.pose.orientation
        norm = math.sqrt(sum(v*v for v in (q.x, q.y, q.z, q.w)))
        if not math.isfinite(norm) or not 0.9 < norm < 1.1:
            raise RuntimeError(name + ' 四元数异常')
    if flight:
        if data['state'][0].mode != 'POSCTL':
            raise RuntimeError('飞行模式发生变化：' + data['state'][0].mode)
        if not data['state'][0].armed:
            raise RuntimeError('起飞阶段飞控已上锁')
        p, v = xyz(data['pose'][0]), xyz(data['vins'][0], True)
        dx, dy, dz = [a-b for a, b in zip(p, base['pose'])]
        if math.hypot(dx, dy) > 1.0 or not -0.3 <= dz <= 1.8:
            raise RuntimeError('飞控位置超过本次测试范围')
        if abs((v[2]-base['vins'][2])-dz) > 0.8:
            raise RuntimeError('VINS与飞控相对高度差超过0.8米')


def near_ground_for_low_throttle(data, base, now_wall, now_sim):
    """仅用新鲜的飞控/VINS相对高度判断收尾油门，不代替着地标志。"""
    if base is None:
        return False
    try:
        validate(data, now_wall, now_sim)
    except RuntimeError:
        return False
    dz = xyz(data['pose'][0])[2]-base['pose'][2]
    vz = xyz(data['vins'][0], True)[2]-base['vins'][2]
    return -0.2 <= dz <= 0.06 and -0.2 <= vz <= 0.10 and abs(dz-vz) <= 0.10


def main():
    parser = argparse.ArgumentParser(description='Gazebo中无GPS POSCTL悬停与降落测试')
    parser.add_argument('--hover', type=float, default=10.0,
                        help='悬停仿真秒数，范围5~30，默认10')
    parser.add_argument('--motion', action='store_true',
                        help='悬停后追加前后左右小幅移动、松杆保持和升降保持')
    args = parser.parse_args()
    if not 5 <= args.hover <= 30:
        parser.error('--hover必须为5~30个仿真秒')
    rospy.init_node('posctl_first_hover_test', disable_signals=True)
    latest, lock = {}, threading.Lock()
    command = {'x': 0.0, 'y': 0.0, 'r': 0.0, 'z': 0.0,
               'abort': False, 'lease': time.monotonic()}
    stop_sender = threading.Event()
    subs = []
    pub = rospy.Publisher('/mavros/manual_control/send', ManualControl, queue_size=1)

    def callback(msg, key):
        with lock:
            latest[key] = (msg, time.monotonic())

    for key, topic, kind in (
        ('state', '/mavros/state', State),
        ('land', '/mavros/extended_state', ExtendedState),
        ('pose', '/mavros/local_position/pose', PoseStamped),
        ('vins', '/vins_estimator/imu_propagate', Odometry),
        ('vision', '/mavros/vision_pose/pose', PoseStamped),
    ):
        subs.append(rospy.Subscriber(topic, kind, callback, callback_args=key, queue_size=1))

    def snapshot():
        with lock:
            return dict(latest)

    def sender():
        msg = ManualControl()
        while not stop_sender.is_set() and not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            msg.buttons = 0
            moving = command['z'] > 500 or any(command[k] != 0 for k in ('x', 'y', 'r'))
            if moving and time.monotonic()-command['lease'] > 0.7:
                command['abort'] = True
                command['x'] = command['y'] = command['r'] = 0.0
                command['z'] = 350.0
                rospy.logerr_throttle(2, '控制循环超时，水平摇杆居中并请求缓降')
            msg.x, msg.y, msg.r = (command[k] for k in ('x', 'y', 'r'))
            msg.z = command['z']
            pub.publish(msg)
            stop_sender.wait(0.05)

    def on_interrupt(signum, frame):
        command['abort'] = True
        print('收到停止请求，将进入降落处理……', flush=True)

    signal.signal(signal.SIGINT, on_interrupt)
    signal.signal(signal.SIGTERM, on_interrupt)
    thread = threading.Thread(target=sender, daemon=True)
    record = None
    record_log = csv_file = None
    arm_attempted = False
    base = None
    landed = False
    success = False
    started_sender = False
    tag = time.strftime('%Y%m%d_%H%M%S')
    prefix = Path('/home/noetic') / (('posctl_motion_' if args.motion else 'posctl_first_hover_') + tag)
    bag_path = str(prefix) + '.bag'

    try:
        if rospy.get_param('/use_sim_time', False) is not True:
            raise RuntimeError('本脚本仅用于启用/use_sim_time的Gazebo仿真')
        master = rosgraph.Master(rospy.get_name())
        publishers, _, services = master.getSystemState()
        if '/gazebo/get_world_properties' not in dict(services):
            raise RuntimeError('未发现Gazebo服务，停止测试')
        others = [n for n in dict(publishers).get('/mavros/manual_control/send', [])
                  if n != rospy.get_name()]
        if others:
            raise RuntimeError('请先停止终端⑤的旧摇杆程序：' + ', '.join(others))

        thread.start()
        started_sender = True
        print('接收地面数据，最多等待20秒……', flush=True)
        deadline = time.monotonic() + 20
        last_error = ''
        while time.monotonic() < deadline:
            if command['abort']:
                raise RuntimeError('用户取消测试')
            try:
                validate(snapshot(), time.monotonic(), rospy.Time.now().to_sec())
                break
            except RuntimeError as error:
                last_error = str(error)
                time.sleep(0.1)
        else:
            raise RuntimeError(last_error)
        d = snapshot()
        if d['state'][0].armed or d['land'][0].landed_state != 1:
            raise RuntimeError('开始测试前必须已经着地、上锁')
        print('等待MAVROS建立摇杆连接，最多10秒……', flush=True)
        deadline = time.monotonic() + 10
        while pub.get_num_connections() == 0:
            if command['abort']:
                raise RuntimeError('用户取消测试')
            if time.monotonic() > deadline:
                raise RuntimeError('MAVROS摇杆订阅连接未建立，请检查rostopic info /mavros/manual_control/send')
            time.sleep(0.1)
        validate(snapshot(), time.monotonic(), rospy.Time.now().to_sec())
        d = snapshot()
        if d['state'][0].armed or d['land'][0].landed_state != 1:
            raise RuntimeError('等待连接期间地面上锁状态发生变化')

        for service in ('/mavros/param/get', '/mavros/param/set',
                        '/mavros/set_mode', '/mavros/cmd/arming'):
            rospy.wait_for_service(service, timeout=10)
        get_param = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        set_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        expected = {'EKF2_EV_CTRL': 3, 'EKF2_GPS_CTRL': 0, 'SYS_HAS_GPS': 0,
                    'EKF2_HGT_REF': 3, 'EKF2_BARO_CTRL': 0, 'EKF2_RNG_CTRL': 0,
                    'COM_RC_IN_MODE': 1, 'COM_ARM_WO_GPS': 1}
        before = {}
        for name, value in expected.items():
            result = get_param(param_id=name)
            if not result.success or result.value.integer != value:
                raise RuntimeError(name + ' 与已验证配置不一致')
            before[name] = result.value.integer
        # 无GPS测试中，失去操纵输入或位置时请求降落，保留其他正常检查。
        for name in ('NAV_RCL_ACT', 'COM_POSCTL_NAVL'):
            result = get_param(param_id=name)
            if not result.success:
                raise RuntimeError('无法读取 ' + name)
            before[name] = result.value.integer
        Path(str(prefix) + '_params_before.json').write_text(
            json.dumps(before, indent=2), encoding='utf-8')
        for name, value in (('NAV_RCL_ACT', 3), ('COM_POSCTL_NAVL', 1)):
            result = set_param(param_id=name, value=ParamValue(integer=value, real=0.0))
            if not result.success or result.value.integer != value:
                raise RuntimeError('无法设置 ' + name)
            print('%s=%d：失去相应输入时降落' % (name, value), flush=True)

        topics = ['/clock', '/mavros/state', '/mavros/extended_state',
                  '/mavros/manual_control/send', '/mavros/local_position/pose',
                  '/mavros/local_position/velocity_local', '/mavros/vision_pose/pose',
                  '/vins/imu_gazebo', '/vins_estimator/odometry',
                  '/vins_estimator/imu_propagate', '/gazebo/model_states',
                  '/mavros/statustext/recv']
        record_log = open(str(prefix) + '_rosbag.log', 'w')
        record = subprocess.Popen(['rosbag', 'record', '__name:=posctl_first_hover_record',
                                   '-O', bag_path] + topics,
                                  stdout=record_log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        deadline = time.monotonic() + 10
        while not Path(bag_path + '.active').exists():
            if record.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('rosbag没有成功启动，请查看本次_rosbag.log')
            time.sleep(0.1)
        csv_file = open(str(prefix) + '.csv', 'w', newline='')
        writer = csv.writer(csv_file)
        writer.writerow(['phase', 'sim_time', 'px4_stamp', 'vins_stamp',
                         'px4_dz_m', 'vins_dz_m', 'px4_dxy_m', 'vision_age_s',
                         'mode', 'armed', 'landed_state', 'manual_z_raw',
                         'manual_x_raw', 'manual_y_raw', 'px4_dx_m', 'px4_dy_m'])

        if not mode(base_mode=0, custom_mode='POSCTL').mode_sent:
            raise RuntimeError('POSCTL切换指令未发送')
        print('最低油门接入，等待3秒稳定……', flush=True)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            validate(snapshot(), time.monotonic(), rospy.Time.now().to_sec())
            if command['abort']:
                raise RuntimeError('用户取消测试')
            time.sleep(0.1)
        d = snapshot()
        if d['state'][0].mode != 'POSCTL' or d['state'][0].armed or d['land'][0].landed_state != 1:
            raise RuntimeError('地面POSCTL未确认，停止测试')
        base = {'pose': xyz(d['pose'][0]), 'vins': xyz(d['vins'][0], True)}
        print('地面基准：PX4 z=%.3fm，VINS z=%.3fm' %
              (base['pose'][2], base['vins'][2]), flush=True)

        def report(phase):
            d = snapshot()
            if not all(k in d for k in ('state', 'land', 'pose', 'vins', 'vision')):
                return
            now = rospy.Time.now().to_sec()
            p, v = xyz(d['pose'][0]), xyz(d['vins'][0], True)
            dz = p[2]-base['pose'][2]
            vz = v[2]-base['vins'][2]
            drift = math.hypot(p[0]-base['pose'][0], p[1]-base['pose'][1])
            age = now-d['vision'][0].header.stamp.to_sec()
            state, land = d['state'][0], d['land'][0]
            writer.writerow([phase, now, d['pose'][0].header.stamp.to_sec(),
                             d['vins'][0].header.stamp.to_sec(), dz, vz, drift,
                             age, state.mode, state.armed, land.landed_state, command['z'],
                             command['x'], command['y'],
                             p[0]-base['pose'][0], p[1]-base['pose'][1]])
            csv_file.flush()
            print('%s PX4上升=%.3fm VINS上升=%.3fm 水平位移=%.3fm '
                  '视觉年龄=%.3fs mode=%s armed=%s' %
                  (phase, dz, vz, drift, age, state.mode, state.armed), flush=True)

        arm_attempted = True
        if not arming(value=True).success:
            raise RuntimeError('正常解锁被拒绝，没有使用强制解锁')
        deadline = time.monotonic() + 3
        while not snapshot()['state'][0].armed:
            if time.monotonic() > deadline:
                raise RuntimeError('解锁状态未确认')
            time.sleep(0.1)
        print('已正常解锁；目标相对高度约1米，随后悬停%g个仿真秒' % args.hover, flush=True)
        command['lease'] = time.monotonic()
        command['z'] = 750.0
        deadline = time.monotonic() + 25
        next_report = rospy.Time.now().to_sec()
        while True:
            d = snapshot()
            now = rospy.Time.now().to_sec()
            command['lease'] = time.monotonic()
            validate(d, time.monotonic(), now, base, flight=True)
            if command['abort'] or time.monotonic() > deadline:
                raise RuntimeError('起飞阶段停止请求或超时')
            if now >= next_report:
                report('起飞')
                next_report = now + 1
            if xyz(d['pose'][0])[2]-base['pose'][2] >= 1.0:
                break
            time.sleep(0.05)

        command['z'] = 500.0
        print('油门居中：先稳定2秒，再计时悬停%g个仿真秒' % args.hover, flush=True)
        start = rospy.Time.now().to_sec()
        deadline = time.monotonic() + 2*(args.hover+2) + 15
        next_report = start
        while rospy.Time.now().to_sec()-start < args.hover+2:
            now = rospy.Time.now().to_sec()
            validate(snapshot(), time.monotonic(), now, base, flight=True)
            if command['abort'] or time.monotonic() > deadline:
                raise RuntimeError('悬停阶段停止请求或仿真时间停滞')
            if now >= next_report:
                report('稳定' if now-start < 2 else '悬停')
                next_report = now + 1
            time.sleep(0.05)
        if args.motion:
            def timed_phase(phase, duration, x=0.0, y=0.0, z=500.0):
                command.update(x=x, y=y, r=0.0, z=z, lease=time.monotonic())
                start = rospy.Time.now().to_sec()
                deadline = time.monotonic() + 3*duration + 10
                next_report = start
                while rospy.Time.now().to_sec()-start < duration:
                    now = rospy.Time.now().to_sec()
                    command['lease'] = time.monotonic()
                    validate(snapshot(), time.monotonic(), now, base, flight=True)
                    if command['abort'] or time.monotonic() > deadline:
                        raise RuntimeError(phase + '停止请求或仿真时间停滞')
                    if now >= next_report:
                        report(phase)
                        next_report = now + 0.5
                    time.sleep(0.05)

            print('开始小幅移动：方向相对机头；每次输入1.5秒，随后松杆保持5秒。', flush=True)
            for name, x, y in (('前进', 200.0, 0.0), ('后退', -200.0, 0.0),
                               ('左移', 0.0, -200.0), ('右移', 0.0, 200.0)):
                timed_phase(name, 1.5, x=x, y=y)
                timed_phase(name+'松杆保持', 5.0)

            height_before = xyz(snapshot()['pose'][0])[2]-base['pose'][2]

            def change_height(phase, target, z):
                command.update(x=0.0, y=0.0, r=0.0, z=z, lease=time.monotonic())
                deadline = time.monotonic() + 15
                next_report = 0.0
                while True:
                    d = snapshot()
                    now = rospy.Time.now().to_sec()
                    command['lease'] = time.monotonic()
                    validate(d, time.monotonic(), now, base, flight=True)
                    if command['abort'] or time.monotonic() > deadline:
                        raise RuntimeError(phase+'停止请求或高度调整超时')
                    height = xyz(d['pose'][0])[2]-base['pose'][2]
                    if now >= next_report:
                        report(phase)
                        next_report = now + 0.5
                    if (z > 500 and height >= target) or (z < 500 and height <= target):
                        break
                    time.sleep(0.05)
                command['z'] = 500.0

            change_height('升高', height_before+0.20, 600.0)
            timed_phase('升高后松杆保持', 5.0)
            change_height('降低', height_before, 400.0)
            timed_phase('降低后松杆保持', 5.0)
        success = True

    except Exception as error:
        print('测试阶段停止：' + str(error), flush=True)
    finally:
        command['x'] = command['y'] = command['r'] = 0.0
        if arm_attempted:
            print('开始降落处理……', flush=True)
            command['z'] = 350.0
            deadline = time.monotonic() + 45
            next_report = 0
            fallback_sent = False
            near_ground_since = None
            finish_low_throttle = False
            while time.monotonic() < deadline and not rospy.is_shutdown():
                d = snapshot()
                now_wall = time.monotonic()
                s = d.get('state')
                l = d.get('land')
                fresh = s is not None and l is not None and now_wall-s[1] < 3 and now_wall-l[1] < 3
                if fresh and s[0].mode != 'POSCTL' and s[0].armed and not fallback_sent:
                    try:
                        print('请求AUTO.LAND：', mode(base_mode=0, custom_mode='AUTO.LAND').mode_sent,
                              flush=True)
                    except Exception as error:
                        print('降落模式请求异常：', error, flush=True)
                    fallback_sent = True
                now_sim = rospy.Time.now().to_sec()
                if fresh and not finish_low_throttle:
                    if near_ground_for_low_throttle(d, base, now_wall, now_sim):
                        if near_ground_since is None:
                            near_ground_since = now_sim
                        if now_sim-near_ground_since >= 0.4:
                            finish_low_throttle = True
                            print('已接近地面，油门降到最低，等待真实着地标志和自动上锁……', flush=True)
                    else:
                        near_ground_since = None
                if fresh and l[0].landed_state == 1:
                    command['z'] = 0.0
                    if not s[0].armed:
                        landed = True
                        break
                elif fresh:
                    command['z'] = 0.0 if finish_low_throttle else 350.0
                try:
                    validate(d, now_wall, rospy.Time.now().to_sec())
                except RuntimeError as error:
                    success = False
                    if now_wall >= next_report:
                        print('降落期间数据检查：', error, flush=True)
                if now_wall >= next_report and base is not None and csv_file is not None:
                    report('降落')
                    next_report = now_wall + 1
                time.sleep(0.1)
            if landed:
                print('已着地且上锁，继续记录10个仿真秒……', flush=True)
                start = rospy.Time.now().to_sec()
                deadline = time.monotonic() + 25
                next_report = start
                while rospy.Time.now().to_sec()-start < 10 and time.monotonic() < deadline:
                    now = rospy.Time.now().to_sec()
                    try:
                        validate(snapshot(), time.monotonic(), now)
                    except RuntimeError as error:
                        success = False
                        if now >= next_report:
                            print('着地后数据检查：', error, flush=True)
                    if now >= next_report and base is not None and csv_file is not None:
                        report('着地后')
                        next_report = now + 1
                    time.sleep(0.1)
            else:
                success = False
                try:
                    print('未确认着地上锁；请求AUTO.LAND：',
                          mode(base_mode=0, custom_mode='AUTO.LAND').mode_sent, flush=True)
                except Exception as error:
                    print('降落请求异常：', error, flush=True)
                print('请保持仿真、VINS、视觉转发运行，并发送飞控状态', flush=True)

        if started_sender:
            stop_sender.set()
            thread.join(timeout=1)
        if record is not None:
            try:
                if record.poll() is None:
                    os.killpg(record.pid, signal.SIGINT)
                record.wait(timeout=15)
            except Exception as error:
                print('记录关闭异常：', error, flush=True)
            deadline = time.monotonic() + 10
            while Path(bag_path + '.active').exists() and time.monotonic() < deadline:
                time.sleep(0.1)
            print('记录文件：' + bag_path, flush=True)
            if Path(bag_path + '.active').exists():
                success = False
                print('仍存在.bag.active，请发送记录关闭信息', flush=True)
            elif Path(bag_path).exists():
                print('记录文件已关闭', flush=True)
            else:
                success = False
                print('未找到正式bag，请发送记录关闭信息', flush=True)
        if csv_file is not None:
            csv_file.close()
        if record_log is not None:
            record_log.close()
        print('飞行过程执行完成，需分析记录确认悬停与落地表现' if success and landed
              else '本次测试未完整通过，请发送输出', flush=True)
        rospy.signal_shutdown('测试脚本结束')


if __name__ == '__main__':
    main()
