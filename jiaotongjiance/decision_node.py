#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import message_filters
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist


class DecisionCenter:
    def __init__(self):
        rospy.init_node('decision_node')

        # 类别映射表 (对应你提供的 class_names 列表索引)
        self.sign_map = {
            2: "限速30", 5: "限速60", 42: "慢行",
            52: "停车让行", 53: "禁止通行",
            22: "允许左转", 24: "允许右转", 31: "允许掉头"
        }

        # 状态变量
        self.current_sign = None
        self.stop_timer = 0
        self.is_stopping = False

        # 订阅与发布
        lane_sub = message_filters.Subscriber('/lane_offset', Float32)
        sign_sub = message_filters.Subscriber('/traffic_sign', Int32)
        ts = message_filters.ApproximateTimeSynchronizer([lane_sub, sign_sub], 10, 0.1)
        ts.registerCallback(self.control_loop)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # 控制参数
        self.kp = 1.8  # 转向增益
        self.base_speed = 0.5  # 默认巡航速度
        self.lane_limit = 0.1  # 10厘米偏移限制

    def control_loop(self, lane_msg, sign_msg):
        offset = lane_msg.data
        sign_id = sign_msg.data
        self.current_sign = self.sign_map.get(sign_id, "无")

        twist = Twist()
        speed = self.base_speed
        turn_extra = 0.0  # 标志位带来的额外转向补偿

        # ===== 1. 标志位控制细节逻辑 =====

        # --- 限速与慢行逻辑 ---
        if self.current_sign == "限速30":
            speed = 0.3
        elif self.current_sign == "限速60":
            speed = 0.6
        elif self.current_sign == "慢行":
            speed = 0.2

        # --- 停车逻辑 (带计时器) ---
        elif self.current_sign in ["停车让行", "禁止通行"]:
            speed = 0.0
            rospy.loginfo(f"检测到 {self.current_sign}，执行停车")

        # --- 转向引导逻辑 ---
        # 这里的 turn_extra 会叠加在车道线巡线逻辑上
        elif self.current_sign == "允许左转":
            turn_extra = 0.5  # 给一个向左的预置量，辅助巡线
        elif self.current_sign == "允许右转":
            turn_extra = -0.5  # 给一个向右的预置量

        # --- 掉头逻辑 ---
        elif self.current_sign == "允许掉头":
            speed = 0.2
            turn_extra = 1.2  # 较大的转向值执行掉头

        # ===== 2. 车道保持 (巡线) 逻辑 =====
        # 基础转向由 PID 计算
        steering = (0.0 - offset) * self.kp

        # 叠加标志位的动作
        final_steering = steering + turn_extra

        # ===== 3. 安全约束与发布 =====
        if abs(offset) > self.lane_limit:
            rospy.logwarn(f"警报：偏移量 {abs(offset) * 100:.1f}cm 超过10cm限制！")
            # 如果偏移过大，可以采取强制减速措施
            speed *= 0.5

        twist.linear.x = speed
        twist.angular.z = final_steering
        self.cmd_pub.publish(twist)


if __name__ == '__main__':
    try:
        DecisionCenter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass