#!/usr/bin/env python

from __future__ import print_function
import sys
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class TakePhoto(Node):
    def __init__(self):
        super().__init__('take_photo')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(1)
        self.rot = Twist()
        
        self.state = "FIND_BALL"
        self.ball_in_control = False
        self.is_resetting = False
        self.push_timer = None
        self.back_off_timer = None
        self.rotate_timer = None
        self.forward_timer = None  # เพิ่มตัวแปร forward_timer

        self.ball_cx = 0
        self.goal_cx = 0
        self.goal_width = 0
        self.obstacle_cx = 0
        self.obstacle_width = 0

        self.bridge = CvBridge()
        self.image_received = False

        self.ball_count = 0  # Initialize ball_count attribute

        self.robot_x = 0  # Initialize robot_x attribute
        self.robot_y = 0  # Initialize robot_y attribute

        img_topic = "/camera/image_raw"
        self.image_sub = self.create_subscription(Image, img_topic, self.callback, 10)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image
        
        if self.state == "BACK_OFF" or self.state == "ROTATE_180":
            self.detect_ball(cv_image)
        else:
            self.detect_ball(cv_image)
            self.detect_goal(cv_image)
            self.detect_obstacle(cv_image)

            if self.state == "FIND_BALL":
                if self.ball_in_control:
                    self.state = "PUSH_TO_GOAL"
                    print("Ball in control - switching to PUSH_TO_GOAL")
            elif self.state == "PUSH_TO_GOAL":
                if not self.ball_in_control:
                    self.state = "RECOVER_BALL"
                    print("Ball lost - switching to RECOVER_BALL")
            elif self.state == "RECOVER_BALL":
                if self.ball_in_control:
                    self.state = "PUSH_TO_GOAL"
                    print("Ball recovered - switching to PUSH_TO_GOAL")

        self.move_robot()

    def detect_ball(self, img):
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_frame = cv2.resize(hsv_frame, (640, 300))
        img = cv2.resize(img, (640, 300))

        low_H, low_S, low_V = 0, 100, 100
        high_H, high_S, high_V = 18, 255, 255

        mask_frame = cv2.inRange(hsv_frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
        contours, _ = cv2.findContours(mask_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        X, Y, W, H = 0, 0, 0, 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 30:
                x, y, w, h = cv2.boundingRect(contour)
                if w * h > W * H:
                    X, Y, W, H = x, y, w, h

        if W * H > 0:
            img = cv2.rectangle(img, (X, Y), (X + W, Y + H), (0, 0, 255), 2)
            self.ball_cx = X + (W / 2)
            self.ball_in_control = (Y + H > 280) and (W > 500)
        else:
            self.ball_cx = 0
            self.ball_in_control = False

        cv2.imshow("window", img)
        cv2.waitKey(3)

    def detect_goal(self, img):
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_frame = cv2.resize(hsv_frame, (640, 300))

        low_H, low_S, low_V = 35, 50, 50
        high_H, high_S, high_V = 85, 255, 255

        mask_frame = cv2.inRange(hsv_frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
        contours, _ = cv2.findContours(mask_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        X, Y, W, H = 0, 0, 0, 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:
                x, y, w, h = cv2.boundingRect(contour)
                if w * h > W * H and y < 150:
                    X, Y, W, H = x, y, w, h

        if W * H > 0:
            img = cv2.rectangle(img, (X, Y), (X + W, Y + H), (0, 255, 0), 2)
            self.goal_cx = X + (W / 2)
            self.goal_width = W
        else:
            self.goal_cx = 0
            self.goal_width = 0

        cv2.imshow("goal_mask", mask_frame)
        cv2.waitKey(3)

    def detect_obstacle(self, img):
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_frame = cv2.resize(hsv_frame, (640, 300))

        low_H, low_S, low_V = 0, 0, 50
        high_H, high_S, high_V = 180, 50, 200

        mask_frame = cv2.inRange(hsv_frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
        contours, _ = cv2.findContours(mask_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        X, Y, W, H = 0, 0, 0, 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:
                x, y, w, h = cv2.boundingRect(contour)
                if w * h > W * H and y > 150:
                    X, Y, W, H = x, y, w, h

        if W * H > 0:
            img = cv2.rectangle(img, (X, Y), (X + W, Y + H), (255, 0, 0), 2)
            self.obstacle_cx = X + (W / 2)
            self.obstacle_width = W
            print(f"Obstacle detected at cx={self.obstacle_cx}, width={self.obstacle_width}")
        else:
            self.obstacle_cx = 0
            self.obstacle_width = 0

        cv2.imshow("obstacle_mask", mask_frame)
        cv2.waitKey(3)

    def move_robot(self):
        text = "searching"
        if self.state == "FIND_BALL" or self.state == "RECOVER_BALL":
            if self.ball_cx == 0:
                text = "spinning to find ball"
                self.rot.angular.z = 0.15
                self.rot.linear.x = 0.0
            else:
                obj_x = self.ball_cx - 320
                if -40 <= obj_x <= 40:
                    text = "straight to ball"
                    self.rot.angular.z = 0.0
                    self.rot.linear.x = 0.15
                elif obj_x > 40:
                    text = "left to ball"
                    self.rot.angular.z = -0.15
                    self.rot.linear.x = 0.0
                elif obj_x < -40:
                    text = "right to ball"
                    self.rot.angular.z = 0.15
                    self.rot.linear.x = 0.0

        elif self.state == "PUSH_TO_GOAL":
            if self.goal_cx == 0:
                text = "spinning to find goal"
                self.rot.angular.z = -0.45
                self.rot.linear.x = 0.0
            else:
                is_goal_full_screen = self.goal_width > 600  # ปรับค่าได้ตามขนาดจอ
                
                obj_x = self.goal_cx - 320
                speed = max(0.1, min(0.3, self.goal_width / 3000.0))
                
                if is_goal_full_screen and self.ball_in_control and not self.push_timer:
                    print(f"goal_width: {self.goal_width}, ball_in_control: {self.ball_in_control} -> Starting final push")
                    text = "goal full screen - pushing ball to goal"
                    self.rot.angular.z = 0.0
                    self.rot.linear.x = 0.5
                    self.pub.publish(self.rot)  # **อัปเดตค่า**
                    self.push_timer = self.create_timer(5.0, self.finish_push)
                
                elif self.obstacle_cx != 0 and self.obstacle_width > 100:
                    obstacle_x = self.obstacle_cx - 320
                    if abs(obstacle_x) < 150:
                        if obstacle_x > 0:
                            text = "avoiding obstacle - turning left"
                            self.rot.angular.z = -0.3
                            self.rot.linear.x = 0.05
                        else:
                            text = "avoiding obstacle - turning right"
                            self.rot.angular.z = 0.3
                            self.rot.linear.x = 0.05
                    else:
                        text = "obstacle on side - moving straight"
                        self.rot.angular.z = 0.0
                        self.rot.linear.x = speed
                else:
                    if -40 <= obj_x <= 40:
                        text = "straight to goal"
                        self.rot.angular.z = 0.0
                        self.rot.linear.x = speed
                    elif obj_x > 40:
                        text = "left to goal"
                        self.rot.angular.z = -0.08
                        self.rot.linear.x = speed * 0.9
                    elif obj_x < -40:
                        text = "right to goal"
                        self.rot.angular.z = 0.08
                        self.rot.linear.x = 0.9 * speed

        elif self.state == "BACK_OFF":
            text = "backing off"
            self.rot.angular.z = 0.0
            self.rot.linear.x = -0.5

        elif self.state == "ROTATE_180":
            text = "rotating 180 to clear ball"
            self.rot.angular.z = 0.5
            self.rot.linear.x = 0.0

        elif self.state == "RETURN_HOME":
            if self.robot_x == 0 and self.robot_y == 0:
                text = "Arrived at home - stopping"
                self.stop()
                self.state = "STOP"
            else:
                text = f"Returning home... x: {self.robot_x}, y: {self.robot_y}"
                self.rot.angular.z = 0.0
                self.rot.linear.x = -0.2

        elif self.state == "STOP":
            text = "Stopped"
            self.stop()
            self.state = "RETURN_HOME"

        self.pub.publish(self.rot)
        print(f"State: {self.state} | {text}")
        
    def finish_back_off(self):
        self.stop()
        self.state = "ROTATE_180"
        self.rotate_timer = self.create_timer(4, self.finish_rotate)
        if self.back_off_timer:
            self.back_off_timer.destroy()
        self.back_off_timer = None
        print("Finished backing off - rotating 180 degrees")

    def finish_rotate(self):
        self.stop()
        self.state = "FIND_BALL"
        self.is_resetting = True
        self.reset_robot()
        if self.rotate_timer:
            self.rotate_timer.destroy()
        self.rotate_timer = None
        print("Finished rotating 180 - resetting to FIND_BALL as initial state")

    def finish_push(self):
        if self.push_timer:
            self.push_timer.destroy()
        self.push_timer = None
        
        self.rot.linear.x = 0.0
        self.rot.angular.z = 0.0
        self.pub.publish(self.rot)
        
        self.ball_count += 1
        print(f"Ball scored! Total: {self.ball_count}")

        if self.ball_count >= 3:
            print("Scored 3 balls - returning home")
            self.state = "RETURN_HOME"
        else:
            self.state = "BACK_OFF"
            self.back_off_timer = self.create_timer(6.0, self.finish_back_off)  # เพิ่มเวลาในการถอยหลัง

    def reset_robot(self):
        self.ball_in_control = False
        self.ball_cx = 0
        self.goal_cx = 0
        self.goal_width = 0
        self.obstacle_cx = 0
        self.obstacle_width = 0
        print("All variables reset to initial state")

    def stop(self):
        self.rot.angular.z = 0.0
        self.rot.linear.x = 0.0
        self.pub.publish(self.rot)
        print("Stopped")

    def destroy_node(self):
        self.stop()
        if self.back_off_timer:
            self.back_off_timer.destroy()
        if self.rotate_timer:
            self.rotate_timer.destroy()
        if self.push_timer:
            self.push_timer.destroy()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera = TakePhoto()

    try:
        rclpy.spin(camera)
    except KeyboardInterrupt:
        print("Shutting down gracefully...")
    finally:
        camera.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()