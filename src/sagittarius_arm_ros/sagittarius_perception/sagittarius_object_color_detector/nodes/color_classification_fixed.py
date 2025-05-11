import math
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import actionlib
from sagittarius_object_color_detector.msg import SGRCtrlAction, SGRCtrlGoal, SGRCtrlResult
from sensor_msgs.msg import Image
import tf.transformations as tf_transformations
import deepseek
from ultralytics import YOLO
import os

model_path = os.path.expanduser('~/yolo11s.pt')
model = YOLO(model_path)

# 用户需求输入接口
user_input = input("请输入您的需求或感受：")

obj_list = []
current_target = {
    'Found': False,
    'x': 66,
    'y': 66
}

def calculate_priority(obj_info):
    """计算物体优先级（备用方案）"""
    x_norm = obj_info['x'] / 640
    y_norm = obj_info['y'] / 480
    accessibility = 1 / (1 + math.sqrt((x_norm - 0.5)**2 + (y_norm - 0.5)**2))
    
    safety_map = {
        'food': 0.9, 'knife': 0.7, 'scissors': 0.7,
        'medicine': 0.8, 'bottle': 0.85, 'cup': 0.9
    }
    safety = safety_map.get(obj_info['class'].lower(), 1.0)
    
    return obj_info['match_score'] * 0.6 + accessibility * 0.3 + safety * 0.1

def image_callback(data):
    global current_target, obj_list
    try:
        cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    cv2.imshow("source", cv_image)
    cv2.waitKey(1)

    if current_target['Found']:
        return
    
    try:
        results = model.predict(cv_image)
    except Exception as e:
        print(f"YOLO预测异常：{str(e)}")
        return
    
    annotated_frame = results[0].plot()
    cv2.imshow("YOLO Detection", annotated_frame)
    obj_list = {}
    obj_names = ''
    for result in results:
        boxes = result.boxes
        names = model.names
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            cls_id = int(box.cls[0])
            obj_name = names[cls_id]
            obj_names += str(obj_name) + ' , '
            obj_list[obj_name] = {
                'x': float((x1 + x2) / 2),
                'y': float((y1 + y2) / 2),
            }

    return_obj = deepseek.deepseek_chat(user_input, obj_names)

    if return_obj:
                
        current_target = {
            'Found': True,
            'x': obj_list[return_obj]['x'],
            'y': obj_list[return_obj]['y'],
            'object_class': return_obj  
        }


def main():
    rospy.init_node('color_classification_node', anonymous=False)
    client = actionlib.SimpleActionClient(
        rospy.get_param("~arm_name", "sgr532") + '/' + 'sgr_ctrl', SGRCtrlAction)
    client.wait_for_server()
    r1 = rospy.Rate(25)
    goal_search = SGRCtrlGoal()
    goal_search.action_type = goal_search.ACTION_TYPE_XYZ_RPY
    goal_search.grasp_type = goal_search.GRASP_OPEN
    goal_search.pos_x = 0.2
    goal_search.pos_z = 0.15
    goal_search.pos_pitch = 1.57

    b1 = 0.331634612
    b2 = 0.0854435319
    k1 = -0.000303136327
    k2 = -0.000283300835

    destination = {'x': 0.26, 'y': 0.18}
    goal_pick = SGRCtrlGoal()
    goal_pick.grasp_type = goal_pick.GRASP_OPEN
    goal_pick.action_type = goal_pick.ACTION_TYPE_PICK_XYZ
    goal_pick.pos_z = 0.02
    goal_pick.pos_pitch = 1.57
    goal_put = SGRCtrlGoal()
    goal_put.action_type = goal_pick.ACTION_TYPE_PUT_XYZ
    goal_put.pos_z = 0.2
    
    client.send_goal_and_wait(goal_search, rospy.Duration.from_sec(30))
    
    sub1 = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    
    try:
        while not rospy.is_shutdown():
            if not current_target['Found']:
                r1.sleep()
                continue
            goal_pick.pos_x = k1 * current_target['y'] + b1
            goal_pick.pos_y = k2 * current_target['x'] + b2
            print(f"当前目标坐标: X:{current_target['x']:.1f} Y:{current_target['y']:.1f}")
            client.send_goal_and_wait(goal_pick, rospy.Duration.from_sec(30))
            ret = client.get_result()
            
            if ret.result == SGRCtrlResult.PLAN_NOT_FOUND:
                rospy.logwarn("路径规划失败")
            elif ret.result == SGRCtrlResult.GRASP_FAILD:
                rospy.logwarn("抓取失败")
            else:
                goal_put.pos_x = destination['x']
                goal_put.pos_y = destination['y']
                client.send_goal_and_wait(goal_put, rospy.Duration.from_sec(30))
                
                try:
                    companion_prompt = (
                        f"作为家庭陪伴机器人，我刚刚帮用户抓取了{current_target['object_class']}。"
                        f"用户最初的需求是：'{user_input}'。"
                        "请生成一句充满关怀的简短问候（20字以内），直接输出这句话。"
                    )
                    
                    companion_response = deepseek.deepseek_chat(
                        companion_prompt,
                        current_target['object_class'],
                        model="deepseek-r1:14b",
                        url="http://localhost:11434/api/generate"
                    )
                    
                    print(f"[机器人贴心提示] {companion_response}")
                except Exception as e:
                    print(f"陪伴功能异常: {str(e)}")
            client.send_goal_and_wait(goal_search, rospy.Duration.from_sec(30))
            r1.sleep()
            current_target['Found'] = False
    except rospy.ROSInterruptException:
        rospy.loginfo("程序终止")

if __name__ == '__main__':
    main()