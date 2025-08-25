import socket
import time
from uarm.wrapper import SwiftAPI
import threading
import json
import math

INITIAL_X = 200
INITIAL_Y = -50
INITIAL_Z = 150

def linear_map(val, src_min, src_max, dst_min, dst_max):
    if src_max == src_min:
        return (dst_min + dst_max) / 2
    return dst_min + (val - src_min) * (dst_max - dst_min) / (src_max - src_min)

def map_delta_to_xy(dy, dx):
    x = 326 - 0.462021 * dx
    y = 141.3 - 0.353462 * dy
    print(f"map_delta_to_xy: dy={dy}, dx={dx} => x={x}, y={y}")
    return x, y


client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ("192.168.1.26", 9090)


client_socket.sendto("Hello, UDP Server!".encode(), server_address)


received_coordinates = None
waiting_for_confirmation = False
current_action = None

def udp_message_listener():
    global received_coordinates, waiting_for_confirmation, current_action
    
    while True:
        try:
            data, _ = client_socket.recvfrom(1024)
            msg = data.decode().strip()

            object_name = None
            coordinates = None
            
            if ": Center(" in msg:
                parts = msg.split(": Center(")
                if len(parts) == 2:
                    object_name = parts[0]
                    coord_str = parts[1].rstrip(")")
                    try:
                        x, y = map(int, coord_str.split(", "))
                        coordinates = (x, y)

                        if waiting_for_confirmation and current_action == object_name:
                            received_coordinates = coordinates
                            waiting_for_confirmation = False
                            print(f"✓ 收到服务端确认，{object_name}位置：({x}, {y})，继续执行抓取")
                        else:
                            received_coordinates = coordinates
                            execute_action(object_name)
                        
                    except ValueError:
                        print(f"坐标解析失败: {coord_str}")
                        object_name = parts[0] 
                        execute_action(object_name)
            else:
                object_name = msg
                print(f"收到{object_name}指令，开始抓取")
                execute_action(object_name)
                
        except Exception as e:
            print(f"接收消息时发生错误: {e}")
            break

def execute_action(object_name):
    global received_coordinates, waiting_for_confirmation, current_action
    
    swift = SwiftAPI(port='/dev/ttyACM0')
    
    if object_name == "cup":
        execute_cup_action(swift)
    elif object_name == "bowl":
        execute_bowl_action(swift)
    elif object_name == "mouse":
        execute_mouse_action(swift)
    else:
        print("收到未知指令，忽略。")

def execute_cup_action(swift):
    global received_coordinates
    
    try:
        swift.connect()
        time.sleep(2)
        swift.set_position(x=INITIAL_X, y=INITIAL_Y, z=INITIAL_Z, wait=True)
        time.sleep(1)

        if received_coordinates:
            px, py = received_coordinates
            x, y = map_delta_to_xy(px, py)
            
            swift.set_position(x=x, y=INITIAL_Y, z=INITIAL_Z, wait=True)
            time.sleep(1)
            swift.set_position(x=x, y=y, z=INITIAL_Z, wait=True)
            time.sleep(1)
            swift.set_position(x=x, y=y, z=70, wait=True)
            time.sleep(1)
        
        swift.set_pump(on=True)
        time.sleep(1)
        swift.set_position(x=95, y=95, z=100, wait=True)
        time.sleep(1)
        swift.set_position(x=95, y=200, z=100, wait=True)
        time.sleep(1)
        swift.set_position(x=95, y=200, z=150, wait=True)
        time.sleep(1)
        swift.set_position(x=200, y=-50, z=150, wait=True)
        time.sleep(1)
        swift.set_position(x=200, y=-50, z=100, wait=True)
        time.sleep(1)
        swift.set_pump(on=False)
        time.sleep(1)

        swift.set_position(x=200, y=-50, z=150, wait=True)
        time.sleep(1)
        swift.disconnect()
        
    except Exception as e:
        swift.disconnect()

def execute_bowl_action(swift):
    global received_coordinates
    try:
        swift.connect()
        time.sleep(2)
        swift.set_position(x=INITIAL_X, y=INITIAL_Y, z=INITIAL_Z, wait=True)
        time.sleep(1)

        if received_coordinates:
            px, py = received_coordinates
            x, y = map_delta_to_xy(px, py)
            swift.set_position(x=x, y=INITIAL_Y, z=INITIAL_Z, wait=True)
            time.sleep(1)
            swift.set_position(x=x, y=y, z=INITIAL_Z, wait=True)
            time.sleep(1)
            swift.set_position(x=x, y=y, z=5, wait=True)
        
        swift.set_pump(on=True)
        time.sleep(1)
        swift.set_position(x=300, y=130, z=100, wait=True)
        time.sleep(1)
        swift.set_position(x=300, y=-50, z=100, wait=True)
        time.sleep(1)
        swift.set_position(x=200, y=-50, z=100, wait=True)
        time.sleep(1)
        swift.set_position(x=200, y=-50, z=20, wait=True)
        time.sleep(1)
        swift.set_pump(on=False)
        time.sleep(1)
        swift.set_position(x=200, y=-50, z=150, wait=True)
        time.sleep(1)
        swift.disconnect()
        
    except Exception as e:
        swift.disconnect()

def execute_mouse_action(swift):
    global received_coordinates
    try:
        swift.connect()
        time.sleep(2)
        swift.set_position(x=INITIAL_X, y=INITIAL_Y, z=INITIAL_Z, wait=True)
        time.sleep(1)
        if received_coordinates:
            px, py = received_coordinates
            x, y = map_delta_to_xy(px, py)
            swift.set_position(x=x, y=INITIAL_Y, z=INITIAL_Z, wait=True)
            time.sleep(1)
            swift.set_position(x=x, y=y, z=INITIAL_Z, wait=True)
            time.sleep(1)
            swift.set_position(x=x, y=y, z=30, wait=True)
        
        swift.set_pump(on=True)
        time.sleep(1)
        swift.set_position(x=195, y=130, z=150, wait=True)
        time.sleep(1)
        swift.set_position(x=200, y=-50, z=150, wait=True)
        time.sleep(1)
        swift.set_position(x=200, y=-50, z=140, wait=True)
        time.sleep(1)
        swift.set_pump(on=False)
        time.sleep(1)
        swift.set_position(x=200, y=-50, z=150, wait=True)
        time.sleep(1)
        swift.disconnect()
        
    except Exception as e:
        swift.disconnect()

udp_thread = threading.Thread(target=udp_message_listener, daemon=True)
udp_thread.start()

try:
    while True:
        time.sleep(0.1) 
except KeyboardInterrupt:
    print("客户端退出。")
finally:
    client_socket.close()
