import socket
import time
from uarm.wrapper import SwiftAPI

# 创建UDP客户端
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ("192.168.1.26", 9090)  # 与服务端IP一致

# 主动发送消息给服务端，暴露自己地址
client_socket.sendto("Hello, UDP Server!".encode(), server_address)
print("等待服务端指令...")

try:
    while True:
        swift = SwiftAPI(port='/dev/ttyACM0')
        data, _ = client_socket.recvfrom(1024)
        msg = data.decode().strip()
        print(f"收到服务端消息: {msg}")
        if msg == "cup":
            print("执行：抓取杯子的动作！")
            swift.connect()
            time.sleep(2)

            ###############
            swift.set_position(x=200, y=-50, z=150, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=200, y=95, z=150, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=200, y=95, z=100, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=95, y=95, z=100, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=95, y=95, z=70, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_pump(on=True)
            time.sleep(1)

            swift.set_position(x=95, y=95, z=100, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=95, y=200, z=100, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=95, y=200, z=150, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=200, y=-50, z=150, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=200, y=-50, z=100, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_pump(on=False)
            time.sleep(1)

            swift.set_position(x=200, y=-50, z=150, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            # 断开连接
            swift.disconnect()
        elif msg == "bowl":
            print("执行：抓取盆子的动作！")
            swift.connect()
            time.sleep(2)

            ###############
            swift.set_position(x=200, y=-50, z=150, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=300, y=0, z=100, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=300, y=130, z=100, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=300, y=130, z=5, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_pump(on=True)
            time.sleep(1)

            swift.set_position(x=300, y=130, z=100, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=300, y=-50, z=100, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=200, y=-50, z=100, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=200, y=-50, z=20, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_pump(on=False)
            time.sleep(1)

            swift.set_position(x=200, y=-50, z=150, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)
            # 断开连接
            swift.disconnect()
        elif msg == "mouse":
            print("执行：抓取鼠标的动作！")
            swift.connect()
            time.sleep(2)

            ###############
            swift.set_position(x=200, y=-50, z=150, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=195, y=130, z=150, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=195, y=130, z=30, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_pump(on=True)
            time.sleep(1)

            swift.set_position(x=195, y=130, z=150, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=200, y=-50, z=150, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_position(x=200, y=-50, z=140, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            swift.set_pump(on=False)
            time.sleep(1)

            swift.set_position(x=200, y=-50, z=150, wait=True)
            print("到达地面:", swift.get_position())
            time.sleep(1)

            # 断开连接
            swift.disconnect()

        else:
            print("收到未知指令，忽略。")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("客户端退出。")
finally:
    client_socket.close()