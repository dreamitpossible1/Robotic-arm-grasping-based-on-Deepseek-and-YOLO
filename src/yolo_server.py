from openai import OpenAI
from prompt_toolkit import prompt
import time
import socket

# 初始化客户端
client = OpenAI(
    api_key="sk-**********************",
    base_url="https://api.deepseek.com"
)

# 物品中英文对照表
OBJECT_TRANSLATIONS = {
    # 常见物品
    "杯子": "cup",
    "水杯": "cup",
    "茶杯": "cup",
    "碗": "bowl",
    "盒子": "box",
    "鼠标": "mouse",
    "键盘": "keyboard",
    "手机": "phone",
    "笔": "pen",
    "铅笔": "pencil",
    "书": "book",
    "瓶子": "bottle",
    "水瓶": "bottle",
    "眼镜": "glasses",
    "钥匙": "key",
    "遥控器": "remote",
    "纸": "paper",
    "剪刀": "scissors",
    "尺子": "ruler",
    "包": "bag",
    "钱包": "wallet",
    "手表": "watch",
    "充电器": "charger",
    "耳机": "headphones",
    "相机": "camera",
    "餐盘": "plate",
    "叉子": "fork",
    "勺子": "spoon",
    "刀": "knife",
    # 反向映射（英文到中文）
    "cup": "杯子",
    "bowl": "碗",
    "box": "盒子",
    "mouse": "鼠标",
    "keyboard": "键盘",
    "phone": "手机",
    "pen": "笔",
    "pencil": "铅笔",
    "book": "书",
    "bottle": "瓶子",
    "glasses": "眼镜",
    "key": "钥匙",
    "remote": "遥控器",
    "paper": "纸",
    "scissors": "剪刀",
    "ruler": "尺子",
    "bag": "包",
    "wallet": "钱包",
    "watch": "手表",
    "charger": "充电器",
    "headphones": "耳机",
    "camera": "相机",
    "plate": "餐盘",
    "fork": "叉子",
    "spoon": "勺子",
    "knife": "刀"
}

# UDP 服务器设置
UDP_IP = "192.168.1.26"
UDP_PORT = 9090
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((UDP_IP, UDP_PORT))
print(f"UDP 服务器启动，等待客户端消息... ({UDP_IP}:{UDP_PORT})")
client_address = None

# 等待客户端连接
while client_address is None:
    print("等待客户端连接...")
    data, client_address = server_socket.recvfrom(1024)
    print(f"收到客户端 {client_address} 的消息：{data.decode()}")

# 初始化对话历史
messages = [
    {"role": "system", "content": """你是一个智能助手，负责帮助用户找寻和拿取物品。你需要：
1. 当用户要求拿取某个物品时，你要查看detected_objects.txt文件中的内容
2. 如果文件中包含用户要找的物品，回复表示已经找到并帮助拿取
3. 如果文件中没有用户要找的物品，礼貌地告知用户当前没有找到该物品
4. 回答要自然、友好，像一个真实的助手
5. 要告诉用户当前检测到的所有物品，使用中文名称

请用中文对话。"""}
]

def translate_to_english(chinese_word):
    """将中文物品名转换为英文"""
    return OBJECT_TRANSLATIONS.get(chinese_word)

def translate_to_chinese(english_word):
    """将英文物品名转换为中文"""
    return OBJECT_TRANSLATIONS.get(english_word, english_word)

def read_detected_objects():
    """读取当前检测到的物品列表并转换为中文"""
    try:
        with open("detected_objects.txt", "r") as f:
            objects = [line.strip() for line in f.readlines()]
            # 将检测到的英文物品名转换为中文
            return [translate_to_chinese(obj) for obj in objects]
    except FileNotFoundError:
        return []

def find_object_in_detected(chinese_name, detected_objects_english):
    """检查用户要找的物品是否在检测列表中"""
    if not chinese_name:
        return False
    english_name = translate_to_english(chinese_name)
    if not english_name:
        return False
    return english_name.lower() in [obj.lower() for obj in detected_objects_english]

while True:
    # 获取用户输入
    user_input = prompt("\n用户：")
    
    # 退出机制
    if user_input.lower() in ["退出", "exit"]:
        print("对话结束。")
        break
    
    # 读取当前检测到的物品（英文）
    try:
        with open("detected_objects.txt", "r") as f:
            current_objects_english = [line.strip() for line in f.readlines()]
    except FileNotFoundError:
        current_objects_english = []
    
    # 转换为中文显示
    current_objects_chinese = [translate_to_chinese(obj) for obj in current_objects_english]
    
    # 构建包含当前物品信息的用户消息
    context_message = f"""用户的请求是：{user_input}
当前检测到的物品有：{', '.join(current_objects_chinese) if current_objects_chinese else '没有检测到任何物品'}"""
    
    # 添加用户消息到历史
    messages.append({"role": "user", "content": context_message})
    
    try:
        # 获取模型响应
        response = client.chat.completions.create(
            model="deepseek-chat",
            messages=messages,
            stream=False
        )
        
        # 解析回复内容
        assistant_response = response.choices[0].message.content
        
        # 添加助手回复到历史
        messages.append({"role": "assistant", "content": assistant_response})
        
        # 打印回复
        print("\n助手：", assistant_response)
        
        # 检查用户请求的物品是否在检测列表中
        found_item = None
        for name in OBJECT_TRANSLATIONS:
            if name in user_input:
                found_item = name
                break
        if found_item and translate_to_english(found_item) in current_objects_english:
            # 向客户端发送英文物品名
            if client_address:
                english_name = translate_to_english(found_item)
                server_socket.sendto(english_name.encode(), client_address)
                print(f"已发送指令 '{english_name}' 给客户端 {client_address}")
        
    except Exception as e:
        print(f"发生错误：{str(e)}")
        break
