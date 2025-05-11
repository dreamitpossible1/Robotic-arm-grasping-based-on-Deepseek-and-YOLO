import requests
import re
from PhotoCap import PhotoCap
import subprocess


def deepseek_chat(user_input, object_class, model="deepseek-r1:7b", url="http://localhost:11434/api/generate"):
    payload = {
        "model": model,
        "prompt": f"你是一个智能机械臂，我的需求'{user_input}'，拥有物体'{object_class}'，根据相关程度，直接输出一个相关度最高的所给的物体名字",
        "stream": False,
        "options": {
            "temperature": 0.1,
            "top_p": 0.9,
            "max_tokens": 600
        }
    }

    response = requests.post(url, json=payload)
    if not response.ok:
        raise RuntimeError(f"Request failed: {response.status_code} - {response.text}")

    full = response.json().get("response", "").strip()
       
    # Extract answer after </think>
    answer = full.split("</think>")[-1].strip()

    return answer
