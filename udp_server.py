import socket
import threading
import time
import datetime

# === 配置 ===
PORT_FOR_CLIENT_A = 4000  # 对应 ESP32 / 客户端A
PORT_FOR_CLIENT_B = 4001  # 对应 电脑 / 客户端B

# 全局变量 (加锁是为了防止多线程读写冲突，虽然Python GIL有保护，但加锁更规范)
client_a_addr = None
client_b_addr = None
lock = threading.Lock()

# 创建 socket
sock_4000 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_4000.bind(('0.0.0.0', PORT_FOR_CLIENT_A))

sock_4001 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_4001.bind(('0.0.0.0', PORT_FOR_CLIENT_B))

def get_time():
    return datetime.datetime.now().strftime("%H:%M:%S")

def handle_port_4000():
    """
    监听 4000 (Client A/ESP32)
    收到数据 -> 更新 A 的地址 -> 转发给 B
    """
    global client_a_addr
    print(f"[{get_time()}] Listening on UDP {PORT_FOR_CLIENT_A} for Client A...")
    
    while True:
        try:
            data, addr = sock_4000.recvfrom(4096)
            
            with lock:
                # === 关键修改：检测地址变化 ===
                # 如果是新地址，或者地址变了（重连了），立即更新
                if client_a_addr != addr:
                    print(f"[{get_time()}] [Event] Client A connected/changed: {client_a_addr} -> {addr}")
                    client_a_addr = addr
                
                # 获取当前的 B 地址
                target = client_b_addr

            # 如果 B 还在（或者曾经来过），就转发
            if target:
                # 用 4001 端口发给 B
                sock_4001.sendto(data, target)
            else:
                # B 还没上线，数据只能丢弃，或者打印一下等待中
                # print(f"[{get_time()}] Received from A, but waiting for B...")
                pass

        except Exception as e:
            print(f"Error 4000: {e}")

def handle_port_4001():
    """
    监听 4001 (Client B/PC)
    收到数据 -> 更新 B 的地址 -> 转发给 A
    """
    global client_b_addr
    print(f"[{get_time()}] Listening on UDP {PORT_FOR_CLIENT_B} for Client B...")
    
    while True:
        try:
            data, addr = sock_4001.recvfrom(4096)
            
            with lock:
                # === 关键修改：检测地址变化 ===
                if client_b_addr != addr:
                    print(f"[{get_time()}] [Event] Client B connected/changed: {client_b_addr} -> {addr}")
                    client_b_addr = addr
                
                target = client_a_addr

            if target:
                # 用 4000 端口发给 A
                sock_4000.sendto(data, target)
            else:
                # print(f"[{get_time()}] Received from B, but waiting for A...")
                pass

        except Exception as e:
            print(f"Error 4001: {e}")

if __name__ == "__main__":
    t1 = threading.Thread(target=handle_port_4000, daemon=True)
    t2 = threading.Thread(target=handle_port_4001, daemon=True)

    t1.start()
    t2.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Server stopping...")
