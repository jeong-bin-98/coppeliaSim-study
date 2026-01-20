"""
Inverse Kinematics 시뮬레이션 서버
소켓을 통해 target_input.py에서 좌표를 받아 시뮬레이션에 적용
Path Interpolation으로 부드러운 타겟 이동
"""
import os
import math
import time
import threading
import socket
import numpy as np
import sim
import simConst
from dotenv import load_dotenv

from coppeliasim_client import Coppeliasim_client

load_dotenv()

# 소켓 설정
HOST = '127.0.0.1'
PORT = 5555

# 전역 변수
current_target = np.array([0.0, 0.0, 0.0])  # 현재 타겟 위치 (보간 중)
goal_target = np.array([0.0, 0.0, 0.0])     # 목표 타겟 위치 (사용자 입력)
running = True

# 보간 설정
INTERPOLATION_SPEED = 0.5  # m/s (초당 이동 거리)
LOOP_INTERVAL = 0.025      # 루프 주기 (초) - 40Hz (시뮬레이션 20Hz의 2배)

def lerp_target(current, goal, max_step):
    """
    Linear Interpolation: 현재 위치에서 목표로 max_step만큼 이동
    """
    direction = goal - current
    distance = np.linalg.norm(direction)
    
    if distance <= max_step:
        return goal.copy()  # 목표에 도달
    
    # 방향 벡터 정규화 후 max_step만큼 이동
    unit_dir = direction / distance
    return current + unit_dir * max_step

def socket_server_thread():
    """소켓 서버: 클라이언트로부터 좌표를 수신"""
    global goal_target, running
    
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    server.settimeout(1)
    
    print(f"[서버] 소켓 서버 시작 (포트: {PORT})")
    
    while running:
        try:
            conn, addr = server.accept()
            with conn:
                data = conn.recv(1024).decode().strip()
                if data:
                    coords = data.split()
                    if len(coords) == 3:
                        goal_target[0] = float(coords[0])
                        goal_target[1] = float(coords[1])
                        goal_target[2] = float(coords[2])
                        print(f"\n[수신] 새 목표 좌표: ({goal_target[0]:.4f}, {goal_target[1]:.4f}, {goal_target[2]:.4f})")
        except socket.timeout:
            continue
        except Exception as e:
            if running:
                print(f"[서버 오류] {e}")
    
    server.close()
    print("[서버] 소켓 서버 종료")

def main():
    global current_target, goal_target, running
    
    # --- 1. Remote API 연결 ---
    client = Coppeliasim_client()
    client.check_connection()

    # --- 2. 핸들 초기화 ---
    client.initialize_handles()
    
    # --- 3. 시뮬레이션 시작 ---
    sim.simxStartSimulation(client.client_id, simConst.simx_opmode_blocking)
    client.start_streaming()

    print(">> Simulation Started")  

    time.sleep(1)

    # --- 초기 TCP 위치로 타겟 좌표 설정 ---
    _, tcp_pos, _ = client.get_data_synchronized()
    current_target = np.array(tcp_pos)
    goal_target = np.array(tcp_pos)
    print(f"[초기화] 시작 위치: ({current_target[0]:.4f}, {current_target[1]:.4f}, {current_target[2]:.4f})")

    # --- 소켓 서버 쓰레드 시작 ---
    server_t = threading.Thread(target=socket_server_thread, daemon=True)
    server_t.start()

    print(f"\n[설정] 보간 속도: {INTERPOLATION_SPEED} m/s")
    print("[안내] 다른 터미널에서 'python target_input.py' 실행하여 좌표 입력\n")

    try:
        while running:
            # --- 4. Path Interpolation 적용 ---
            max_step = INTERPOLATION_SPEED * LOOP_INTERVAL
            current_target = lerp_target(current_target, goal_target, max_step)
            
            # --- 5. Target 위치 설정 (보간된 좌표 사용) ---
            client.set_target_position(current_target[0], current_target[1], current_target[2])
            
            # --- 6. 데이터 가져오기 ---
            angles, tcp_pos, target_pos = client.get_data_synchronized()
            
            # --- 7. 결과 출력 ---
            error = np.linalg.norm(np.array(target_pos) - np.array(tcp_pos))
            remaining = np.linalg.norm(goal_target - current_target)
            
            print("-" * 60)
            print(f"Goal Target   : ({goal_target[0]:.4f}, {goal_target[1]:.4f}, {goal_target[2]:.4f})")
            print(f"Current Target: ({current_target[0]:.4f}, {current_target[1]:.4f}, {current_target[2]:.4f})")
            print(f"TCP Position  : ({tcp_pos[0]:.4f}, {tcp_pos[1]:.4f}, {tcp_pos[2]:.4f})")
            print(f"Tracking Error: {error:.5f} m | Remaining: {remaining:.5f} m")
            print("-" * 60)

            time.sleep(LOOP_INTERVAL)
   
    except KeyboardInterrupt:
        print("\n>> 종료 요청 받음.")
        running = False
    
    finally:
        running = False
        sim.simxStopSimulation(client.client_id, simConst.simx_opmode_blocking)
        sim.simxFinish(client.client_id)
        print(">> Remote API 연결 종료")

if __name__ == "__main__":
    main()