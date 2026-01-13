import os
import math
import time
import numpy as np
import sim
import simConst
from dotenv import load_dotenv

from coppeliasim_client import Coppeliasim_client


load_dotenv()


class HT_matrix:
    def __init__(self):
        self.dh_d     = self.parse_dh_param(os.getenv("DH_D"))
        self.dh_a     = self.parse_dh_param(os.getenv("DH_A"))
        self.dh_alpha = self.parse_dh_param(os.getenv("DH_ALPHA"))
        
        self.T_base_to_world = np.array([
            [ 0,  1,  0,  0],  # Base의 Y가 World의 X가 됨
            [-1,  0,  0,  0],  # Base의 X가 World의 -Y가 됨
            [ 0,  0,  1,  0],  # Z는 그대로
            [ 0,  0,  0,  1]
        ])

    def parse_dh_param(self, env_str):
        if not env_str: return [0]*6
        safe_env_str = env_str.replace('pi', str(math.pi))
        try:
            return [float(eval(item)) for item in safe_env_str.split(',')]
        except Exception as e:
            print(f"DH Parameter Parsing Error: {e}")
            return [0]*6

    def get_dh_Transform(self, theta, d, a, alpha):
        return np.array([
            [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0,                math.sin(alpha),                  math.cos(alpha),                 d],
            [0,                0,                                0,                               1]
        ])

    def ForwardKinematics(self, joint_angles):
        # 각도 보정
        theta = [
            joint_angles[0],
            joint_angles[1] - math.pi/2, 
            joint_angles[2],
            joint_angles[3] - math.pi/2,
            joint_angles[4],
            joint_angles[5]
        ]

        T_final = np.eye(4)
        for i in range(6):
            T_i = self.get_dh_Transform(theta[i], self.dh_d[i], self.dh_a[i], self.dh_alpha[i])
            T_final = np.dot(T_final, T_i)
            
        # World 좌표계 변환 적용
        self.T06 = np.dot(self.T_base_to_world, T_final)
        return self.T06
    
    def get_position(self):
        return self.T06[:3, 3]

def main():
    # --- 1. 초기 설정 (Setup) ---
    client = Coppeliasim_client()
    client.check_connection()
    client.initialize_handles() 
    
    kinematics = HT_matrix()
    
    # --- 2. 시뮬레이션 시작 ---
    sim.simxStartSimulation(client.client_id, simConst.simx_opmode_blocking)
    client.start_streaming() # 데이터 전송 요청

    print(">> Simulation Started")
    time.sleep(1) # 데이터가 안정적으로 들어올 때까지 잠시 대기

    try:
        while True:
            # --- 3. 데이터 획득 (Synchronized) ---
	        # 관절 각도와 실제 TCP 위치를 동시에 가져옴
            angles, sim_tcp_world = client.get_data_synchronized()

            # 데이터가 아직 안 들어왔으면 건너뜀 (Safety)
            if len(angles) != 6: 
                continue 

            # --- 4. 정기구학 계산 ---
            kinematics.ForwardKinematics(angles)
            my_tcp_world = kinematics.get_position() # 내가 계산한 좌표
            
            # --- 5. 검증 및 오차 계산 ---
	        # 시뮬레이터 값 vs 내 계산 값 거리 차이
            error = np.linalg.norm(my_tcp_world - np.array(sim_tcp_world))

            # 결과 출력
            print("-" * 50)
            print(f"My FK (World) : X={my_tcp_world[0]:.4f}, Y={my_tcp_world[1]:.4f}, Z={my_tcp_world[2]:.4f}")
            print(f"Sim TCP(World): X={sim_tcp_world[0]:.4f}, Y={sim_tcp_world[1]:.4f}, Z={sim_tcp_world[2]:.4f}")
            print(f"Error         : {error:.5f} m")

            # 루프 속도 조절
            time.sleep(0.5) 

    except KeyboardInterrupt:
        print("\n>> 종료 요청 받음.")
    
    finally:
        # --- 6. 안전 종료 ---
        sim.simxStopSimulation(client.client_id, simConst.simx_opmode_blocking)
        sim.simxFinish(client.client_id)
        print(">> Remote API 연결 종료")

if __name__ == "__main__":
    main()