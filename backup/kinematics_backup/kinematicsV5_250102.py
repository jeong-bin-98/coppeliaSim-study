import os
import math
import time
import numpy as np
import sim
from dotenv import load_dotenv

# [모듈 import] 분리한 클래스를 가져옵니다
from coppeliasim_client import Coppeliasim_client

load_dotenv()

class HT_matrix:
    def __init__(self):
        self.dh_d     = self.parse_dh_param(os.getenv("DH_D"))
        self.dh_a     = self.parse_dh_param(os.getenv("DH_A"))
        self.dh_alpha = self.parse_dh_param(os.getenv("DH_ALPHA"))

        # [수정된 변환 행렬] Z축 +90도 회전
        # 데이터 근거: My_X -> Sim_Y, My_Y -> Sim_-X
        self.T_base_to_world = np.array([
            [ 0,  1,  0,  0],  # X_new = Y_old
            [-1,  0,  0,  0],  # Y_new = -X_old
            [ 0,  0,  1,  0],  # Z_new = Z_old
            [ 0,  0,  0,  1]
        ])

        # [팁 보정 행렬] 스케일링 4배 포함
        self.T_tip_correction = np.array([
            [ 0.,  4.,  0., -0.],   # Row 0: 0, 4, 0
            [-4.,  0., -0., -0.],   # Row 1: -4, 0, 0
            [ 0., -0.,  1., -0.],   # Row 2: Z축은 그대로 (1배)
            [ 0.,  0.,  0.,  1.]
        ])

        self.T_tip_correction = np.eye(4)
        # 쿼터니언 계산을 위해 회전만 있는(스케일링 없는) 행렬을 따로 저장할 변수
        self.T_pure_rotation = np.eye(4)

    def parse_dh_param(self, env_str):
        if not env_str: return [0]*6
        safe_str = env_str.replace('pi', str(math.pi))
        return [float(eval(item)) for item in safe_str.split(',')]

    def get_dh_Transform(self, th, d, a, alpha):
        return np.array([
            [math.cos(th), -math.sin(th)*math.cos(alpha),  math.sin(th)*math.sin(alpha), a*math.cos(th)],
            [math.sin(th),  math.cos(th)*math.cos(alpha), -math.cos(th)*math.sin(alpha), a*math.sin(th)],
            [0,             math.sin(alpha),               math.cos(alpha),              d],
            [0,             0,                             0,                            1]
        ])

    def ForwardKinematics(self, joint_angles):
        # [각도 보정] UR3 Standard -> CoppeliaSim
        th = [
            joint_angles[0],
            joint_angles[1] - math.pi/2,
            joint_angles[2],
            joint_angles[3] - math.pi/2,
            joint_angles[4],
            joint_angles[5]
        ]

        # 1. 로봇 내부 FK (Base 기준)
        T_robot = np.eye(4)
        for i in range(6):
            T_i = self.get_dh_Transform(th[i], self.dh_d[i], self.dh_a[i], self.dh_alpha[i])
            T_robot = np.dot(T_robot, T_i)
        
        # 2. World 좌표 변환 (Base -> World)
        # 이 단계까지는 스케일링이 없는 "순수 물리적 회전/위치"입니다.
        T_world_unscaled = np.dot(self.T_base_to_world, T_robot)
        
        # [중요] 쿼터니언 계산용으로 스케일링 전 행렬을 저장합니다.
        # (스케일링 된 행렬로 회전을 구하면 수학적으로 깨짐)
        self.T_pure_rotation = T_world_unscaled.copy()

        # 3. [핵심 수정] Tip Correction (스케일링) 적용
        # 최종 T06 = (World변환) @ (Robot FK) @ (Tip 보정)
        self.T06 = np.dot(T_world_unscaled, self.T_tip_correction)
        
        return self.T06
    
    def get_position(self):
        # 스케일링이 적용된 최종 위치 반환
        return self.T06[:3, 3]

    def get_quaternion(self):
        """
        [핵심] 4x4 행렬의 회전 부분(3x3)을 Quaternion [x, y, z, w]로 변환
        주의: 스케일링이 포함된 T06 대신, 순수 회전 행렬인 T_pure_rotation을 사용해야 합니다.
        """
        # 수정: self.T06 대신 self.T_pure_rotation 사용
        R = self.T_pure_rotation[:3, :3] 
        
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        
        return np.array([qx, qy, qz, qw])

def main():
    # 1. 클라이언트 객체 생성 및 연결
    client = Coppeliasim_client()
    if not client.check_connection(): return
    
    client.initialize_handles() 
    
    # 2. 시뮬레이션 시작
    sim.simxStartSimulation(client.client_id, sim.simx_opmode_blocking)
    client.start_streaming()    
    
    print(">> Simulation Started... (Wait 1 sec)")
    time.sleep(1.0) 

    kinematics = HT_matrix()

    try:
        while True:
            # 데이터 가져오기
            angles, sim_tcp_world, sim_quat = client.get_data_synchronized()
            
            if len(angles) != 6: continue 

            # FK 계산 (여기서 Tip Correction이 적용됨)
            kinematics.ForwardKinematics(angles)
            my_tcp_world = kinematics.get_position()
            my_quat = kinematics.get_quaternion()

            # [검증 로직]
            if 'first_run_check' not in locals(): 
                sim_q = sim_quat
                x, y, z, w = sim_q
                R_sim = np.array([
                    [1 - 2*y*2 - 2*z*2,  2*x*y - 2*z*w,      2*x*z + 2*y*w],
                    [2*x*y + 2*z*w,      1 - 2*x*2 - 2*z*2,  2*y*z - 2*x*w],
                    [2*x*z - 2*y*w,      2*y*z + 2*x*w,      1 - 2*x*2 - 2*y*2]
                ])
                
                T_sim_matrix = np.eye(4)
                T_sim_matrix[:3, :3] = R_sim
                T_sim_matrix[:3, 3]  = sim_tcp_world

                # 내 FK (보정 적용됨)
                T_my_matrix = kinematics.T06 

                # 오차 행렬 계산 (이론상 Identity가 나와야 함)
                T_diff = np.dot(np.linalg.inv(T_my_matrix), T_sim_matrix)

                print("\n" + "="*50)
                print(">>> [Result Verification] 단위 행렬(Identity)에 가까워야 성공 <<<")
                with np.printoptions(precision=4, suppress=True):
                    print(T_diff)
                print("="*50 + "\n")
                
                first_run_check = True

            # 오차 계산
            error = np.linalg.norm(my_tcp_world - np.array(sim_tcp_world))

            dot_product = np.abs(np.dot(my_quat, np.array(sim_quat)))
            quat_error = 1.0 - dot_product 

            print(f"My Quat  : [{my_quat[0]:.4f}, {my_quat[1]:.4f}, {my_quat[2]:.4f}, {my_quat[3]:.4f}]")
            print(f"Sim Quat : [{sim_quat[0]:.4f}, {sim_quat[1]:.4f}, {sim_quat[2]:.4f}, {sim_quat[3]:.4f}]")
            print(f"Rot Error: {quat_error:.5f}")
            print("-" * 45)

            print(f"My FK (World): [{my_tcp_world[0]:.4f}, {my_tcp_world[1]:.4f}, {my_tcp_world[2]:.4f}]")
            print(f"Sim TCP(World): [{sim_tcp_world[0]:.4f}, {sim_tcp_world[1]:.4f}, {sim_tcp_world[2]:.4f}]")
            print(f"Error        : {error:.5f} m")
            print("-" * 45)
            
            if sim.simxGetConnectionId(client.client_id) == -1: break
            time.sleep(0.2) 

    except KeyboardInterrupt:
        print("\n>> 종료 요청")
    
    finally:
        client.stop_simulation()

if __name__ == "__main__":
    main()