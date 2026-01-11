import os
import time
import math
import numpy as np
import sim
import simConst
from dotenv import load_dotenv

# .env 파일 로드
load_dotenv()

class Coppeliasim_client: 
    def __init__(self, port):
        self.client_id = sim.simxStart('127.0.0.1', port, True, True, 5000, 5)
    
    def check_connection(self):
        if self.client_id != -1:
            print(">> CoppeliaSim 연결 성공")
        else:
            print("!! 연결 실패: 시뮬레이터가 실행 중인지 확인하세요.")
            exit()
    
    def get_joint_handle(self):
        self.joint_handles = []
        # UR3 조인트 이름 (CoppeliaSim Scene Hierarchy에 적힌 이름과 정확히 일치해야 함)
        self.joint_names = [f"UR3_joint{i}" for i in range(1, 7)]

        for jname in self.joint_names:
            returnHandle, handle = sim.simxGetObjectHandle(self.client_id, jname, simConst.simx_opmode_blocking) 
            if returnHandle == sim.simx_return_ok:
                self.joint_handles.append(handle)
            else:
                print(f"!! 핸들 얻기 실패: {jname}")
        return self.joint_handles
    
    def get_joint_angle(self):
        if not hasattr(self, 'joint_handles') or not self.joint_handles:
            return None
        
        joint_angles = []
        for jhandle in self.joint_handles:
            ret, angle = sim.simxGetJointPosition(self.client_id, jhandle, simConst.simx_opmode_blocking)
            if ret == sim.simx_return_ok:
                joint_angles.append(angle)
            else:
                return None
        return joint_angles
    
    def get_tcp_position(self):
        # UR3_connection은 보통 End-Effector의 좌표계입니다.
        ret, tcp_handle = sim.simxGetObjectHandle(self.client_id, "UR3_connection", simConst.simx_opmode_blocking)
        if ret == sim.simx_return_ok:
            ret_pos, tcp_position = sim.simxGetObjectPosition(self.client_id, tcp_handle, -1, simConst.simx_opmode_blocking)
            if ret_pos == sim.simx_return_ok:
                return tcp_position
        return [0, 0, 0]

class HT_matrix:
    def __init__(self):
        # .env에서 로드한 문자열을 파싱하여 리스트로 저장
        self.dh_d     = self.parse_dh_param(os.getenv("DH_D"))
        self.dh_a     = self.parse_dh_param(os.getenv("DH_A"))
        self.dh_alpha = self.parse_dh_param(os.getenv("DH_ALPHA"))
        # [핵심] Base -> World 변환 행렬 정의
        # 데이터 분석 결과: My_Y -> Sim_X 로 매핑되고 있음
        # 따라서 Z축 기준 -90도 회전 행렬을 적용합니다.
        # 만약 로봇 베이스가 (0,0,0)이 아닌 다른 곳에 있다면 마지막 열에 x,y,z 좌표를 넣으면 됩니다.
        self.T_base_to_world = np.array([
            [ 0,  1,  0,  0],  # Base의 Y가 World의 X가 됨
            [-1,  0,  0,  0],  # Base의 X가 World의 -Y가 됨 (오른손 법칙)
            [ 0,  0,  1,  0],  # Z는 그대로
            [ 0,  0,  0,  1]
        ])
        # 디버깅: 파라미터가 잘 들어갔는지 확인
        print(f"Loaded DH_D: {self.dh_d}")
        print(f"Loaded DH_A: {self.dh_a}")

    def parse_dh_param(self, env_str):
        """ 'pi/2, 0, ...' 문자열을 [1.57, 0, ...] 리스트로 변환 """
        if not env_str:
            return [0]*6
        
        # 'pi' 문자를 math 모듈의 값으로 치환 후 계산
        safe_env_str = env_str.replace('pi', str(math.pi))
        try:
            return [float(eval(item)) for item in safe_env_str.split(',')]
        except Exception as e:
            print(f"DH Parameter Parsing Error: {e}")
            return [0]*6

    def get_dh_Transform(self, theta, d, a, alpha):
        """ Standard DH Transformation Matrix """
        return np.array([
            [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0,                math.sin(alpha),                  math.cos(alpha),                 d],
            [0,                0,                                0,                               1]
        ])

    def ForwardKinematics(self, joint_angles):
        """ 
        Forward Kinematics 계산 
        joint_angles: 현재 로봇의 관절 각도 리스트 (6개)
        """
        # UR3 Kinematics 보정 (CoppeliaSim 좌표계 vs Standard DH 보정)
        # J2, J4 등 초기 자세가 다를 경우 여기서 Offset을 더해줍니다.
        # 예: theta_corrected = [q[0], q[1]-pi/2, q[2], q[3]-pi/2, q[4], q[5]] 
        # *필요 시 수정하세요*
        thetas = [
            joint_angles[0],
            joint_angles[1] - math.pi/2, 
            joint_angles[2],
            joint_angles[3] - math.pi/2,
            joint_angles[4],
            joint_angles[5]
        ]

        T_final = np.eye(4)
        
        for i in range(6):
            T_i = self.get_dh_Transform(
                thetas[i], 
                self.dh_d[i], 
                self.dh_a[i], 
                self.dh_alpha[i]
            )
            T_final = np.dot(T_final, T_i)
        self.T06 = np.dot(self.T_base_to_world, T_final)
        return self.T06
    
    def get_position(self):
        return self.T06[:3, 3] # x, y, z 부분만 반환

def main():
    # 1. 시뮬레이션 연결
    client = Coppeliasim_client(19999) # 포트 번호 확인
    client.check_connection()
    client.get_joint_handle()
    
    # 2. Kinematics 객체 생성 (DH 파라미터는 생성자에서 한 번만 로드)
    kinematics = HT_matrix()
    
    # 3. 시뮬레이션 시작
    sim.simxStartSimulation(client.client_id, simConst.simx_opmode_blocking)
    print(">> Simulation Started")
    time.sleep(1)

    # ==================================================================
    # [검증 코드] 여기에 넣으세요! (try 진입 전)
    # 시뮬레이터 내부의 "진짜" 치수를 측정해서 출력합니다.
    # ==================================================================
    print("\n[--- DH Parameter Verification ---]")
    
    # 핸들 가져오기 (편의상 변수 할당)
    # joint_handles[0] = J1, [1] = J2, [2] = J3 ... (인덱스 주의)
    h_j1 = client.joint_handles[0]
    h_j2 = client.joint_handles[1]
    h_j3 = client.joint_handles[2]
    h_j4 = client.joint_handles[3]
    
    # 절대 좌표(World Frame 기준) 가져오기
    _, pos_j1 = sim.simxGetObjectPosition(client.client_id, h_j1, -1, simConst.simx_opmode_blocking)
    _, pos_j2 = sim.simxGetObjectPosition(client.client_id, h_j2, -1, simConst.simx_opmode_blocking)
    _, pos_j3 = sim.simxGetObjectPosition(client.client_id, h_j3, -1, simConst.simx_opmode_blocking)
    
    # d1 계산 (J2의 높이 - J1의 높이 ... 보통 J1이 바닥에 있다면 J2의 Z값)
    # 정확히는 Base에서 J2까지의 수직 거리입니다.
    calc_d1 = pos_j2[2] - pos_j1[2] + 0.0 # 만약 J1이 바닥(0)보다 떠있다면 보정 필요하지만 보통 이 차이가 d1
    # 참고: UR3 모델링에 따라 J1의 원점이 바닥이 아닐 수 있으므로, 
    # 확실한 건 'Base' 객체의 Z와 'J2' 객체의 Z 차이를 구하는 것입니다. 
    # 하지만 일단 J2의 절대 높이(pos_j2[2])가 d1과 가장 유사할 것입니다.
    
    # a2 계산 (J2와 J3 사이의 거리)
    # J2와 J3는 높이(z)는 같고 수평 거리만 다릅니다.
    calc_a2 = math.sqrt((pos_j2[0]-pos_j3[0])**2 + (pos_j2[1]-pos_j3[1])**2 + (pos_j2[2]-pos_j3[2])**2)
    
    # a3 계산 (J3와 J4 사이의 거리)
    _, pos_j4 = sim.simxGetObjectPosition(client.client_id, h_j4, -1, simConst.simx_opmode_blocking)
    calc_a3 = math.sqrt((pos_j3[0]-pos_j4[0])**2 + (pos_j3[1]-pos_j4[1])**2 + (pos_j3[2]-pos_j4[2])**2)

    print(f"Measured d1 (Base~J2 Z diff) : {pos_j2[2]:.5f} m (Standard: 0.1519)")
    print(f"Measured a2 (J2~J3 Distance) : {calc_a2:.5f} m (Standard: 0.24365)")
    print(f"Measured a3 (J3~J4 Distance) : {calc_a3:.5f} m (Standard: 0.21325)")
    print("----------------------------------\n")
    # ==================================================================

    try:
        while True:
            # 연결 상태 확인
            if sim.simxGetConnectionId(client.client_id) == -1: 
                print("!! CoppeliaSim 연결 끊김.")
                break
            
            # 현재 관절 각도 읽기
            angles = client.get_joint_angle()
            if angles is None or len(angles) != 6:
                continue # 데이터를 못 읽었으면 스킵

            # FK 계산
            kinematics.ForwardKinematics(angles)
            my_xyz = kinematics.get_position()
            
            # CoppeliaSim 실제 TCP 위치 읽기 (비교용)
            sim_xyz = client.get_tcp_position()

            # 결과 출력 (비교)
            print("-" * 50)
            print(f"My FK : X={my_xyz[0]}, Y={my_xyz[1]}, Z={my_xyz[2]}")
            print(f"Sim TCP: X={sim_xyz[0]}, Y={sim_xyz[1]}, Z={sim_xyz[2]}")
            
            # 오차 확인 (너무 크면 DH 파라미터나 Offset 수정 필요)
            error = np.linalg.norm(np.array(my_xyz) - np.array(sim_xyz))
            print(f"Error : {error} m")
            

            time.sleep(0.1) # 과부하 방지

    except KeyboardInterrupt:
        print("\n>> 종료 요청 받음.")
    
    finally:
        sim.simxStopSimulation(client.client_id, simConst.simx_opmode_blocking)
        sim.simxFinish(client.client_id)
        print(">> Remote API 연결 종료")

if __name__ == "__main__":
    main()