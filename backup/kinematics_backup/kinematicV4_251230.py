import os
import math
import time
import numpy as np
import sim
import simConst
from dotenv import load_dotenv

load_dotenv()

class Coppeliasim_client: 
    def __init__(self, port):
        self.client_id = sim.simxStart('127.0.0.1', port, True, True, 5000, 5)
        self.joint_handles = []
        self.tcp_handle = None # TCP 핸들 저장용 변수 추가
    
    def check_connection(self):
        if self.client_id != -1:
            print(">> CoppeliaSim 연결 성공")
        else:
            print("!! 연결 실패: 시뮬레이터가 실행 중인지 확인하세요.")
            exit()
    
    def get_handles(self): 
        # Joint 핸들 얻기
        self.joint_handles = []
        self.joint_names = [f"UR3_joint{i}" for i in range(1, 7)]

        for jname in self.joint_names:
            returnHandle, handle = sim.simxGetObjectHandle(self.client_id, jname, simConst.simx_opmode_blocking) 
            if returnHandle == sim.simx_return_ok:
                self.joint_handles.append(handle)
            else:
                print(f"!! 핸들 얻기 실패: {jname}")
        
        # TCP 핸들도 미리 얻어서 저장해둡니다 (루프에서 쓰기 위해)
        ret, self.tcp_handle = sim.simxGetObjectHandle(self.client_id, "UR3_connection", simConst.simx_opmode_blocking)
        if ret != sim.simx_return_ok:
            print("!! TCP 핸들 얻기 실패")

        return self.joint_handles

class HT_matrix:
    def __init__(self):
        self.dh_d     = self.parse_dh_param(os.getenv("DH_D"))
        self.dh_a     = self.parse_dh_param(os.getenv("DH_A"))
        self.dh_alpha = self.parse_dh_param(os.getenv("DH_ALPHA"))
        
        # [180도 회전 행렬] World 좌표계 보정 (X->-X, Y->-Y)
        self.T_base_to_world = np.array([
            [ 0,  1,  0,  0],  # Base의 Y가 World의 X가 됨
            [-1,  0,  0,  0],  # Base의 X가 World의 -Y가 됨 (오른손 법칙)
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
        # [각도 보정] UR3 Standard -> CoppeliaSim
        th = [
            joint_angles[0],
            joint_angles[1] - math.pi/2, 
            joint_angles[2],
            joint_angles[3] - math.pi/2,
            joint_angles[4],
            joint_angles[5]
        ]

        T_final = np.eye(4)
        for i in range(6):
            T_i = self.get_dh_Transform(th[i], self.dh_d[i], self.dh_a[i], self.dh_alpha[i])
            T_final = np.dot(T_final, T_i)
            
        # World 좌표계 변환 적용
        self.T06 = np.dot(self.T_base_to_world, T_final)
        return self.T06
    
    def get_position(self):
        return self.T06[:3, 3]

def main():
    # 1. 시뮬레이션 연결
    client = Coppeliasim_client(19999)
    client.check_connection()
    client.get_handles() # Joint와 TCP 핸들을 모두 가져옵니다.
    
    # 2. Kinematics 객체 생성
    kinematics = HT_matrix()
    
    # 3. 시뮬레이션 시작
    sim.simxStartSimulation(client.client_id, simConst.simx_opmode_blocking)
    print(">> Simulation Started")
    time.sleep(1)

    # ==================================================================
    # [설정] 데이터 스트리밍 초기화 (Streaming)
    # 루프 들어가기 전에 "나 이 데이터 계속 받을 거야"라고 서버에 등록합니다.
    # ==================================================================
    for h in client.joint_handles:
        sim.simxGetJointPosition(client.client_id, h, simConst.simx_opmode_streaming)
    
    # TCP 위치 스트리밍 (World 기준이므로 -1 사용)
    sim.simxGetObjectPosition(client.client_id, client.tcp_handle, -1, simConst.simx_opmode_streaming)
    
    # 데이터가 찰 때까지 잠깐 대기
    time.sleep(0.5)

    try:
        while True:
            # 연결 확인
            if sim.simxGetConnectionId(client.client_id) == -1: 
                print("!! CoppeliaSim 연결 끊김.")
                break
            
            # ==============================================================
            # [핵심] 동기화된 데이터 읽기 (Pause -> Buffer -> Resume)
            # ==============================================================
            
            # 1. 통신 일시 정지 (모든 데이터의 시간을 멈춤)
            sim.simxPauseCommunication(client.client_id, True)

            # 2. 관절 각도 읽기 (Buffer에서 즉시 가져옴)
            angles = []
            for h in client.joint_handles:
                ret, val = sim.simxGetJointPosition(client.client_id, h, simConst.simx_opmode_buffer)
                if ret == sim.simx_return_ok:
                    angles.append(val)
            
            # 3. TCP 위치 읽기 (Buffer에서 즉시 가져옴, World 기준 -1)
            ret, sim_xyz = sim.simxGetObjectPosition(client.client_id, client.tcp_handle, -1, simConst.simx_opmode_buffer)

            # 4. 통신 재개 (시뮬레이터 진행)
            sim.simxPauseCommunication(client.client_id, False)
            
            # ==============================================================

            # 데이터 유효성 검사 (다 못 읽었으면 이번 턴 스킵)
            if len(angles) != 6 or ret != sim.simx_return_ok:
                continue

            # FK 계산
            kinematics.ForwardKinematics(angles)
            my_xyz = kinematics.get_position()
            
            # 오차 계산
            error = np.linalg.norm(my_xyz - np.array(sim_xyz))

            # 결과 출력
            print("-" * 50)
            print(f"My FK (World): X={my_xyz[0]:.4f}, Y={my_xyz[1]:.4f}, Z={my_xyz[2]:.4f}")
            print(f"Sim TCP(World): X={sim_xyz[0]:.4f}, Y={sim_xyz[1]:.4f}, Z={sim_xyz[2]:.4f}")
            print(f"Error        : {error:.5f} m")

            # 루프 속도 조절 (너무 빠르면 보기 힘드니 약간 지연, 제어 시엔 제거 가능)
            time.sleep(0.5) 

    except KeyboardInterrupt:
        print("\n>> 종료 요청 받음.")
    
    finally:
        sim.simxStopSimulation(client.client_id, simConst.simx_opmode_blocking)
        sim.simxFinish(client.client_id)
        print(">> Remote API 연결 종료")

if __name__ == "__main__":
    main()