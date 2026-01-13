import sim
import simConst
import os
from dotenv import load_dotenv

load_dotenv()

class Coppeliasim_client: 
    def __init__(self, port = None):
        if port is None:
            port = int(os.getenv("PORT", 19999))
            
        self.client_id = sim.simxStart('127.0.0.1', port, True, True, 5000, 5)
        self.joint_handles = []
        self.tcp_handle = None
    
    def check_connection(self):
        if self.client_id != -1:
            print(">> CoppeliaSim 연결 성공")
            return True
        else:
            print("!! 연결 실패: 시뮬레이터가 실행 중인지 확인하세요.")
            return False

    def initialize_handles(self):
        # Joint와 TCP 핸들을 찾아서 클래스 변수에 저장
        self.joint_handles = []
        joint_names = [f"UR3_joint{i}" for i in range(1, 7)]

        # Joint Handles
        for jname in joint_names:
            ret, handle = sim.simxGetObjectHandle(self.client_id, jname, simConst.simx_opmode_blocking)
            if ret == sim.simx_return_ok:
                self.joint_handles.append(handle)
            else:
                print(f"!! 핸들 찾기 실패: {jname}")
        
        # TCP Handle
        ret, self.tcp_handle = sim.simxGetObjectHandle(self.client_id, "UR3_connection", simConst.simx_opmode_blocking)
        if ret != sim.simx_return_ok:
            print("!! TCP 핸들(UR3_connection) 찾기 실패")

    def start_streaming(self):
        """데이터 스트리밍 시작 (최초 1회 호출)"""
        # 관절 각도 스트리밍 등록
        for h in self.joint_handles:
            sim.simxGetJointPosition(self.client_id, h, simConst.simx_opmode_streaming)
        
        # TCP 위치 스트리밍 등록 (World 기준: -1)
        if self.tcp_handle:
            sim.simxGetObjectPosition(self.client_id, self.tcp_handle, -1, simConst.simx_opmode_streaming)

    def get_data_synchronized(self):
        """ 반환값: (joint_angles, tcp_position_world) """
        
        # 1. 통신 일시 정지 (스냅샷)
        sim.simxPauseCommunication(self.client_id, True)

        # 2. 데이터 읽기 (Buffer에서 즉시 로드)
        angles = []
        for h in self.joint_handles:
            ret, val = sim.simxGetJointPosition(self.client_id, h, simConst.simx_opmode_buffer)
            if ret == sim.simx_return_ok:
                angles.append(val)
        
        tcp_pos = [0, 0, 0]
        if self.tcp_handle:
            # World 기준(-1) 좌표 가져오기
            _, tcp_pos = sim.simxGetObjectPosition(self.client_id, self.tcp_handle, -1, simConst.simx_opmode_buffer)

        # 3. 통신 재개
        sim.simxPauseCommunication(self.client_id, False)

        return angles, tcp_pos

    def stop_simulation(self):
        sim.simxStopSimulation(self.client_id, simConst.simx_opmode_blocking)
        sim.simxFinish(self.client_id)
        print(">> 연결 종료")