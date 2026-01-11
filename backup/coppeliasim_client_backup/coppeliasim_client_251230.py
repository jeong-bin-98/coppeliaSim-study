import sim
import simConst

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