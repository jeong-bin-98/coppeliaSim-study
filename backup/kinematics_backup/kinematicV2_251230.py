import os
import sim
import simConst
import time
from math import *
import numpy as np
from dotenv import load_dotenv

load_dotenv()

## 내일할 일
# 에러 비교

class Coppeliasim_client: 
    def __init__(self, port):
       self.client_id = sim.simxStart('127.0.0.1', port, True, True, 5000, 5)
    
    def check_connection(self): # 시뮬레이터와의 연결 상태를 확인하는 함수
        if self.client_id != -1:
            print("연결 성공")
        else:
            print("연결 실패")
    
    def get_joint_handle(self): # joint의 ID을 얻는 함수
        self.joint_handles = []
        self.joint_names = [f"UR3_joint{i}" for i in range(1, 7)] # 사용하는 로봇 joint 이름에 따라 변경 필요 (UR5 robot 기준)

        # simGetObjectHandle는 (returnCode, handle) 튜플을 반환 즉, 2개의 값을 반환``
        # returnCode는 함수 실행 결과를 나타내는 정수 값, handle은 객체의 고유 식별자(coppeliaSim 내에서 확인할 것 EX) deplicated name: object name)
        for jname in self.joint_names:
            returnHandle, handle = sim.simxGetObjectHandle(self.client_id, jname, simConst.simx_opmode_blocking) 
            
            if returnHandle == sim.simx_return_ok:
                self.joint_handles.append(handle)

        return self.joint_handles
    
    def get_joint_angle(self): # joint handle을 받아와서 해당 joint의 현재 각도를 읽어오는 함수
        if not self.joint_handles:
            raise Exception("!!! 먼저 get_joint_handle() 불러와 !!!")
        
        self.joint_angle = []

        for jhandle in self.joint_handles:
            returnAngle, angle = sim.simxGetJointPosition(self.client_id, jhandle, simConst.simx_opmode_blocking) # handle에 해당하는 joint의 현재 각도를 반환
            
            if returnAngle == sim.simx_return_ok:
                self.joint_angle.append(angle)
            else:
                return None

        return self.joint_angle
    
    def get_tcp_position(self): # TCP의 현재 위치를 읽어오는 함수
        returnTCP_handle, tcp_handle = sim.simxGetObjectHandle(self.client_id, "UR3_connection", simConst.simx_opmode_blocking)

        if returnTCP_handle == sim.simx_return_ok:
            returnTCP_position, tcp_position = sim.simxGetObjectPosition(self.client_id, tcp_handle, -1, simConst.simx_opmode_blocking)
            
            if returnTCP_position == sim.simx_return_ok:
                return tcp_position

class HT_matrix:
    def __init__(self, dh_theta):
        self.q = dh_theta
        
        self.dh_theta = [
            self.q[0],
            self.q[1] - pi/2,   
            self.q[2],
            self.q[3] + pi/2,
            self.q[4],
            self.q[5] - pi/2
        ]
        # DH parameter (UR5 robot 기준, 사용하는 로봇에 따라 변경 필요)
        self.dh_d     = [float(x) for x in os.getenv("DH_D").split(",")]
        self.dh_a     = [float(x) for x in os.getenv("DH_A").split(",")]
        self.dh_alpha = [float(x) for x in os.getenv("DH_ALPHA").split(",")]

    def get_dh_Transform(self, i): # DH 변환 행렬 계산(단일 관절 기준),  T_i를 만드는 것, Standard form
        return np.array([[cos(self.dh_theta[i]), -sin(self.dh_theta[i])*cos(self.dh_alpha[i]),  sin(self.dh_theta[i])*sin(self.dh_alpha[i]), self.dh_a[i]*cos(self.dh_theta[i])],
                         [sin(self.dh_theta[i]),  cos(self.dh_theta[i])*cos(self.dh_alpha[i]), -cos(self.dh_theta[i])*sin(self.dh_alpha[i]), self.dh_a[i]*sin(self.dh_theta[i])],
                         [0                    ,  sin(self.dh_alpha[i])                      ,  cos(self.dh_alpha[i])                      , self.dh_d[i]                      ],
                         [0                    ,  0                                          ,  0                                          , 1                                 ]])
       
    def dummy_dTrans(self, d):
        return np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, d],
                         [0, 0, 0, 1]])

    def ForwardKinematics(self):
        T01 = np.eye(4) # 4 x 4 형태 / N=4, M=4와 동일함
        T1D = np.eye(4)
        TD2 = np.eye(4)
        T23 = np.eye(4)
        T34 = np.eye(4)
        T45 = np.eye(4)
        T56 = np.eye(4)

        T01 = self.get_dh_Transform(0)    # Joint value
        ###############################
        T1D = self.dummy_dTrans(0.1117)   # d value(dummy1 frame)
        ###############################
        TD2 = self.get_dh_Transform(1)
        T23 = self.get_dh_Transform(2)
        T34 = self.get_dh_Transform(3)
        T45 = self.get_dh_Transform(4)
        T56 = self.get_dh_Transform(5)

        self.T0D_1 = np.dot(T01,T1D)
        self.T02 = np.dot(self.T0D_1,TD2)
        self.T03 = np.dot(self.T02,T23)
        self.T04 = np.dot(self.T03,T34)
        self.T05 = np.dot(self.T04,T45)
        self.T06 = np.dot(self.T05,T56)

        return self.T06
    
    def show_dh_Parameters(self):
        for i in range(6):
            print(f"Joint {i+1}: d = {self.dh_d[i]}, a = {self.dh_a[i]}, alpha = {self.dh_alpha[i]}, theta = {self.dh_theta[i]}")

    def show_position(self):
        self.x_p = self.T06[0][3]
        self.y_p = self.T06[1][3]
        self.z_p = self.T06[2][3]
        
        print(f"Position X : {self.x_p}, Position Y : {self.y_p}, Position Z : {self.z_p}")

    def show_orientation(self): 
        print(f"[[{self.T06[0][0]}, {self.T06[0][1]}, {self.T06[0][2]}],")
        print(f" [{self.T06[1][0]}, {self.T06[1][1]}, {self.T06[1][2]}],")
        print(f" [{self.T06[2][0]}, {self.T06[2][1]}, {self.T06[2][2]}]]")

def main():
    client = Coppeliasim_client(19999)
    client.check_connection() # Test connection
    client.get_joint_handle() # 조인트 핸들 읽기
    
    sim.simxStartSimulation(client.client_id, simConst.simx_opmode_blocking)

    while True:
        if sim.simxGetConnectionId(client.client_id) == -1: 
            print("CoppeliaSim이 종료되어 Python코드도 종료합니다.")
            break
        angle = client.get_joint_angle()
        if angle is None or len(angle) != 6:
            print("CoppeliaSim이 종료되어 Python코드도 종료합니다.")
            break

        HT = HT_matrix(angle)
        # HT.show_dh_Parameters()
        
        HT.ForwardKinematics()
        HT.show_position()
        # HT.show_orientation()
        print(client.get_tcp_position())
        time.sleep(0.002) # 2ms
    
    sim.simxStopSimulation(client.client_id, simConst.simx_opmode_blocking)
    sim.simxFinish(client.client_id)
    print("Remote API 연결 종료 완료")

if __name__ == "__main__":
    main()
