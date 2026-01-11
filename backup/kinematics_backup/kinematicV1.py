import sim
import simConst
import time
from math import *
import numpy as np
import keyboard

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
        self.joint_names = [f"UR5_joint{i}" for i in range(1, 7)] # 사용하는 로봇 joint 이름에 따라 변경 필요 (UR5 robot 기준)
        # self.joint_names = [f"redundantRob_joint{i}" for i in range(1, 8)] # 사용하는 로봇 joint 이름에 따라 변경 필요 (redundant robot 기준)

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
        returnTCP_handle, tcp_handle = sim.simxGetObjectHandle(self.client_id, "UR5_connection", simConst.simx_opmode_blocking)
        # returnTCP_handle, tcp_handle = sim.simxGetObjectHandle(self.client_id, "redundantRob_tip", simConst.simx_opmode_blocking)

        if returnTCP_handle == sim.simx_return_ok:
            returnTCP_position, tcp_position = sim.simxGetObjectPosition(self.client_id, tcp_handle, -1, simConst.simx_opmode_blocking)
            
            if returnTCP_position == sim.simx_return_ok:
                return tcp_position

class HT_matrix:
    def __init__(self, dh_theta):
        self.q = dh_theta
        
        self.dh_theta = [
            ## 2
            self.q[0] - pi/2,
            self.q[1] + pi/2,   # 여기 오프셋 적용!
            self.q[2],
            self.q[3] - pi/2,
            self.q[4],
            self.q[5]

            # self.q[0],
            # self.q[1] - pi,   # 여기 오프셋 적용!
            # self.q[2] - pi,
            # self.q[3] + pi/2,
            # self.q[4],
            # self.q[5] - pi,
            # self.q[6]
            
        ]
        # DH parameter (UR5 robot 기준, 사용하는 로봇에 따라 변경 필요)
        # self.dh_d     = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]
        # self.dh_d     = [0.06605, 0, 0, 0.0397, 0.04918, 0.08938]
        # self.dh_a     = [0, -0.425, -0.39225, 0, 0, 0]
        # self.dh_alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]

        ## 2
        self.dh_d     = [0.06605, 0, 0, 0.09755, 0.09475, 0.08938]
        # self.dh_d     = [0.0892, 0, 0, 0.1093, 0.09475, 0.0825]
        self.dh_a     = [0, 0.425, 0.39225, 0, 0, 0]
        self.dh_alpha = [pi/2, 0, 0, -pi/2, pi/2, 0]

        ## redundantRob
        # self.dh_d     = [0.1392, 0, 0.1947, 0, 0.2335, 0, 0.06427]
        # self.dh_a     = [0, 0, 0, 0, 0, 0, 0]
        # self.dh_alpha = [pi/2, pi/2, -pi/2, pi/2, pi/2, pi/2, 0]

    def get_dh_Transform(self, i): # DH 변환 행렬 계산(단일 관절 기준),  T_i를 만드는 것, Standard form
        # Rz = np.array([[cos(self.dh_theta[i]),-sin(self.dh_theta[i]), 0                    , 0               ],
        #                [sin(self.dh_theta[i]), cos(self.dh_theta[i]), 0                    , 0               ],
        #                [0                    , 0                    , 1                    , 0               ],
        #                [0                    , 0                    , 0                    , 1               ]])

        # Td = np.array([[1                    , 0                    , 0                    , 0               ],
        #                [0                    , 1                    , 0                    , 0               ],
        #                [0                    , 0                    , 1                    , self.dh_d[i]    ],
        #                [0                    , 0                    , 0                    , 1               ]])
        
        # Ta = np.array([[1                    , 0                    , 0                    , self.dh_a[i]    ],
        #                [0                    , 1                    , 0                    , 0               ],
        #                [0                    , 0                    , 1                    , 0               ],
        #                [0                    , 0                    , 0                    , 1               ]])
        
        # Rx = np.array([[1                    , 0                    , 0                    , 0               ],
        #                [0                    , cos(self.dh_alpha[i]),-sin(self.dh_alpha[i]), 0               ],
        #                [0                    , sin(self.dh_alpha[i]), cos(self.dh_alpha[i]), 0               ],
        #                [0                    , 0                    , 0                    , 1               ]])

        return np.array([[cos(self.dh_theta[i]), -sin(self.dh_theta[i])*cos(self.dh_alpha[i]),  sin(self.dh_theta[i])*sin(self.dh_alpha[i]), self.dh_a[i]*cos(self.dh_theta[i])],
                         [sin(self.dh_theta[i]),  cos(self.dh_theta[i])*cos(self.dh_alpha[i]), -cos(self.dh_theta[i])*sin(self.dh_alpha[i]), self.dh_a[i]*sin(self.dh_theta[i])],
                         [0                    ,  sin(self.dh_alpha[i])                      ,  cos(self.dh_alpha[i])                      , self.dh_d[i]                      ],
                         [0                    ,  0                                          ,  0                                          , 1                                 ]])
       

    def ForwardKinematics(self):
        T01 = np.eye(4) # 4 x 4 형태 / N=4, M=4와 동일함
        T12 = np.eye(4)
        T23 = np.eye(4)
        T34 = np.eye(4)
        T45 = np.eye(4)
        T56 = np.eye(4)

        T01 = self.get_dh_Transform(0)
        T12 = self.get_dh_Transform(1)
        T23 = self.get_dh_Transform(2)
        T34 = self.get_dh_Transform(3)
        T45 = self.get_dh_Transform(4)
        T56 = self.get_dh_Transform(5)

        # self.T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56 # 행렬 곱
        self.T02 = np.dot(T01,T12)
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
        time.sleep(0.0001) # 2ms
    
    sim.simxStopSimulation(client.client_id, simConst.simx_opmode_blocking)
    sim.simxFinish(client.client_id)
    print("Remote API 연결 종료 완료")

if __name__ == "__main__":
    main()
