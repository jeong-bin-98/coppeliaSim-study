import sim

sim.simxFinish(-1)
client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if client_id != -1:
    print("✅ 연결 성공!")
    sim.simxFinish(client_id)
else:
    print("❌ 연결 실패")