"""흔한 PyBullet 실수가 들어간 스크립트. simdiff-check pybullet examples/pybullet_trap.py --event-time 0.4"""
from pathlib import Path
import pybullet as p

HERE = Path(__file__).parent
cid = p.connect(p.DIRECT)
p.setGravity(0, 0, -9.81)
p.setPhysicsEngineParameter(fixedTimeStep=0.008, contactERP=0.8)           # 큰 dt, 큰 ERP
pend = p.loadURDF(str(HERE / "assets/pendulum.urdf"), basePosition=[0, 0, 2], useFixedBase=True)  # 플래그 없음
floor = p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE))
box = p.createMultiBody(1, p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05]),
                        basePosition=[0, 0, 0.05])
p.changeDynamics(floor, -1, lateralFriction=0.6)                            # 양쪽에 μ
p.changeDynamics(box, -1, lateralFriction=0.6)
for _ in range(100):
    p.stepSimulation()
p.disconnect()
