"""같은 장면을 규칙대로 고친 버전."""
from pathlib import Path
import pybullet as p

HERE = Path(__file__).parent
cid = p.connect(p.DIRECT)
p.setGravity(0, 0, -9.81)
p.setPhysicsEngineParameter(fixedTimeStep=0.001, contactERP=0.2)
pend = p.loadURDF(str(HERE / "assets/pendulum.urdf"), basePosition=[0, 0, 2], useFixedBase=True,
                  flags=p.URDF_USE_INERTIA_FROM_FILE)
floor = p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE))
box = p.createMultiBody(1, p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05]),
                        basePosition=[0, 0, 0.05])
p.changeDynamics(floor, -1, lateralFriction=1.0)                            # 바닥 1.0
p.changeDynamics(box, -1, lateralFriction=0.6)                              # 원하는 μ는 물체에
for b in (pend, box):
    for link in range(-1, p.getNumJoints(b)):
        p.changeDynamics(b, link, linearDamping=0, angularDamping=0)
for _ in range(100):
    p.stepSimulation()
p.disconnect()
