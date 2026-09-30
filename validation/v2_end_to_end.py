"""검증 V2: 처음 보는 무작위 장면에서 린터를 실제 입력 경로 그대로 돌린다 (표본 외 검증).

V1은 규칙을 만든 데이터(정육면체, 올바른 마찰 설정)에 다시 적용한 것이라 표본 내 검증이다.
V2는 새 난수로
  - 상자 높이/폭 비율을 0.5~1.5로 바꾸고 (V1은 정육면체만)
  - 흔한 실수(MuJoCo 바닥 마찰 기본값, PyBullet 기본 감쇠·양쪽 μ)를 20% 확률로 섞고
  - MuJoCo는 MJCF 문자열 → audit(), PyBullet은 install() 기록기 → audit() 를 거쳐
린터 판정과 실제 오차(해석해 대비 정지 거리)를 비교한다.

같은 장면을 dt/2, dt/4로도 돌려 V3(자기 불일치 신뢰도 판정)에 쓴다.
python validation/v2_end_to_end.py [N_mujoco] [N_pybullet]  →  validation/results/v2_{engine}.csv
"""
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np
import pandas as pd

G = 9.81
HALF = 0.05
SETTLE, MARGIN = 0.3, 0.7
OUT = Path(__file__).parent / "results"


def sample(engine, i):
    rng = np.random.default_rng([7, 0 if engine == "mujoco" else 1, i])
    while True:
        s = dict(i=i, dt=10 ** rng.uniform(-3.3, -2.0), mu=rng.uniform(0.1, 0.8), v0=rng.uniform(0.5, 3.0),
                 aspect=rng.uniform(0.5, 1.5))
        if s["mu"] * s["aspect"] < 0.95:   # 강체 이론상으로도 넘어지는 조건(τ≥1)은 해석해가 정답이 아니므로 제외
            break
    if engine == "mujoco":
        s.update(timeconst=10 ** rng.uniform(-2.3, -1.3), impratio=rng.uniform(1, 10),
                 cone=["pyramidal", "elliptic"][rng.integers(2)],
                 trap_floor_default=bool(rng.random() < 0.2))
    else:
        s.update(iterations=int(round(10 ** rng.uniform(0, 2))), contact_erp=rng.uniform(0.05, 0.8),
                 trap_damping=bool(rng.random() < 0.2), trap_both_mu=bool(rng.random() < 0.2))
    s["tau"] = s["mu"] * s["aspect"]
    s["t_stop"] = s["v0"] / (s["mu"] * G)
    s["d_true"] = s["v0"] ** 2 / (2 * s["mu"] * G)
    return s


# ---------------------------------------------------------------- MuJoCo
def mjcf(s, dt):
    hz = HALF * s["aspect"]
    floor_f = "" if s["trap_floor_default"] else f' friction="{s["mu"]} 0.005 0.0001"'
    return f"""<mujoco><option timestep="{dt}" cone="{s['cone']}" impratio="{s['impratio']}"/>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 1"{floor_f} solref="{s['timeconst']} 1"/>
    <body name="box" pos="0 0 {hz}"><freejoint/>
      <geom name="box" type="box" size="{HALF} {HALF} {hz}" mass="1" friction="{s['mu']} 0.005 0.0001"
            solref="{s['timeconst']} 1"/>
    </body>
  </worldbody></mujoco>"""


def run_mj(s, dt):
    import mujoco
    m = mujoco.MjModel.from_xml_string(mjcf(s, dt)); d = mujoco.MjData(m)
    for _ in range(int(round(SETTLE / dt))):
        mujoco.mj_step(m, d)
    x0 = d.qpos[0]; d.qvel[:] = 0; d.qvel[0] = s["v0"]
    for _ in range(int(round((s["t_stop"] + MARGIN) / dt))):
        mujoco.mj_step(m, d)
    q = d.qpos
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[4] ** 2 + q[5] ** 2), -1, 1)))
    return float(q[0] - x0), float(tilt)


def eval_mj(i):
    from simdiff_check import audit_mujoco
    s = sample("mujoco", i)
    rep = audit_mujoco(mjcf(s, s["dt"]), event_time=s["t_stop"])
    s["rules"] = ",".join(r for r in rep.rule_ids if r != "INFO01")
    s["n_error"], s["n_warning"] = rep.count("error"), rep.count("warning")
    for k, f in enumerate([1, 2, 4]):
        d, tilt = run_mj(s, s["dt"] / f)
        s[f"d{f}"] = d
        if f == 1:
            s["tilt_deg"] = tilt
    return s


# ---------------------------------------------------------------- PyBullet
def build_pb(p, s, dt):
    cid = p.connect(p.DIRECT)
    p.setGravity(0, 0, -G, physicsClientId=cid)
    p.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=0, numSolverIterations=s["iterations"],
                                contactERP=s["contact_erp"], physicsClientId=cid)
    hz = HALF * s["aspect"]
    floor = p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE, physicsClientId=cid), physicsClientId=cid)
    box = p.createMultiBody(1, p.createCollisionShape(p.GEOM_BOX, halfExtents=[HALF, HALF, hz], physicsClientId=cid),
                            basePosition=[0, 0, hz], physicsClientId=cid)
    p.changeDynamics(floor, -1, lateralFriction=s["mu"] if s["trap_both_mu"] else 1.0, restitution=0,
                     physicsClientId=cid)
    p.changeDynamics(box, -1, lateralFriction=s["mu"], restitution=0, physicsClientId=cid)
    if not s["trap_damping"]:
        p.changeDynamics(box, -1, linearDamping=0, angularDamping=0, physicsClientId=cid)
    return cid, box


def run_pb(p, s, dt, audit=False):
    cid, box = build_pb(p, s, dt)
    rep = None
    try:
        if audit:
            import simdiff_check.pybullet as sp
            rep = sp.audit(cid, event_time=s["t_stop"], p=p)
        for _ in range(int(round(SETTLE / dt))):
            p.stepSimulation(physicsClientId=cid)
        x0 = p.getBasePositionAndOrientation(box, physicsClientId=cid)[0][0]
        p.resetBaseVelocity(box, [s["v0"], 0, 0], [0, 0, 0], physicsClientId=cid)
        for _ in range(int(round((s["t_stop"] + MARGIN) / dt))):
            p.stepSimulation(physicsClientId=cid)
        pos, orn = p.getBasePositionAndOrientation(box, physicsClientId=cid)
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (orn[0] ** 2 + orn[1] ** 2), -1, 1)))
        return float(pos[0] - x0), float(tilt), rep
    finally:
        p.disconnect(physicsClientId=cid)


def eval_pb(i):
    import pybullet as p
    import simdiff_check.pybullet as sp
    sp.install(p)
    sp.REC.clear()
    s = sample("pybullet", i)
    for f in [1, 2, 4]:
        d, tilt, rep = run_pb(p, s, s["dt"] / f, audit=(f == 1))
        s[f"d{f}"] = d
        if f == 1:
            s["tilt_deg"] = tilt
            s["rules"] = ",".join(r for r in rep.rule_ids if r != "INFO01")
            s["n_error"], s["n_warning"] = rep.count("error"), rep.count("warning")
    return s


def main(n_mj=1200, n_pb=800):
    OUT.mkdir(exist_ok=True)
    for eng, fn, n in [("mujoco", eval_mj, n_mj), ("pybullet", eval_pb, n_pb)]:
        t0 = time.time()
        with Pool(2) as pool:
            rows = pool.map(fn, range(n), chunksize=4)
        df = pd.DataFrame(rows)
        df["err_pct"] = (df.d1 / df.d_true - 1) * 100
        df["flagged"] = df.rules.fillna("") != ""
        df.to_csv(OUT / f"v2_{eng}.csv", index=False)
        print(f"[{eng}] {n}개 장면, {time.time() - t0:.0f}s", flush=True)


if __name__ == "__main__":
    a = [int(x) for x in sys.argv[1:]]
    main(*a)
