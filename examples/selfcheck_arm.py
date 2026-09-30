"""자기 불일치 예제: 2링크 팔을 2초 흔들고 관절각 오차를 정답 없이 추정."""
import mujoco
import numpy as np

from simdiff_check import check_mujoco

XML = """<mujoco><option timestep="0.001" gravity="0 0 -9.81"/>
  <worldbody><body><joint name="j1" type="hinge" axis="0 1 0"/>
    <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" mass="1"/>
    <body pos="0 0 -0.5"><joint name="j2" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" mass="1"/></body></body></worldbody></mujoco>"""

model = mujoco.MjModel.from_xml_string(XML)


def start(m, d):
    d.qpos[:] = np.deg2rad([60, 30])


sc = check_mujoco(model, duration=2.0, setup=start, observe=lambda m, d: d.qpos.copy(), levels=3)
print(sc.to_text())
