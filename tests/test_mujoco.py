from pathlib import Path

import mujoco
import pytest

from simdiff_check import audit_mujoco

EX = Path(__file__).parents[1] / "examples"


def xml(floor="", box='friction="0.4 0.005 0.0001"', opt='timestep="0.001"', half_z=0.05, extra=""):
    return f"""<mujoco><option {opt}/>{extra}<worldbody>
      <geom name="floor" type="plane" size="0 0 1" {floor}/>
      <body name="b" pos="0 0 {half_z}"><freejoint/>
        <geom name="g" type="box" size="0.05 0.05 {half_z}" mass="1" {box}/></body></worldbody></mujoco>"""


def test_trap_example():
    rep = audit_mujoco(EX / "box_trap.xml", event_time=0.4)
    assert set(rep.rule_ids) == {"MJ001", "MJ002", "MJ003", "EV001", "INFO01"}
    assert rep.exit_code() == 1


def test_clean_example():
    rep = audit_mujoco(EX / "box_clean.xml", event_time=0.4)
    assert rep.rule_ids == ["INFO01"] and rep.exit_code(strict=True) == 0


def test_floor_default_friction_is_caught():
    assert "MJ002" in audit_mujoco(xml()).rule_ids
    assert "MJ002" not in audit_mujoco(xml(floor='friction="0.4 0.005 0.0001"')).rule_ids


def test_priority_resolves_friction():
    # 상자 geom의 priority가 높으면 상자 마찰이 쓰인다
    assert "MJ002" not in audit_mujoco(xml(box='friction="0.4 0.005 0.0001" priority="1"')).rule_ids


def test_tipping_uses_vertical_axis():
    flat = audit_mujoco(xml(floor='friction="0.8 0.005 0.0001"', box='friction="0.8 0.005 0.0001"', half_z=0.02))
    tall = audit_mujoco(xml(floor='friction="0.8 0.005 0.0001"', box='friction="0.8 0.005 0.0001"', half_z=0.1))
    assert "MJ003" not in flat.rule_ids and "MJ003" in tall.rule_ids


def test_solref_mixing_and_direct():
    # 바닥 0.02, 상자 0.002 → solmix 평균 0.011 = 11 dt (통과)
    rep = audit_mujoco(xml(floor='friction="0.4 0.005 0.0001" solref="0.02 1"',
                           box='friction="0.4 0.005 0.0001" solref="0.002 1"'))
    assert "MJ001" not in rep.rule_ids
    rep = audit_mujoco(xml(floor='friction="0.4 0.005 0.0001" solref="0.004 1"',
                           box='friction="0.4 0.005 0.0001" solref="0.004 1"'))
    assert "MJ001" in rep.rule_ids
    rep = audit_mujoco(xml(floor='friction="0.4 0.005 0.0001" solref="-10000 -100"',
                           box='friction="0.4 0.005 0.0001" solref="-10000 -100"'))
    assert "MJ001" not in rep.rule_ids and any("direct" in n for n in rep.notes)


def test_override_flag():
    opt = 'timestep="0.001" o_solref="0.003 1" o_friction="0.4 0.4 0.005 0.0001 0.0001"'
    rep = audit_mujoco(xml(opt=opt, extra="").replace("<option ", '<option ').replace(
        "/>", "><flag override=\"enable\"/></option>", 1))
    assert "MJ001" in rep.rule_ids and any("override" in n for n in rep.notes)


def test_rk4_no_info():
    assert "INFO01" not in audit_mujoco(xml(opt='timestep="0.001" integrator="RK4"',
                                            floor='friction="0.4 0.005 0.0001"')).rule_ids


def test_accepts_model_object_and_json():
    m = mujoco.MjModel.from_xml_path(str(EX / "box_trap.xml"))
    d = audit_mujoco(m).to_dict()
    assert d["summary"]["error"] == 1 and d["findings"][0]["rule"] == "MJ002"
