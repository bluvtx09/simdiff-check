import subprocess
import sys
from pathlib import Path

import pytest

EX = Path(__file__).parents[1] / "examples"


def cli(*args):
    r = subprocess.run([sys.executable, "-m", "simdiff_check.cli", *args], capture_output=True, text=True)
    return r.returncode, r.stdout


def test_trap_script():
    code, out = cli("pybullet", str(EX / "pybullet_trap.py"), "--event-time", "0.4", "--json")
    import json
    rep = json.loads(out[out.index("{"):])
    ids = {f["rule"] for f in rep["findings"]}
    assert {"PB001", "PB002", "PB003", "PB004", "PB005", "EV002"} <= ids and code == 1


def test_clean_script():
    code, out = cli("pybullet", str(EX / "pybullet_clean.py"), "--event-time", "0.4", "--strict")
    assert code == 0, out


def test_api_without_install_notes_limits():
    import pybullet as p
    import simdiff_check.pybullet as sp
    sp.uninstall(p)
    cid = p.connect(p.DIRECT)
    try:
        p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE, physicsClientId=cid), physicsClientId=cid)
        p.createMultiBody(1, p.createCollisionShape(p.GEOM_BOX, halfExtents=[.05] * 3, physicsClientId=cid),
                          basePosition=[0, 0, .05], physicsClientId=cid)
        rep = sp.audit(cid)
        assert "PB001" not in rep.rule_ids and any("install" in n for n in rep.notes)
        assert "PB003" in rep.rule_ids   # 기본값 0.5 x 0.5
    finally:
        p.disconnect(cid)


def test_install_records_damping_and_erp():
    import pybullet as p
    import simdiff_check.pybullet as sp
    sp.install(p)
    try:
        cid = p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(fixedTimeStep=0.001, contactERP=0.3, physicsClientId=cid)
        b = p.createMultiBody(1, p.createCollisionShape(p.GEOM_BOX, halfExtents=[.05] * 3, physicsClientId=cid),
                              physicsClientId=cid)
        p.changeDynamics(b, -1, linearDamping=0, physicsClientId=cid)   # angular은 그대로 0.04
        rep = sp.audit(cid)
        f = [x for x in rep.findings if x.rule == "PB001"]
        assert f and "angularDamping 0.04" in f[0].detail and rep.context["contactERP"] == 0.3
        p.disconnect(cid)
    finally:
        sp.uninstall(p)
