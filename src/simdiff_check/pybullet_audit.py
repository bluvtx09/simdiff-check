"""PyBullet 시뮬레이션 검사.

PyBullet은 설정이 파일이 아니라 코드 호출로 만들어지고, 링크 감쇠·contactERP·로드 플래그를
되읽는 API가 없다. 그래서 두 가지를 섞는다.

  1) 기록기: install() 이후의 loadURDF / createMultiBody / changeDynamics /
     setPhysicsEngineParameter / setTimeStep 호출을 기록한다.
  2) 실측: 시뮬레이션 상태(getDynamicsInfo, getCollisionShapeData, 엔진 파라미터)를 직접 읽는다.
     URDF 관성은 파일을 다시 읽어 엔진이 실제로 쓰는 값과 비교한다.

사용법:
    import simdiff_check.pybullet as sdc   # 로드 전에
    sdc.install()
    ... 평소대로 장면 구성 ...
    print(sdc.audit(event_time=0.4).to_text())
"""
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np

from . import rules as R
from .i18n import t
from .report import Report


class _Recorder:
    def __init__(self):
        self.clear()

    def clear(self):
        self.loads = defaultdict(dict)      # cid -> body -> {"kind", "path", "flags"}
        self.damping = defaultdict(dict)    # cid -> body -> {"linear": v, "angular": v}
        self.params = defaultdict(dict)     # cid -> {"contactERP": v, ...}


REC = _Recorder()
_ORIG = {}


def _cid(kw):
    return kw.get("physicsClientId", 0)


def install(p=None):
    """pybullet 모듈 함수를 감싸 설정 호출을 기록한다. 여러 번 불러도 안전."""
    if p is None:
        import pybullet as p
    if _ORIG:
        return p

    def wrap(name, after):
        f = getattr(p, name)
        _ORIG[name] = f

        def g(*a, **kw):
            out = f(*a, **kw)
            try:
                after(out, a, kw)
            except Exception:  # 기록 실패가 사용자 시뮬레이션을 깨면 안 됨
                pass
            return out
        g.__name__ = name
        g.__doc__ = f.__doc__
        setattr(p, name, g)

    def on_load(out, a, kw):
        path = a[0] if a else kw.get("fileName")
        flags = kw.get("flags", a[5] if len(a) > 5 else 0)
        REC.loads[_cid(kw)][out] = {"kind": "urdf", "path": str(path), "flags": int(flags or 0)}

    def on_create(out, a, kw):
        REC.loads[_cid(kw)][out] = {"kind": "createMultiBody", "path": None, "flags": 0}

    def on_dyn(out, a, kw):
        body = a[0] if a else kw.get("bodyUniqueId")
        rec = REC.damping[_cid(kw)].setdefault(body, {})
        for k, key in (("linearDamping", "linear"), ("angularDamping", "angular")):
            if k in kw and kw[k] is not None and kw[k] >= 0:
                rec[key] = float(kw[k])

    def on_params(out, a, kw):
        for k in ("contactERP", "erp", "frictionERP", "numSubSteps", "fixedTimeStep"):
            if k in kw:
                REC.params[_cid(kw)][k] = kw[k]

    def on_reset(out, a, kw):
        c = _cid(kw)
        REC.loads[c].clear(); REC.damping[c].clear(); REC.params[c].clear()

    wrap("loadURDF", on_load)
    wrap("createMultiBody", on_create)
    wrap("changeDynamics", on_dyn)
    wrap("setPhysicsEngineParameter", on_params)
    wrap("resetSimulation", on_reset)
    return p


def uninstall(p=None):
    if p is None:
        import pybullet as p
    for k, f in _ORIG.items():
        setattr(p, k, f)
    _ORIG.clear()
    REC.clear()


# ---- URDF 관성 ---------------------------------------------------------------

def _urdf_inertias(path):
    """링크 이름 -> 주관성 모멘트(오름차순). <inertial>이 없는 링크는 빠진다."""
    out = {}
    try:
        root = ET.parse(path).getroot()
    except Exception:
        return out
    for link in root.iter("link"):
        it = link.find("inertial/inertia")
        if it is None:
            continue
        g = lambda k: float(it.get(k, 0))
        Imat = np.array([[g("ixx"), g("ixy"), g("ixz")], [g("ixy"), g("iyy"), g("iyz")],
                         [g("ixz"), g("iyz"), g("izz")]])
        out[link.get("name")] = np.sort(np.linalg.eigvalsh(Imat))
    return out


def _link_names(p, body, cid):
    names = {-1: p.getBodyInfo(body, physicsClientId=cid)[0].decode()}
    for j in range(p.getNumJoints(body, physicsClientId=cid)):
        names[j] = p.getJointInfo(body, j, physicsClientId=cid)[12].decode()
    return names


# ---- 검사 ------------------------------------------------------------------

def audit(physicsClientId=0, event_time=None, urdf_paths=None, p=None):
    """현재 PyBullet 장면을 검사해 Report를 돌려준다.

    event_time: 관심 있는 사건의 지속시간(초). 주면 스텝 수 규칙을 검사한다.
    urdf_paths: {body_id: urdf 경로}. install() 없이 불러온 몸체의 관성을 검사할 때.
    """
    if p is None:
        import pybullet as p
    cid = physicsClientId
    pe = p.getPhysicsEngineParameters(physicsClientId=cid)
    sub = int(pe.get("numSubSteps", 0) or 0)
    dt = float(pe["fixedTimeStep"]) / (sub if sub > 0 else 1)
    recorded = bool(_ORIG)
    cerp = REC.params[cid].get("contactERP")
    ctx = {"dt": dt, "contactERP": cerp if cerp is not None else t("pb_erp_default", v=R.PB_DEFAULT_CONTACT_ERP),
           "event_time": event_time}
    rep = Report("pybullet", f"physicsClientId={cid}", ctx)
    if sub > 0:
        rep.notes.append(t("pb_substeps", n=sub, dt=dt))
    if not recorded:
        rep.notes.append(t("pb_not_installed"))

    for rid in R.check_numbers("pybullet", dt, event_time=event_time,
                               contact_erp=cerp if cerp is not None else R.PB_DEFAULT_CONTACT_ERP):
        where = "setPhysicsEngineParameter" if rid == "PB005" else "fixedTimeStep"
        val = {"PB004": f"dt {dt * 1000:.2f} ms", "PB005": f"contactERP {cerp}"}.get(
            rid, t("steps", T=event_time, dt=dt, n=(event_time or 0) / dt))
        rep.add(rid, where, val)

    bodies = [p.getBodyUniqueId(i, physicsClientId=cid) for i in range(p.getNumBodies(physicsClientId=cid))]
    info = {}
    for b in bodies:
        nj = p.getNumJoints(b, physicsClientId=cid)
        links = {l: p.getDynamicsInfo(b, l, physicsClientId=cid) for l in range(-1, nj)}
        dynamic = nj > 0 or links[-1][0] > 0
        colliding = [l for l in range(-1, nj) if p.getCollisionShapeData(b, l, physicsClientId=cid)]
        info[b] = {"nj": nj, "links": links, "dynamic": dynamic, "colliding": colliding,
                   "name": p.getBodyInfo(b, physicsClientId=cid)[1].decode() or f"body{b}"}

    # ---- PB001 감쇠 ----------------------------------------------------------
    if recorded:
        for b in bodies:
            if not info[b]["dynamic"]:
                continue
            d = REC.damping[cid].get(b, {})
            lin, ang = d.get("linear", R.PB_DEFAULT_DAMPING), d.get("angular", R.PB_DEFAULT_DAMPING)
            if lin > 0 or ang > 0:
                src = REC.loads[cid].get(b, {}).get("path") or "createMultiBody"
                rep.add("PB001", f"body {b} ({info[b]['name']}, {Path(src).name if src else src})",
                        f"linearDamping {lin:g}, angularDamping {ang:g}"
                        + (t("pb_default") if not d else ""))

    # ---- PB002 관성 ----------------------------------------------------------
    paths = {}
    for b, rec in REC.loads[cid].items():
        if rec["kind"] == "urdf" and not (rec["flags"] & p.URDF_USE_INERTIA_FROM_FILE):
            paths[b] = rec["path"]
    for b, path in (urdf_paths or {}).items():
        paths[b] = path
    for b, path in paths.items():
        if b not in info:
            continue
        ref = _urdf_inertias(path)
        names = _link_names(p, b, cid)
        bad = []
        for l, nm in names.items():
            if nm not in ref:
                continue
            got = np.sort(np.array(info[b]["links"][l][2], dtype=float))
            want = ref[nm]
            if np.max(np.abs(got - want)) > 0.01 * max(np.max(np.abs(want)), 1e-12):
                bad.append((nm, want, got))
        if bad:
            nm, want, got = max(bad, key=lambda t: np.max(np.abs(t[1] - t[2]) / np.maximum(t[1], 1e-12)))
            rep.add("PB002", t("pb_links", b=b, f=Path(path).name, n=len(bad)),
                    t("pb_inertia", nm=nm, want=np.round(want, 8).tolist(), got=np.round(got, 8).tolist()))

    # ---- PB003 마찰 곱 ----------------------------------------------------------
    def mu_of(b):
        return min(float(info[b]["links"][l][1]) for l in info[b]["colliding"])
    seen = []
    for i, a in enumerate(bodies):
        for b in bodies[i + 1:]:
            if not (info[a]["dynamic"] or info[b]["dynamic"]):
                continue
            if not (info[a]["colliding"] and info[b]["colliding"]):
                continue  # 충돌 모양이 없는 몸체는 닿을 수 없음
            ma, mb = mu_of(a), mu_of(b)
            if ma < 1 and mb < 1:
                seen.append((ma * mb, a, b, ma, mb))
    for eff, a, b, ma, mb in sorted(seen)[:20]:
        rep.add("PB003", f"body {a} ({info[a]['name']}) ↔ body {b} ({info[b]['name']})",
                t("pb_friction", a=ma, b=mb, e=eff))
    if len(seen) > 20:
        rep.notes.append(t("pb_friction_more", n=len(seen)))

    # ---- PB006 넘어짐 여유 -------------------------------------------------------
    statics = [b for b in bodies if not info[b]["dynamic"] and info[b]["colliding"]]
    for b in bodies:
        if info[b]["nj"] != 0 or not info[b]["dynamic"] or not statics:
            continue
        shapes = p.getCollisionShapeData(b, -1, physicsClientId=cid)
        if len(shapes) != 1 or shapes[0][2] != p.GEOM_BOX:
            continue
        half = np.array(shapes[0][3], dtype=float) / 2
        orn = p.getBasePositionAndOrientation(b, physicsClientId=cid)[1]
        rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        up = int(np.argmax(np.abs(rot[2])))
        h, base = half[up], min(half[i] for i in range(3) if i != up)
        mu = float(info[b]["links"][-1][1]) * max(mu_of(s) for s in statics)
        tau = R.tipping_margin(mu, h, base)
        if "PB006" in R.check_numbers("pybullet", dt, tau=tau):
            extra = t("tau_extra") if tau >= 1 else ""
            rep.add("PB006", f"body {b} ({info[b]['name']})",
                    t("pb_tau", mu=mu, h=h, b=base, tau=tau, k=R.PB_TAU_MAX, extra=extra))

    rep.add("INFO01", "PyBullet", t("first_order", name=t("pb_integrator")))
    return rep
