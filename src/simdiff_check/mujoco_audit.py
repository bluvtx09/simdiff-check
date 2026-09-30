"""MuJoCo 모델 검사.

XML을 문자열로 훑지 않고 컴파일된 MjModel을 읽는다. 그래서 <default> 상속, include,
refsafe 보정, override 플래그, geom priority 가 모두 반영된 '엔진이 실제로 쓰는 값'을 본다.
"""
from pathlib import Path

import numpy as np

from . import rules as R
from .i18n import t
from .report import Report


def _load(model):
    import mujoco
    if isinstance(model, mujoco.MjModel):
        return model, "MjModel"
    s = str(model)
    if s.lstrip().startswith("<"):
        return mujoco.MjModel.from_xml_string(s), "<xml string>"
    return mujoco.MjModel.from_xml_path(s), Path(s).name


def _name(m, objtype, i):
    import mujoco
    n = mujoco.mj_id2name(m, objtype, i)
    return n if n else f"#{i}"


def _pairs(m):
    """충돌 가능한 geom 쌍 (a, b, solref_tc, friction). 적어도 한쪽이 움직이는 쌍만."""
    import mujoco
    ng = m.ngeom
    moving = m.body_weldid[m.geom_bodyid] != 0
    ct, ca = m.geom_contype.astype(np.int64), m.geom_conaffinity.astype(np.int64)
    ia, ib = np.triu_indices(ng, 1)
    compat = ((ct[ia] & ca[ib]) | (ct[ib] & ca[ia])) != 0
    diff_body = m.geom_bodyid[ia] != m.geom_bodyid[ib]
    any_moving = moving[ia] | moving[ib]
    # MuJoCo는 부모-자식 body 사이 접촉을 기본으로 끈다 (filterparent)
    pa, pb = m.geom_bodyid[ia], m.geom_bodyid[ib]
    parent_child = (m.body_parentid[pa] == pb) | (m.body_parentid[pb] == pa)
    filterparent = not (m.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
    keep = compat & diff_body & any_moving
    if filterparent:
        keep &= ~(parent_child & (pa != 0) & (pb != 0))
    ia, ib = ia[keep], ib[keep]

    pri = m.geom_priority
    pa_hi, pb_hi = pri[ia] > pri[ib], pri[ib] > pri[ia]
    same = ~(pa_hi | pb_hi)
    # solref: 우선순위가 높은 쪽, 같으면 solmix 가중 평균
    mixa, mixb = m.geom_solmix[ia], m.geom_solmix[ib]
    tot = mixa + mixb
    wa = np.where(tot < 1e-15, 0.5, mixa / np.where(tot < 1e-15, 1, tot))
    tca, tcb = m.geom_solref[ia, 0], m.geom_solref[ib, 0]
    tc = np.where(pa_hi, tca, np.where(pb_hi, tcb, wa * tca + (1 - wa) * tcb))
    # 한쪽이라도 음수(직접 강성·감쇠 지정)면 timeconst 규칙 대상 아님
    tc = np.where((tca <= 0) | (tcb <= 0), np.nan, tc)
    # friction: 우선순위가 높은 쪽, 같으면 원소별 최댓값
    fa, fb = m.geom_friction[ia, 0], m.geom_friction[ib, 0]
    fr = np.where(pa_hi, fa, np.where(pb_hi, fb, np.maximum(fa, fb)))
    out = [(int(a), int(b), float(t), float(f)) for a, b, t, f in zip(ia, ib, tc, fr)]
    for k in range(m.npair):  # <contact><pair> 로 직접 지정한 쌍
        a, b = int(m.pair_geom1[k]), int(m.pair_geom2[k])
        t = float(m.pair_solref[k, 0])
        out.append((a, b, t if t > 0 else float("nan"), float(m.pair_friction[k, 0])))
    return out, same


def audit(model, event_time=None):
    """MuJoCo 모델(MjModel, XML 경로, XML 문자열)을 검사해 Report를 돌려준다.

    event_time: 관심 있는 사건(미끄러지다 멈추기, 한 번의 접촉 등)의 지속시간(초).
                주면 스텝 수 규칙(EV001/EV002)과 MJ004를 검사한다.
    """
    import mujoco
    m, src = _load(model)
    dt = float(m.opt.timestep)
    refsafe = not (m.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_REFSAFE)
    override = bool(m.opt.enableflags & mujoco.mjtEnableBit.mjENBL_OVERRIDE)
    integ = mujoco.mjtIntegrator(m.opt.integrator).name.replace("mjINT_", "")
    rep = Report("mujoco", src, {"dt": dt, "integrator": integ, "event_time": event_time})
    G = mujoco.mjtObj.mjOBJ_GEOM

    # ---- 스텝 수 --------------------------------------------------------------
    for rid in R.check_numbers("mujoco", dt, event_time=event_time):
        rep.add(rid, "option/timestep", t("steps", T=event_time, dt=dt, n=event_time / dt))

    # ---- 접촉 쌍 --------------------------------------------------------------
    pairs, _ = _pairs(m)
    if override:
        rep.notes.append(t("mj_override"))
        pairs = [(a, b, float(m.opt.o_solref[0]) if m.opt.o_solref[0] > 0 else float("nan"),
                  float(m.opt.o_friction[0])) for a, b, _, _ in pairs]

    tc_bad, tc_slow, n_direct = [], [], 0
    for a, b, tc, _ in pairs:
        if np.isnan(tc):
            n_direct += 1
            continue
        eff = R.mujoco_effective_timeconst(tc, dt, refsafe)
        fl = R.check_numbers("mujoco", dt, event_time=event_time, timeconst=tc, refsafe=refsafe)
        if "MJ001" in fl:
            tc_bad.append((eff, a, b))
        if "MJ004" in fl:
            tc_slow.append((eff, a, b))
    if tc_bad:
        eff, a, b = min(tc_bad)
        rep.add("MJ001", t("mj_pairs_stiff", n=len(tc_bad), a=_name(m, G, a), b=_name(m, G, b)),
                t("mj_tc_low", tc=eff, r=eff / dt, k=R.MJ_TC_MIN_DT))
    if tc_slow:
        eff, a, b = max(tc_slow)
        rep.add("MJ004", t("mj_pairs_soft", n=len(tc_slow), a=_name(m, G, a), b=_name(m, G, b)),
                t("mj_tc_high", tc=eff, r=eff / event_time, T=event_time, k=R.MJ_TC_MAX_EVENT_FRAC))
    if n_direct:
        rep.notes.append(t("mj_direct", n=n_direct))

    # ---- 마찰 결합 (MJ002) -----------------------------------------------------
    moving = m.body_weldid[m.geom_bodyid] != 0
    worst = {}
    for a, b, _, fr in pairs:
        for g, other in ((a, b), (b, a)):
            if moving[g] and not moving[other]:
                own = float(m.geom_friction[g, 0]) if not override else float(m.opt.o_friction[0])
                if fr > own * (1 + 1e-9) and (g not in worst or fr > worst[g][0]):
                    worst[g] = (fr, other, own)
    for g, (fr, other, own) in sorted(worst.items()):
        rep.add("MJ002", f"geom {_name(m, G, g)} ↔ {_name(m, G, other)}",
                t("mj_friction", own=own, fr=fr))

    # ---- 넘어짐 여유 (MJ003) ----------------------------------------------------
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)
    free_bodies = {int(m.jnt_bodyid[j]) for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE}
    collid = (m.geom_contype | m.geom_conaffinity) != 0
    for bid in sorted(free_bodies):
        gs = [g for g in range(m.ngeom) if m.geom_bodyid[g] == bid and collid[g]]
        if len(gs) != 1 or m.geom_type[gs[0]] != mujoco.mjtGeom.mjGEOM_BOX:
            continue
        g = gs[0]
        mus = [fr for a, b, _, fr in pairs if g in (a, b) and not moving[b if a == g else a]]
        if not mus:
            continue
        mu = max(mus)
        zcol = d.geom_xmat[g].reshape(3, 3)[2]  # 각 geom 축이 세계 z와 이루는 성분
        up = int(np.argmax(np.abs(zcol)))
        size = m.geom_size[g]
        h, base = size[up], min(size[i] for i in range(3) if i != up)
        tau = R.tipping_margin(mu, h, base)
        if "MJ003" in R.check_numbers("mujoco", dt, tau=tau):
            extra = t("tau_extra") if tau >= 1 else ""
            rep.add("MJ003", f"body {_name(m, mujoco.mjtObj.mjOBJ_BODY, bid)} (box geom {_name(m, G, g)})",
                    t("mj_tau", mu=mu, h=h, b=base, tau=tau, k=R.MJ_TAU_MAX, extra=extra))

    # ---- 적분기 ---------------------------------------------------------------
    if integ in ("EULER", "IMPLICIT", "IMPLICITFAST"):
        rep.add("INFO01", "option/integrator", t("first_order", name=integ))
    return rep
