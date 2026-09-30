"""검증 V3: 자기 불일치 오차 추정과 '관측 차수' 신뢰도 판정.

A. 라이브러리 API(check_mujoco)로 SimDiff 2링크 팔 결과를 재현하는가? (실제 오차 / 추정 ≈ 1)
B. 상자 장면(V2의 dt, dt/2, dt/4 결과)에서
   - 추정치가 실제 오차의 0.5~2배 안에 드는 비율
   - 관측 차수 판정(|p_obs - 1| ≤ 0.5)이 '믿을 만한 추정'과 '못 믿을 추정'을 가르는가?
   - 설정 실수(바닥 마찰 기본값, 기본 감쇠, 양쪽 μ)가 있을 때 자기 불일치는 무엇을 보는가?
판정 기준 tol=0.5 는 데이터를 보기 전에 정했다 (조정하지 않음).

python validation/v3_selfcheck.py  →  validation/results/v3.json, v3_box.csv, v3_arm.csv
"""
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import spearmanr

from simdiff_check import check_mujoco

SIMDIFF = Path(os.environ.get("SIMDIFF_ROOT", Path(__file__).resolve().parents[2] / "simdiff"))
OUT = Path(__file__).parent / "results"
TOL = 0.5


def calib(actual, est):
    r = np.asarray(actual) / np.maximum(np.asarray(est), 1e-300)
    return {"n": int(len(r)), "median_actual_over_est": round(float(np.median(r)), 3),
            "share_within_0.5_2x": round(float(((r >= 0.5) & (r <= 2)).mean()), 3),
            "share_underestimate_over_2x": round(float((r > 2).mean()), 3),
            "spearman": round(float(spearmanr(est, actual).correlation), 3) if len(r) > 2 else None}


def part_a():
    sys.path.insert(0, str(SIMDIFF))
    import mujoco
    from simdiff.scenes import two_link_arm as arm
    p = arm.ArmParams()
    ics = pd.read_csv(SIMDIFF / "results/arm_ref_check.csv")
    rows = []
    for r in ics.itertuples():
        q0 = np.deg2rad([r.q1_deg, r.q2_deg])
        for dt in [0.0005, 0.001, 0.002]:
            m = mujoco.MjModel.from_xml_string(arm.mjcf(p, dt, "Euler"))
            for T in [0.5, 1.0, 2.0]:
                ref = arm.reference(p, q0, np.array([0.0, T]))[1, :2]

                def setup(m, d, q0=q0):
                    d.qpos[:2] = q0; d.qvel[:] = 0
                sc = check_mujoco(m, T, dt=dt, setup=setup, observe=lambda m, d: d.qpos[:2].copy(), levels=3, tol=TOL)
                rows.append(dict(ic=r.ic, dt=dt, T=T, actual=float(np.max(np.abs(sc.y[0] - ref))),
                                 est=sc.estimate, p_obs=sc.observed_order, reliable=sc.reliable))
    df = pd.DataFrame(rows)
    df.to_csv(OUT / "v3_arm.csv", index=False)
    return {"all": calib(df.actual, df.est), "reliable": calib(df.actual[df.reliable], df.est[df.reliable]),
            "unreliable_n": int((~df.reliable).sum()),
            "p_obs_median": round(float(df.p_obs.median()), 3)}


def part_b():
    out = {}
    for eng in ["mujoco", "pybullet"]:
        d = pd.read_csv(OUT / f"v2_{eng}.csv")
        d["rules"] = d.rules.fillna("")
        d["actual"] = (d.d1 - d.d_true).abs()
        d["est"] = 2 * (d.d1 - d.d2).abs()
        d1, d2 = (d.d1 - d.d2).abs(), (d.d2 - d.d4).abs()
        d["p_obs"] = np.log2(d1 / d2.replace(0, np.nan))
        d["reliable"] = (d.p_obs - 1).abs() <= TOL
        trap_cols = [c for c in d.columns if c.startswith("trap")]
        d["trap"] = d[trap_cols].any(axis=1)
        clean = ~d.trap
        res = {"clean_all": calib(d.actual[clean], d.est[clean]),
               "clean_reliable": calib(d.actual[clean & d.reliable], d.est[clean & d.reliable]),
               "clean_unreliable": calib(d.actual[clean & ~d.reliable], d.est[clean & ~d.reliable]),
               "share_reliable_clean": round(float(d.reliable[clean].mean()), 3),
               "linter_pass_reliable": calib(d.actual[~d.flagged & d.reliable], d.est[~d.flagged & d.reliable]),
               "linter_pass_share_reliable": round(float(d.reliable[~d.flagged].mean()), 3)}
        for t in trap_cols:
            g = d[t]
            res[t] = {"n": int(g.sum()), "median_actual_pct": round(float((d.actual / d.d_true * 100)[g].median()), 2),
                      "median_est_pct": round(float((d.est / d.d_true * 100)[g].median()), 3),
                      "median_actual_over_est": round(float((d.actual / d.est)[g].median()), 1)}
        # 실제 오차 1% 넘는 장면을 '추정 1% 초과 또는 못 믿음'으로 잡아내는 비율 (깨끗한 장면)
        over = (d.actual / d.d_true > 0.01)
        alarm = (d.est / d.d_true > 0.01) | ~d.reliable
        res["clean_recall_est_or_unreliable"] = round(float(alarm[clean & over].mean()), 3)
        res["clean_recall_est_only"] = round(float((d.est / d.d_true > 0.01)[clean & over].mean()), 3)
        res["clean_false_alarm_rate"] = round(float(alarm[clean & ~over].mean()), 3)
        out[eng] = res
        d.to_csv(OUT / f"v3_box_{eng}.csv", index=False)
    return out


def main():
    OUT.mkdir(exist_ok=True)
    res = {"tol": TOL, "A_arm": part_a(), "B_box": part_b()}
    (OUT / "v3.json").write_text(json.dumps(res, ensure_ascii=False, indent=2))
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
