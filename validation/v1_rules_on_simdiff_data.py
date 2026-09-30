"""검증 V1: 린터 판정 함수를 SimDiff가 이미 돌린 실행 기록에 그대로 적용한다.

질문 1 (구현 일치): 퍼징 규칙 v3 안에서 탐색한 조건을 린터가 전부 '통과'로 판정하는가?
질문 2 (판정 품질): Sobol 무작위 표본(61,440회)에서 린터 통과/위반 그룹의 실제 오차는 어떻게 다른가?

새 시뮬레이션 없이, 린터의 rules.check_numbers 만으로 판정한다.
python validation/v1_rules_on_simdiff_data.py  →  validation/results/v1.json
"""
import json
import os
from pathlib import Path

import numpy as np
import pandas as pd

from simdiff_check import rules as R

SIMDIFF = Path(os.environ.get("SIMDIFF_ROOT", Path(__file__).resolve().parents[2] / "simdiff"))
OUT = Path(__file__).parent / "results"
G = 9.81


def verdict(engine, dt, mu, v0, tc=None, erp=None):
    """정육면체 상자(τ = μ), 사건 = 정지까지. 위반 규칙 id 목록."""
    t_stop = v0 / (mu * G)
    kw = dict(event_time=t_stop, tau=mu)
    if engine == "mujoco":
        kw["timeconst"] = tc
    else:
        kw["contact_erp"] = erp
    return R.check_numbers(engine, dt, **kw)


def stats(err):
    err = np.abs(np.asarray(err, dtype=float))
    if len(err) == 0:
        return {"n": 0}
    return {"n": int(len(err)), "share_over_1pct": round(float((err > 1).mean()), 4),
            "share_over_2pct": round(float((err > 2).mean()), 4),
            "median_pct": round(float(np.median(err)), 3), "p99_pct": round(float(np.quantile(err, 0.99)), 3),
            "max_pct": round(float(err.max()), 2)}


def main():
    OUT.mkdir(exist_ok=True)
    res = {}

    # ---- 질문 1: 퍼징 v3 조건은 모두 통과여야 한다 -----------------------------------
    # 기록 파일의 변수 열은 [0,1] 정규화 값이라 SimDiff의 fuzz.decode로 실제 물리값으로 바꾼다
    import sys
    sys.path.insert(0, str(SIMDIFF))
    import fuzz
    h = pd.read_csv(SIMDIFF / "results/fuzz_history_v3.csv", float_precision="round_trip")
    q1 = {}
    for eng in ["mujoco", "pybullet"]:
        d = h[h.target == eng]
        names = fuzz.USES[eng]
        v = []
        for z in d[names].to_numpy(dtype=float):
            c, dt, x = fuzz.decode(z, names, "v3")
            tc = 10 ** x["mj_log10_timeconst"] if eng == "mujoco" else None
            erp = x["pb_contact_erp"] if eng == "pybullet" else None
            v.append(verdict(eng, dt, c.mu, c.v0, tc=tc, erp=erp))
        flagged = [x for x in v if x]
        q1[eng] = {"n": len(v), "flagged": len(flagged),
                   "flag_kinds": sorted({k for x in flagged for k in x}), "error_stats": stats(d.value)}
    res["q1_fuzz_v3_all_pass"] = q1

    # ---- 질문 2: Sobol 표본 ---------------------------------------------------
    q2 = {}
    for eng in ["mujoco", "pybullet"]:
        z = np.load(SIMDIFF / f"results/sobol_samples_{eng}.npz")
        X, Y = z["X"], z["Y"]
        rows = []
        for x, y in zip(X, Y):
            dt, mu, v0 = 10 ** x[0], x[1], x[2]
            if eng == "mujoco":
                v = verdict(eng, dt, mu, v0, tc=10 ** x[3])
            else:
                v = verdict(eng, dt, mu, v0, erp=x[4])
            rows.append((v, abs(y)))
        ok = [e for v, e in rows if not v]
        bad = [e for v, e in rows if v]
        by_rule = {}
        for rid in sorted({k for v, _ in rows for k in v}):
            by_rule[rid] = stats([e for v, e in rows if rid in v])
        over = np.array([e > 1 for _, e in rows])
        flag = np.array([bool(v) for v, _ in rows])
        q2[eng] = {"pass": stats(ok), "fail": stats(bad), "by_rule": by_rule,
                   # 1% 넘는 실행 중 린터가 잡은 비율 (재현율), 린터가 잡은 것 중 실제로 1% 넘는 비율 (정밀도)
                   "recall_of_over_1pct": round(float(flag[over].mean()), 4),
                   "precision_over_1pct": round(float(over[flag].mean()), 4),
                   "pass_rate": round(float((~flag).mean()), 4)}
    res["q2_sobol"] = q2
    (OUT / "v1.json").write_text(json.dumps(res, ensure_ascii=False, indent=2))
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
