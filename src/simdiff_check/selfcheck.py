"""자기 불일치: 정답 없이 시뮬레이션 수치 오차를 추정한다.

같은 시뮬레이션을 dt, dt/2 (선택: dt/4)로 돌려 결과 차이로 dt에서의 오차를 추정한다.
p차 정확도 방법이면  오차(dt) ≈ |y(dt) - y(dt/2)| · 2^p / (2^p - 1)   (리처드슨)
1차 방법(오일러 계열)은 2배, RK4는 16/15배.

SimDiff 근거: 2링크 팔에서 실제 오차 / 추정 비율 중앙값 1.00, 순위상관 0.999;
미끄러지는 상자에서 순위상관 0.88(MuJoCo)/0.67(PyBullet) — 엔진 간 비교(0.79/0.57)보다 나음.

levels=3 이면 dt/4 도 돌려 '관측 정확도 차수'를 계산한다. 관측 차수가 이론 차수와 멀면
(tol 기본 0.5) 결과가 아직 점근 영역이 아니거나 입력에 대해 불연속이라는 뜻이라
추정치를 믿지 말라고 표시한다.
"""
import copy
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class SelfCheck:
    dt: float
    order: int
    estimate: float              # dt에서의 오차 추정 (관측값 단위, 원소별 최대 절댓값)
    estimate_each: np.ndarray    # 원소별 추정
    y: list                      # 각 수준의 결과 [y(dt), y(dt/2), (y(dt/4))]
    observed_order: float = float("nan")
    reliable: bool = None        # levels=3 일 때만 판정
    tol: float = 0.5
    negligible: bool = False     # 차이가 부동소수점 반올림 수준
    spread: float = float("nan")  # levels=3: 세 수준 값의 최대-최소 (원소별 최대). 추정을 못 믿을 때의 경험적 폭
    diverging: bool = False      # levels=3: dt를 줄일수록 값이 한 방향으로 더 크게 움직이고, 폭이 rel_tol 초과
    rel_spread: float = float("nan")
    usable: bool = None          # levels=3: 세 값의 폭이 값의 rel_tol 이하 → 그 폭을 오차 범위로 붙여 보고 가능

    def to_text(self):
        from .i18n import t
        s = t("sc_head", dt=self.dt, p=self.order, e=self.estimate)
        if self.negligible:
            return s + t("sc_negligible")
        if self.reliable is not None:
            s += t("sc_ok", p=self.observed_order) if self.reliable else t("sc_bad", p=self.observed_order, tol=self.tol)
            if not self.reliable and not math.isnan(self.spread):
                s += t("sc_diverging" if self.diverging else "sc_spread", s=self.spread, r=self.rel_spread)
            if self.usable is not None:
                s += t("sc_usable" if self.usable else "sc_unusable", r=self.rel_spread)
        return s


def _as_arr(y):
    return np.atleast_1d(np.asarray(y, dtype=float))


def self_disagreement(run, dt, order=1, levels=2, tol=0.5, rel_tol=0.1):
    """run(dt) -> 관측값(스칼라 또는 배열). 같은 물리 시각의 값을 돌려줘야 한다.

    levels=3 일 때 판정:
      reliable  : 관측 차수가 이론 차수 ± tol 안 → 추정치를 믿어도 됨
      diverging : 차이가 dt를 줄일수록 커지고(관측 차수 < 0) 값이 한 방향으로 움직이며,
                  세 값의 폭이 값 크기의 rel_tol(기본 10%)을 넘음 → dt의 부산물, 보고하면 안 됨
      둘 다 아니면: 점근 영역 밖이지만 폭(spread)이 작으니 그 폭을 오차 범위로 쓴다
      usable    : 세 값의 폭이 값의 rel_tol 이하 → 값에 그 폭을 붙여 보고해도 됨
    rel_tol 10% 는 contact_gap 절차서 규칙 16(최대 접촉력 수렴 확인)의 기준을 따른 것.

    주의 (contact_gap 검증, 90개 경우): 접촉 최대력·충격량처럼 점근적으로 수렴하지 않는 지표에서는
    관측 차수가 우연히 이론 차수와 맞아 reliable=True 가 나오는 경우가 있었다(폭 60% 이상인데도).
    그런 지표는 reliable 대신 usable(폭 기준)로 판단할 것. usable은 dt/8까지 돌린 참조와 90/90 일치했다.
    """
    if levels not in (2, 3):
        raise ValueError("levels는 2 또는 3")
    ys = [_as_arr(run(dt / 2 ** k)) for k in range(levels)]
    fac = 2 ** order / (2 ** order - 1)
    each = np.abs(ys[0] - ys[1]) * fac
    res = SelfCheck(dt, order, float(np.max(each)), each, ys, tol=tol)
    scale = max(1.0, float(np.max(np.abs(ys[0]))))
    if float(np.max(np.abs(ys[0] - ys[1]))) <= 1e-10 * scale:
        res.negligible, res.reliable = True, True
        return res
    if levels == 3:
        d1 = float(np.max(np.abs(ys[0] - ys[1])))
        d2 = float(np.max(np.abs(ys[1] - ys[2])))
        if d2 == 0 and d1 == 0:
            res.observed_order, res.reliable = float("inf"), True   # 차이가 없음 = 정확
        elif d2 == 0 or d1 == 0:
            res.observed_order, res.reliable = float("nan"), False
        else:
            res.observed_order = math.log2(d1 / d2)
            res.reliable = abs(res.observed_order - order) <= tol
        Y = np.stack(ys)
        res.spread = float(np.max(Y.max(axis=0) - Y.min(axis=0)))
        mag = np.maximum(np.abs(Y).max(axis=0), 1e-300)
        res.rel_spread = float(np.max((Y.max(axis=0) - Y.min(axis=0)) / mag))
        step1, step2 = ys[1] - ys[0], ys[2] - ys[1]
        res.diverging = bool(d2 > d1 and np.all(step1 * step2 > 0) and res.rel_spread > rel_tol)
        res.usable = bool(res.rel_spread <= rel_tol)
    return res


# ---- MuJoCo 도우미 ------------------------------------------------------------

def mujoco_order(model):
    import mujoco
    return 4 if model.opt.integrator == mujoco.mjtIntegrator.mjINT_RK4 else 1


def mujoco_runner(model, duration, observe=None, setup=None, events=None):
    """MuJoCo 모델로 run(dt) 함수를 만든다.

    observe(m, d) -> 관측값 (기본: qpos 전체)
    setup(m, d)   -> 시작 전 상태 설정 (초기 자세, 속도 등)
    events: [(시각, fn(m, d)), ...] 특정 물리 시각에 실행할 동작 (예: 0.3초에 속도 주기).
            스텝 수가 아니라 시각으로 지정해야 dt를 바꿔도 같은 실험이 된다.
    """
    import mujoco
    observe = observe or (lambda m, d: d.qpos.copy())
    events = sorted(events or [], key=lambda e: e[0])

    def run(dt):
        m = copy.deepcopy(model)
        m.opt.timestep = dt
        d = mujoco.MjData(m)
        if setup:
            setup(m, d)
            mujoco.mj_forward(m, d)
        n = int(round(duration / dt))
        ev = [(int(round(t / dt)), f) for t, f in events]
        k = 0
        for i in range(n):
            while k < len(ev) and ev[k][0] == i:
                ev[k][1](m, d); k += 1
            mujoco.mj_step(m, d)
        return observe(m, d)
    return run


def check_mujoco(model, duration, dt=None, observe=None, setup=None, events=None, levels=3, tol=0.5):
    """MuJoCo 모델의 자기 불일치 한 번에 계산. dt 기본값은 모델의 timestep."""
    dt = dt or float(model.opt.timestep)
    run = mujoco_runner(model, duration, observe, setup, events)
    return self_disagreement(run, dt, order=mujoco_order(model), levels=levels, tol=tol)
