"""simdiff-check 규칙 목록.

모든 임계값은 SimDiff 실험(MuJoCo 3.14.0, PyBullet 2025-01 빌드)에서 나왔다.
검증 범위: 진자, 2링크 팔, 바닥 위를 미끄러지다 멈추는 상자.
충돌(반발), 여러 점 접촉(잡기), 구르는 물체에서는 아직 검증되지 않았다.

심각도
  error   : 이 설정 하나만으로 SimDiff에서 5% 이상 오차가 측정됨
  warning : 1~5% 오차가 측정됐거나, 장면에 따라 크게 틀릴 수 있음
  info    : 알아두면 좋은 사실 (오차 원인은 아님)
"""
from dataclasses import dataclass

# ---- 임계값 (SimDiff 퍼징 규칙 v3) --------------------------------------------
MIN_STEPS_PER_EVENT = 300          # 사건이 이 스텝 수 이상에 걸쳐 일어나야 함
HARD_MIN_STEPS_PER_EVENT = 100     # 이보다 적으면 error
MJ_TC_MIN_DT = 10.0                # MuJoCo: timeconst >= 10 x dt
MJ_TC_MAX_EVENT_FRAC = 0.2         # MuJoCo: timeconst <= 0.2 x 사건 지속시간
MJ_TAU_MAX = 0.6                   # MuJoCo 넘어짐 여유 상한
PB_TAU_MAX = 0.9                   # PyBullet 넘어짐 여유 상한
PB_DT_MAX = 0.005                  # PyBullet 타임스텝 상한 (s)
PB_CONTACT_ERP_MAX = 0.6           # PyBullet contactERP 상한
PB_DEFAULT_CONTACT_ERP = 0.2       # PyBullet 기본값 (getPhysicsEngineParameters가 돌려주지 않음)
PB_DEFAULT_DAMPING = 0.04          # PyBullet 링크 기본 감쇠

PREPRINT = "SimDiff preprint"

# 검증 V2 (처음 보는 무작위 상자 장면 엔진당 4,000개): error 없이 warning만 k개일 때 오차 1% 초과 비율
# error가 하나라도 있으면 MuJoCo 92%, PyBullet 80%가 1% 초과. warning 0개(통과)는 0%.
RISK_BY_WARNINGS = {"mujoco": {1: 0.047, 2: 0.070, 3: 0.521}, "pybullet": {1: 0.022, 2: 0.099, 3: 0.562}}


@dataclass(frozen=True)
class Rule:
    id: str
    engine: str
    severity: str
    en: tuple   # (title, evidence, fix)
    ko: tuple

    def _t(self, i):
        from .i18n import LANG
        return (self.ko if LANG == "ko" else self.en)[i]

    title = property(lambda self: self._t(0))
    evidence = property(lambda self: self._t(1))
    fix = property(lambda self: self._t(2))


P = PREPRINT
RULES = {r.id: r for r in [
    # ---------------- PyBullet ----------------
    Rule("PB001", "pybullet", "error",
         ("Link damping is not zero (PyBullet default 0.04)",
          f"an undamped pendulum loses 68% of its energy in 10 s (angle RMSE 1.1 rad); sliding-box stop distance "
          f"-4.6% mean, -11.5% worst ({P} §4.1); 813 validation scenes: median 3.8% (simdiff-check V2)",
          "p.changeDynamics(body, link, linearDamping=0, angularDamping=0) for every body"),
         ("링크 감쇠가 0이 아님 (PyBullet 기본값 0.04)",
          f"감쇠 없는 진자가 10초에 에너지 68% 손실, 각도 RMSE 1.1 rad; 상자 정지 거리 평균 -4.6%, 최대 -11.5% ({P} 4.1절); "
          f"검증 장면 813개 오차 중앙값 3.8% (simdiff-check V2)",
          "p.changeDynamics(body, link, linearDamping=0, angularDamping=0) 를 모든 몸체에 호출")),
    Rule("PB002", "pybullet", "error",
         ("URDF inertia is ignored (loaded without URDF_USE_INERTIA_FROM_FILE)",
          f"pendulum inertia silently became 6.7e-7 instead of 0.002 kg·m²; angle RMSE 0.1 rad over 10 s ({P} §4.1)",
          "p.loadURDF(path, flags=p.URDF_USE_INERTIA_FROM_FILE)"),
         ("URDF의 관성값이 무시됨 (URDF_USE_INERTIA_FROM_FILE 플래그 없음)",
          f"진자 관성 0.002 → 6.7e-7 kg·m² 로 바뀌어 각도 RMSE 0.1 rad (10초) ({P} 4.1절)",
          "p.loadURDF(path, flags=p.URDF_USE_INERTIA_FROM_FILE)")),
    Rule("PB003", "pybullet", "error",
         ("Both contacting bodies have friction < 1 (PyBullet multiplies them)",
          f"writing μ on both sides gives μ² → stop distance +66% to +262%; two defaults give 0.5 x 0.5 = 0.25 "
          f"({P} §4.1); 796 validation scenes: median error 114% (simdiff-check V2)",
          "set lateralFriction=1.0 on one side (usually the ground) and the intended μ on the other"),
         ("마찰계수를 두 물체 모두에 1 미만으로 설정 (PyBullet은 두 값을 곱함)",
          f"양쪽에 μ를 적으면 실효 마찰이 μ² → 상자 정지 거리 +66~+262%; 기본값끼리는 0.5 x 0.5 = 0.25 ({P} 4.1절); "
          f"검증 장면 796개 오차 중앙값 114% (simdiff-check V2)",
          "한쪽(보통 바닥)을 lateralFriction=1.0 으로 두고 다른 쪽에 원하는 μ")),
    Rule("PB004", "pybullet", "warning",
         (f"Time step above {PB_DT_MAX * 1000:.0f} ms",
          f"fuzzing under rule set v1_mu found 6.0% error at dt 9.5 ms ({P} §4.6)",
          f"fixedTimeStep <= {PB_DT_MAX}"),
         (f"타임스텝이 {PB_DT_MAX * 1000:.0f} ms보다 큼",
          f"퍼징 규칙 v1_mu 최악 6.0% (dt 9.5 ms) ({P} 4.6절)",
          f"fixedTimeStep <= {PB_DT_MAX}")),
    Rule("PB005", "pybullet", "warning",
         (f"contactERP above {PB_CONTACT_ERP_MAX}",
          f"fuzzing found 6.0% error at contactERP 0.72; Sobol total index ST 0.81 ({P} §4.4, §4.6)",
          f"setPhysicsEngineParameter(contactERP<={PB_CONTACT_ERP_MAX})"),
         (f"contactERP가 {PB_CONTACT_ERP_MAX}보다 큼",
          f"퍼징 규칙 v1_mu 최악 6.0% (contactERP 0.72); Sobol 전체 영향 ST 0.81 ({P} 4.4, 4.6절)",
          f"setPhysicsEngineParameter(contactERP<={PB_CONTACT_ERP_MAX})")),
    Rule("PB006", "pybullet", "warning",
         (f"Tipping margin τ above {PB_TAU_MAX} (may tip while sliding)",
          f"never tipped below τ 0.9; 89% tipped at τ ≥ 1 ({P} §4.6)",
          "if this box slides: lower μ or the center of mass (τ = μ·half-height/half-width)"),
         (f"넘어짐 여유 τ가 {PB_TAU_MAX}보다 큼 (미끄러질 때 넘어질 수 있음)",
          f"τ < 0.9 에서는 한 번도 안 넘어짐, τ ≥ 1 에서 89% 넘어짐 ({P} 4.6절)",
          "미끄러지는 상자라면 μ를 낮추거나 무게중심을 낮추기 (τ = μ·반높이/반폭)")),
    # ---------------- MuJoCo ----------------
    Rule("MJ001", "mujoco", "warning",
         (f"Contact timeconst below {MJ_TC_MIN_DT:.0f} x dt (contact too stiff)",
          f"fuzzing under rule set v2 found 7.7% error at timeconst ≈ 3.4 dt with impratio 7.3 ({P} §4.6)",
          f"solref timeconst >= {MJ_TC_MIN_DT:.0f} x timestep (default 0.02 / 0.002 sits exactly on the limit)"),
         (f"접촉 timeconst가 {MJ_TC_MIN_DT:.0f} x dt보다 작음 (너무 딱딱한 접촉)",
          f"퍼징 규칙 v2 최악 7.7% (timeconst ≈ 3.4 dt, impratio 7.3) ({P} 4.6절)",
          f"solref의 timeconst >= {MJ_TC_MIN_DT:.0f} x timestep (기본값 0.02 / 0.002 는 딱 경계)")),
    Rule("MJ002", "mujoco", "error",
         ("A moving body has lower friction than what it touches (MuJoCo uses the larger value)",
          "box μ=0.4 on a default floor (μ=1.0) → effective μ 1.00; 809 validation scenes with this mistake: "
          "median stop-distance error 55% (simdiff-check V2)",
          "lower the friction of the floor (and everything it touches) too, or use geom priority"),
         ("움직이는 물체의 마찰계수가 바닥보다 낮음 (MuJoCo는 두 값 중 큰 값을 씀)",
          "상자 μ=0.4, 바닥 기본값 1.0 → 실효 μ 1.00; 이 실수가 든 검증 장면 809개의 정지 거리 오차 중앙값 55% "
          "(simdiff-check V2)",
          "바닥(과 닿는 모든 물체)의 friction도 같이 낮추거나, geom priority로 우선순위 지정")),
    Rule("MJ003", "mujoco", "warning",
         (f"Tipping margin τ above {MJ_TAU_MAX} (MuJoCo tips boxes a rigid body would not)",
          f"39-53% tipped at τ 0.8-0.9, stop-distance error up to 28%; worst fuzzing case 25.8% ({P} §4.6)",
          "if this box slides: lower μ or the center of mass, or increase timeconst"),
         (f"넘어짐 여유 τ가 {MJ_TAU_MAX}보다 큼 (강체 이론상 안 넘어질 상자도 넘어짐)",
          f"τ 0.8~0.9 에서 39~53% 넘어짐, 정지 거리 오차 최대 28%; 퍼징 규칙 v1 최악 25.8% ({P} 4.6절)",
          "미끄러지는 상자라면 μ를 낮추거나 무게중심을 낮추기, 또는 timeconst를 키우기")),
    Rule("MJ004", "mujoco", "warning",
         (f"Contact timeconst above {MJ_TC_MAX_EVENT_FRAC} x event duration",
          f"fuzzing found 9.7% error when the box stopped in 0.085 s with timeconst 0.05 s ({P} §4.6)",
          f"timeconst <= {MJ_TC_MAX_EVENT_FRAC} x event duration (reduce it together with dt)"),
         (f"접촉 timeconst가 사건 지속시간의 {MJ_TC_MAX_EVENT_FRAC}배보다 큼",
          f"퍼징 규칙 v1_mu 최악 9.7% (정지까지 0.085초, timeconst 0.05초) ({P} 4.6절)",
          f"timeconst <= {MJ_TC_MAX_EVENT_FRAC} x 사건 지속시간, 또는 dt와 함께 줄이기")),
    # ---------------- 공통 ----------------
    Rule("EV001", "any", "warning",
         (f"Event shorter than {MIN_STEPS_PER_EVENT} steps",
          f"steps-to-stop explains 68% (MuJoCo) / 48% (PyBullet) of error variance; share under 1% error: "
          f"98% / 99.5% at ≥300 steps, 92% / 96% at 100-300 ({P} §4.4)",
          f"dt <= event duration / {MIN_STEPS_PER_EVENT}"),
         (f"사건이 {MIN_STEPS_PER_EVENT} 스텝보다 짧음",
          f"정지까지 스텝 수가 오차 분산의 68%(MuJoCo)/48%(PyBullet) 설명; 300스텝 이상에서 1% 미만 비율 "
          f"98%/99.5%, 100~300스텝 92%/96% ({P} 4.4절)",
          f"dt <= 사건 지속시간 / {MIN_STEPS_PER_EVENT}")),
    Rule("EV002", "any", "error",
         (f"Event shorter than {HARD_MIN_STEPS_PER_EVENT} steps",
          f"share under 1% error: 26% (MuJoCo) / 73% (PyBullet) at 50-100 steps, 2% / 21% below 20 ({P} §4.4)",
          f"dt <= event duration / {MIN_STEPS_PER_EVENT}"),
         (f"사건이 {HARD_MIN_STEPS_PER_EVENT} 스텝보다 짧음",
          f"50~100스텝에서 1% 미만 비율 26%(MuJoCo)/73%(PyBullet), 20스텝 미만 2%/21% ({P} 4.4절)",
          f"dt <= 사건 지속시간 / {MIN_STEPS_PER_EVENT}")),
    Rule("INFO01", "any", "info",
         ("First-order integrator — error scales with dt; short prediction horizon in chaotic systems",
          f"2-link arm: semi-implicit Euler stays within 0.1 rad for 1.3 s (dt 1 ms) vs 8.8 s for RK4; "
          f"MuJoCo and PyBullet agreed with each other for 17 s while both were wrong ({P} §4.2-4.3)",
          "use RK4 for long trajectories, or estimate the error with simdiff_check.self_disagreement"),
         ("1차 적분기 — 오차가 dt에 비례, 혼돈계에서는 예측 가능 시간이 짧음",
          f"2링크 팔에서 반암시적 오일러의 예측 가능 시간 1.3초(dt 1 ms) vs RK4 8.8초; "
          f"두 엔진은 서로 17초간 일치하면서 함께 틀림 ({P} 4.2, 4.3절)",
          "긴 궤적을 비교해야 한다면 RK4, 아니면 simdiff_check.self_disagreement 로 오차 추정")),
]}


# ---- 순수 판정 함수 (엔진 독립, 회귀 테스트에서 직접 사용) -----------------------

def tipping_margin(mu, half_height, half_base):
    """τ = μ · (높이/2) / (바닥 폭/2). 바닥 폭은 미끄러지는 방향의 반폭 중 작은 값을 쓰면 보수적."""
    return mu * half_height / half_base


def mujoco_effective_timeconst(timeconst, dt, refsafe=True):
    """MuJoCo는 refsafe가 켜져 있으면(기본) timeconst를 2·dt 이상으로 올린다."""
    return max(timeconst, 2 * dt) if refsafe else timeconst


def check_numbers(engine, dt, event_time=None, timeconst=None, tau=None, contact_erp=None, refsafe=True):
    """숫자만으로 판정 가능한 규칙들. [rule_id, ...] 를 돌려준다 (위반한 것만)."""
    out = []
    if event_time is not None:
        n = event_time / dt * (1 + 1e-9)   # 경계(정확히 300스텝)의 부동소수점 반올림 허용
        if n < HARD_MIN_STEPS_PER_EVENT:
            out.append("EV002")
        elif n < MIN_STEPS_PER_EVENT:
            out.append("EV001")
    if engine == "mujoco":
        if timeconst is not None:
            tc = mujoco_effective_timeconst(timeconst, dt, refsafe)
            if tc < MJ_TC_MIN_DT * dt * (1 - 1e-9):
                out.append("MJ001")
            if event_time is not None and tc > MJ_TC_MAX_EVENT_FRAC * event_time * (1 + 1e-9):
                out.append("MJ004")
        if tau is not None and tau > MJ_TAU_MAX * (1 + 1e-9):
            out.append("MJ003")
    elif engine == "pybullet":
        if dt > PB_DT_MAX * (1 + 1e-9):
            out.append("PB004")
        if contact_erp is not None and contact_erp > PB_CONTACT_ERP_MAX * (1 + 1e-9):
            out.append("PB005")
        if tau is not None and tau > PB_TAU_MAX * (1 + 1e-9):
            out.append("PB006")
    return out
