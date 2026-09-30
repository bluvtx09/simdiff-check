"""출력 언어 (en 기본, ko). 환경 변수 SIMDIFF_CHECK_LANG 또는 CLI --lang 으로 바꾼다."""
import os

LANG = os.environ.get("SIMDIFF_CHECK_LANG", "en")


def set_lang(lang):
    global LANG
    if lang not in ("en", "ko"):
        raise ValueError("lang must be 'en' or 'ko'")
    LANG = lang


def t(key, **kw):
    return MSG[key][LANG].format(**kw)


MSG = {
    # ---- report ----
    "no_findings": {"en": "  no issues (within validated rule scope)", "ko": "  문제 없음 (검증된 규칙 범위 안)"},
    "where": {"en": "  where:    ", "ko": "  위치: "},
    "value": {"en": "  value:    ", "ko": "  값:   "},
    "evidence": {"en": "  evidence: ", "ko": "  근거: "},
    "fix": {"en": "  fix:      ", "ko": "  고침: "},
    "note": {"en": "note: ", "ko": "참고: "},
    "summary": {"en": "summary: error {e}, warning {w}, info {i}", "ko": "요약: error {e}, warning {w}, info {i}"},
    "scope": {"en": "validated on: pendulum, 2-link arm, box sliding to a stop. Impacts, grasping, rolling: not yet.",
              "ko": "검증 범위: 진자·2링크 팔·미끄러지다 멈추는 상자. 충돌·잡기·구르기는 미검증."},
    "risk_error": {"en": "stats: in validation, 80-92% of scenes with an error were off by more than 1%",
                   "ko": "참고 통계: error가 있는 장면은 검증에서 80~92%가 오차 1% 초과"},
    "risk_pass": {"en": "stats: in validation, 0 of 1,240 passing box scenes were off by more than 1% (max 0.9%)",
                  "ko": "참고 통계: 규칙 통과 장면은 검증(상자 1,240개)에서 오차 1% 초과 0건, 최대 0.9%"},
    "risk_warn": {"en": "stats: box scenes with {k} different warnings were off by more than 1% in {rate:.0f}% of "
                        "validation cases (one warning alone rarely matters; they compound)",
                  "ko": "참고 통계: 서로 다른 warning {k}개가 겹친 상자 장면은 검증에서 {rate:.0f}%가 오차 1% 초과 "
                        "(warning은 하나만으로는 드물게 문제, 겹칠수록 위험)"},
    # ---- common details ----
    "steps": {"en": "event {T} s / dt {dt} s = {n:.0f} steps", "ko": "사건 {T} s / dt {dt} s = {n:.0f} 스텝"},
    "tau_extra": {"en": " — τ ≥ 1: a rigid box would tip too", "ko": " — τ ≥ 1 이라 강체 이론상으로도 넘어짐"},
    "first_order": {"en": "{name} (first-order accurate)", "ko": "{name} (1차 정확도)"},
    # ---- mujoco ----
    "mj_override": {"en": "override flag is on: checked every contact with option o_solref/o_friction",
                    "ko": "override 플래그가 켜져 있어 모든 접촉에 option의 o_solref/o_friction 을 적용해 검사함"},
    "mj_pairs_stiff": {"en": "{n} contact pair(s) (stiffest: {a} ↔ {b})",
                       "ko": "접촉 쌍 {n}개 (가장 딱딱한 쌍: {a} ↔ {b})"},
    "mj_tc_low": {"en": "effective timeconst {tc:.4g} s = {r:.1f} x dt (need ≥ {k:.0f} x dt)",
                  "ko": "실효 timeconst {tc:.4g} s = {r:.1f} x dt (기준 ≥ {k:.0f} x dt)"},
    "mj_pairs_soft": {"en": "{n} contact pair(s) (softest: {a} ↔ {b})",
                      "ko": "접촉 쌍 {n}개 (가장 무른 쌍: {a} ↔ {b})"},
    "mj_tc_high": {"en": "effective timeconst {tc:.4g} s = {r:.2f} x event {T} s (need ≤ {k})",
                   "ko": "실효 timeconst {tc:.4g} s = 사건 {T} s의 {r:.2f}배 (기준 ≤ {k})"},
    "mj_direct": {"en": "{n} contact pair(s) use direct stiffness/damping solref (negative); skipped timeconst rules",
                  "ko": "solref를 직접 강성·감쇠(음수)로 지정한 접촉 쌍 {n}개는 timeconst 규칙에서 제외"},
    "mj_friction": {"en": "this body's μ={own:g} but the contact uses μ={fr:g} (the larger of the two)",
                    "ko": "이 물체 μ={own:g} 인데 실제 접촉 μ={fr:g} (두 값 중 큰 값)"},
    "mj_tau": {"en": "τ = μ {mu:g} x half-height {h:g} / half-width {b:g} = {tau:.2f} (need ≤ {k}){extra}",
               "ko": "τ = μ {mu:g} x 반높이 {h:g} / 반폭 {b:g} = {tau:.2f} (기준 ≤ {k}){extra}"},
    # ---- pybullet ----
    "pb_erp_default": {"en": "{v} (assumed default)", "ko": "{v} (기본값 가정)"},
    "pb_substeps": {"en": "numSubSteps={n} → checked with effective dt = fixedTimeStep/{n} = {dt:g} s",
                    "ko": "numSubSteps={n} → 실효 dt = fixedTimeStep/{n} = {dt:g} s 로 검사"},
    "pb_not_installed": {"en": "install() was not called before loading: cannot check link damping (PB001) or "
                               "contactERP settings",
                         "ko": "install()을 로드 전에 부르지 않아 링크 감쇠(PB001)와 contactERP 설정을 확인할 수 없음"},
    "pb_default": {"en": " (default)", "ko": " (기본값)"},
    "pb_links": {"en": "body {b} ({f}), {n} link(s)", "ko": "body {b} ({f}), 링크 {n}개"},
    "pb_inertia": {"en": "link '{nm}': URDF principal inertia {want} → engine uses {got}",
                   "ko": "링크 '{nm}': URDF 주관성 {want} → 엔진 사용값 {got}"},
    "pb_friction": {"en": "lateralFriction {a:g} x {b:g} → effective μ {e:.3g}",
                    "ko": "lateralFriction {a:g} x {b:g} → 실효 μ {e:.3g}"},
    "pb_friction_more": {"en": "PB003: showing the 20 lowest of {n} pairs", "ko": "PB003 해당 쌍 {n}개 중 실효 μ가 가장 낮은 20개만 표시"},
    "pb_tau": {"en": "τ = effective μ {mu:.3g} x half-height {h:g} / half-width {b:g} = {tau:.2f} (need ≤ {k}){extra}",
               "ko": "τ = 실효 μ {mu:.3g} x 반높이 {h:g} / 반폭 {b:g} = {tau:.2f} (기준 ≤ {k}){extra}"},
    "pb_integrator": {"en": "semi-implicit Euler", "ko": "반암시적 오일러"},
    # ---- selfcheck ----
    "sc_head": {"en": "self-disagreement error estimate (dt={dt:g}, theoretical order {p}): ±{e:.4g}",
                "ko": "자기 불일치 오차 추정 (dt={dt:g}, 이론 차수 {p}): ±{e:.4g}"},
    "sc_negligible": {"en": "\n  dt and dt/2 agree to round-off — numerical error negligible",
                      "ko": "\n  dt와 dt/2 결과 차이가 부동소수점 반올림 수준 — 수치 오차 무시 가능"},
    "sc_ok": {"en": "\n  observed order {p:.2f} → trustworthy", "ko": "\n  관측 차수 {p:.2f} → 믿을 만함"},
    "sc_bad": {"en": "\n  observed order {p:.2f} → NOT trustworthy (differs from theory by more than {tol}: "
                     "not in the asymptotic regime, or discontinuous behavior)",
               "ko": "\n  관측 차수 {p:.2f} → 믿기 어려움 (이론 차수와 {tol} 이상 차이: 점근 영역 밖이거나 불연속 거동)"},
    "sc_spread": {"en": "\n  values at dt, dt/2, dt/4 span {s:.4g} ({r:.1%} of the value) — use that as a rough error band",
                  "ko": "\n  dt, dt/2, dt/4 값의 폭 {s:.4g} (값의 {r:.1%}) — 이 폭을 대략적인 오차 범위로 쓸 것"},
    "sc_diverging": {"en": "\n  DIVERGING: the value keeps moving one way, faster, as dt shrinks (span {r:.0%} of the value); "
                           "it is an artifact of dt, not a physical prediction",
                     "ko": "\n  발산: dt를 줄일수록 값이 한 방향으로 더 크게 움직임 (값의 {r:.0%}) — 물리 예측이 아니라 dt의 부산물"},
    "sc_usable": {"en": "\n  spread across dt..dt/4 is {r:.1%} of the value (≤ 10%): fine to report with that band",
                  "ko": "\n  dt~dt/4 값의 폭이 값의 {r:.1%} (10% 이하): 이 폭을 붙여 보고해도 됨"},
    "sc_unusable": {"en": "\n  spread across dt..dt/4 is {r:.0%} of the value (> 10%): do not report this number",
                    "ko": "\n  dt~dt/4 값의 폭이 값의 {r:.0%} (10% 초과): 이 숫자는 보고하지 말 것"},
    "sc_cli": {"en": " (qpos after {T} s)", "ko": " (qpos, {T}초 뒤)"},
}
