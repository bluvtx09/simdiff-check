from simdiff_check import rules as R


def test_boundaries_pass():
    # 퍼징 규칙 v3의 경계는 통과여야 한다
    assert R.check_numbers("mujoco", 0.001, event_time=0.3, timeconst=0.01, tau=0.6) == []
    assert R.check_numbers("pybullet", 0.005, event_time=1.5, contact_erp=0.6, tau=0.9) == []


def test_step_rules():
    assert R.check_numbers("mujoco", 0.002, event_time=0.5) == ["EV001"]   # 250 스텝
    assert R.check_numbers("mujoco", 0.01, event_time=0.5) == ["EV002"]    # 50 스텝


def test_refsafe_clamp():
    # timeconst 0.001 < 2dt 이면 MuJoCo가 0.004로 올린다 → 여전히 10dt 미만
    assert R.mujoco_effective_timeconst(0.001, 0.002) == 0.004
    assert R.mujoco_effective_timeconst(0.001, 0.002, refsafe=False) == 0.001
    assert "MJ001" in R.check_numbers("mujoco", 0.002, timeconst=0.001)


def test_pybullet_limits():
    assert set(R.check_numbers("pybullet", 0.008, contact_erp=0.8, tau=1.0)) == {"PB004", "PB005", "PB006"}


def test_every_rule_has_evidence_and_fix():
    for r in R.RULES.values():
        assert r.evidence and r.fix and r.severity in {"error", "warning", "info"}
