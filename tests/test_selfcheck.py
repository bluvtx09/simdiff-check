import math

import numpy as np

from simdiff_check import self_disagreement


def euler_decay(dt, T=1.0):
    y = 1.0
    for _ in range(int(round(T / dt))):
        y += dt * (-y)
    return y


def rk4_decay(dt, T=1.0):
    y, f = 1.0, (lambda y: -y)
    for _ in range(int(round(T / dt))):
        k1 = f(y); k2 = f(y + dt / 2 * k1); k3 = f(y + dt / 2 * k2); k4 = f(y + dt * k3)
        y += dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return y


def test_first_order_estimate_matches_truth():
    sc = self_disagreement(euler_decay, 0.01, order=1, levels=3)
    actual = abs(euler_decay(0.01) - math.exp(-1))
    assert 0.9 < actual / sc.estimate < 1.1 and sc.reliable and abs(sc.observed_order - 1) < 0.05


def test_fourth_order():
    sc = self_disagreement(rk4_decay, 0.1, order=4, levels=3)
    actual = abs(rk4_decay(0.1) - math.exp(-1))
    assert 0.8 < actual / sc.estimate < 1.25 and sc.reliable


def test_wrong_order_is_flagged():
    sc = self_disagreement(rk4_decay, 0.1, order=1, levels=3)   # 차수를 잘못 알려주면
    assert not sc.reliable


def test_discontinuous_is_flagged():
    run = lambda dt: float(np.floor(1.0 / dt) % 3)   # dt에 따라 불연속으로 튀는 결과
    sc = self_disagreement(run, 0.013, levels=3)
    assert not sc.reliable


def test_negligible():
    sc = self_disagreement(lambda dt: 1.0, 0.01, levels=3)
    assert sc.negligible and sc.reliable and sc.estimate == 0


def test_diverging_peak_like_contact_force():
    # 충격량/dt 처럼 dt를 줄일수록 커지는 값 (contact_gap 규칙 16의 최대 접촉력)
    sc = self_disagreement(lambda dt: 1.0 / dt, 0.0004, levels=3)
    assert not sc.reliable and sc.diverging and "DIVERGING" in sc.to_text()


def test_small_noisy_values_get_a_band():
    vals = {0.0004: 0.9794, 0.0002: 0.9693, 0.0001: 0.9793}
    sc = self_disagreement(lambda dt: vals[round(dt, 7)], 0.0004, order=4, levels=3)
    assert not sc.reliable and not sc.diverging and abs(sc.spread - 0.0101) < 1e-9


def test_usable_is_spread_based():
    # 관측 차수는 우연히 1에 가깝지만 폭이 큰 값 (contact_gap PyBullet v0=9 최대력과 같은 모양)
    vals = {0.0004: 976.0, 0.0002: 1417.0, 0.0001: 3000.0}
    sc = self_disagreement(lambda dt: vals[round(dt, 7)], 0.0004, levels=3)
    assert not sc.usable and sc.rel_spread > 0.5
    sc = self_disagreement(lambda dt: {0.0004: 0.801, 0.0002: 0.812, 0.0001: 0.833}[round(dt, 7)], 0.0004, levels=3)
    assert sc.usable
