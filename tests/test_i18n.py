from simdiff_check import audit_mujoco, i18n
from pathlib import Path

EX = Path(__file__).parents[1] / "examples"


def test_korean_output():
    try:
        i18n.set_lang("ko")
        txt = audit_mujoco(EX / "box_trap.xml", event_time=0.4).to_text()
        assert "근거" in txt and "요약" in txt and "MJ002" in txt
    finally:
        i18n.set_lang("en")
    txt = audit_mujoco(EX / "box_trap.xml", event_time=0.4).to_text()
    assert "evidence" in txt and "summary" in txt
