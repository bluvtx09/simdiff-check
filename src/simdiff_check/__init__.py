"""simdiff-check: 물리 시뮬레이션 설정 린터 + 정답 없는 오차 추정.

    from simdiff_check import audit_mujoco, self_disagreement
    import simdiff_check.pybullet as sdc_pb   # PyBullet: sdc_pb.install() 후 sdc_pb.audit()
"""
from .mujoco_audit import audit as audit_mujoco
from .report import Finding, Report
from .rules import RULES
from .selfcheck import check_mujoco, mujoco_runner, self_disagreement

__version__ = "0.1.0"
__all__ = ["audit_mujoco", "self_disagreement", "check_mujoco", "mujoco_runner", "Report", "Finding",
           "RULES", "__version__"]
