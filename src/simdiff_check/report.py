"""검사 결과 모음과 출력 형식."""
import json
from dataclasses import asdict, dataclass, field

from .i18n import t
from .rules import RISK_BY_WARNINGS, RULES

_ORDER = {"error": 0, "warning": 1, "info": 2}


@dataclass
class Finding:
    rule: str
    where: str
    detail: str

    @property
    def severity(self):
        return RULES[self.rule].severity


@dataclass
class Report:
    engine: str
    source: str
    context: dict = field(default_factory=dict)
    findings: list = field(default_factory=list)
    notes: list = field(default_factory=list)

    def add(self, rule, where, detail):
        self.findings.append(Finding(rule, where, detail))

    def count(self, severity):
        return sum(f.severity == severity for f in self.findings)

    @property
    def rule_ids(self):
        return sorted({f.rule for f in self.findings})

    def ok(self, strict=False):
        bad = {"error", "warning"} if strict else {"error"}
        return not any(f.severity in bad for f in self.findings)

    def exit_code(self, strict=False):
        return 0 if self.ok(strict) else 1

    def risk_note(self):
        """검증 V2 기준, 이 판정 조합의 장면에서 오차 1% 초과가 얼마나 흔했는지."""
        table = RISK_BY_WARNINGS.get(self.engine)
        if not table or self.count("error"):
            return t("risk_error") if self.count("error") else None
        k = len({f.rule for f in self.findings if f.severity == "warning"})
        if k == 0:
            return t("risk_pass")
        rate = table[min(k, 3)]
        return t("risk_warn", k=k, rate=rate * 100)

    def to_dict(self):
        return {"engine": self.engine, "source": self.source, "context": self.context,
                "summary": {s: self.count(s) for s in ("error", "warning", "info")},
                "findings": [dict(asdict(f), severity=f.severity, title=RULES[f.rule].title,
                                  evidence=RULES[f.rule].evidence, fix=RULES[f.rule].fix)
                             for f in self._sorted()],
                "notes": self.notes, "risk_note": self.risk_note()}

    def to_json(self):
        return json.dumps(self.to_dict(), ensure_ascii=False, indent=2)

    def _sorted(self):
        return sorted(self.findings, key=lambda f: (_ORDER[f.severity], f.rule, f.where))

    def to_text(self, verbose=True):
        lines = [f"simdiff-check · {self.engine} · {self.source}"]
        if self.context:
            lines.append("  " + ", ".join(f"{k}={v}" for k, v in self.context.items()))
        if not self.findings:
            lines.append(t("no_findings"))
        for f in self._sorted():
            r = RULES[f.rule]
            lines.append(f"\n[{f.severity.upper()}] {f.rule} {r.title}")
            lines.append(t("where") + f.where)
            lines.append(t("value") + f.detail)
            if verbose:
                lines.append(t("evidence") + r.evidence)
                lines.append(t("fix") + r.fix)
        for n in self.notes:
            lines.append("\n" + t("note") + n)
        lines.append("\n" + t("summary", e=self.count("error"), w=self.count("warning"), i=self.count("info")))
        risk = self.risk_note()
        if risk:
            lines.append(risk)
        lines.append(t("scope"))
        return "\n".join(lines)
