"""simdiff-check: lint physics-simulation settings against measured failure rules.

  simdiff-check rules                                   list the rules
  simdiff-check mujoco model.xml [--event-time 0.4]     check a MuJoCo model
         [--selfcheck 2.0]                              + run 2 s and estimate numerical error (dt vs dt/2 vs dt/4)
  simdiff-check pybullet script.py [--event-time 0.4]   run a PyBullet script with the recorder installed and
                                                        check each client right before it disconnects
options: --json, --strict (warnings fail too), --quiet (hide evidence/fix lines), --lang en|ko
--event-time: duration in seconds of the event you care about (e.g. a box sliding to a stop, one contact)
exit code: 0 if clean, 1 if any error (with --strict: any warning)
"""
import argparse
import runpy
import sys

from .i18n import set_lang, t
from .report import Report
from .rules import RULES


def _emit(rep: Report, args):
    print(rep.to_json() if args.json else rep.to_text(verbose=not args.quiet))
    return rep.exit_code(args.strict)


def cmd_rules(args):
    for r in RULES.values():
        print(f"{r.id:7s} {r.severity:8s} [{r.engine}] {r.title}")
        if not args.quiet:
            print(f"        {t('evidence').strip()} {r.evidence}\n        {t('fix').strip()} {r.fix}")
    return 0


def cmd_mujoco(args):
    import mujoco
    from .mujoco_audit import audit
    from .selfcheck import check_mujoco
    m = mujoco.MjModel.from_xml_path(args.model)
    rep = audit(m, event_time=args.event_time)
    rep.source = args.model
    if args.selfcheck:
        sc = check_mujoco(m, args.selfcheck)
        rep.context["selfcheck"] = {"duration_s": args.selfcheck, "qpos_error_estimate": sc.estimate,
                                    "observed_order": sc.observed_order, "reliable": sc.reliable}
        rep.notes.append(sc.to_text() + t("sc_cli", T=args.selfcheck))
    return _emit(rep, args)


def cmd_pybullet(args):
    import pybullet as p
    from . import pybullet_audit as pa
    pa.install(p)
    reports = []
    orig_disc = pa._ORIG.get("disconnect") or p.disconnect

    def disconnect(*a, **kw):
        cid = kw.get("physicsClientId", a[0] if a else 0)
        try:
            reports.append(pa.audit(cid, event_time=args.event_time, p=p))
        except Exception as e:  # 이미 끊긴 클라이언트 등
            print(f"simdiff-check: audit failed ({e})", file=sys.stderr)
        return orig_disc(*a, **kw)
    p.disconnect = disconnect
    sys.argv = [args.script] + args.script_args
    try:
        runpy.run_path(args.script, run_name="__main__")
    finally:
        p.disconnect = orig_disc
    if not reports:
        try:
            reports.append(pa.audit(0, event_time=args.event_time, p=p))
        except Exception as e:
            print(f"simdiff-check: script finished but no physics client was left to check ({e})", file=sys.stderr)
            return 2
    code = 0
    for rep in reports:
        rep.source = f"{args.script} ({rep.source})"
        code = max(code, _emit(rep, args))
    return code


def main(argv=None):
    ap = argparse.ArgumentParser(prog="simdiff-check", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--json", action="store_true")
    common.add_argument("--strict", action="store_true")
    common.add_argument("--quiet", action="store_true")
    common.add_argument("--lang", choices=["en", "ko"], default=None)
    s = sub.add_parser("rules", parents=[common]); s.set_defaults(fn=cmd_rules)
    s = sub.add_parser("mujoco", parents=[common]); s.set_defaults(fn=cmd_mujoco)
    s.add_argument("model"); s.add_argument("--event-time", type=float)
    s.add_argument("--selfcheck", type=float, metavar="SECONDS")
    s = sub.add_parser("pybullet", parents=[common]); s.set_defaults(fn=cmd_pybullet)
    s.add_argument("script"); s.add_argument("--event-time", type=float)
    args, rest = ap.parse_known_args(argv)
    if rest and args.cmd != "pybullet":
        ap.error(f"unrecognized arguments: {' '.join(rest)}")
    args.script_args = rest
    if args.lang:
        set_lang(args.lang)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
