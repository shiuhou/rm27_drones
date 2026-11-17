"""Offline coverage checks for independently assessed evidence, with no scoring."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class EvidenceContext:
    run_id: str
    profile_id: str
    clock_id: str
    reset_epoch: int


@dataclass(frozen=True)
class EvidenceSpan:
    context: EvidenceContext
    source_id: str
    record_id: str
    start_s: float
    end_s: float


@dataclass(frozen=True)
class EvidenceRequirement:
    name: str
    source_id: str
    start_s: float
    end_s: float


@dataclass(frozen=True)
class EvidenceClaim:
    verdict: str
    spans: tuple[EvidenceSpan, ...] = ()


_PRECEDENCE = ('FAIL', 'BLOCKED', 'NOT_RUN', 'PASS')


def _interval(start, end):
    try:
        return all(type(v) in (int, float) and math.isfinite(v) for v in (start, end)) and start <= end
    except OverflowError:
        return False


def _id(value):
    return isinstance(value, str) and bool(value.strip())


def assess_coverage(context, requirements, claims):
    """Check the required windows against explicit producer-observed spans.

    This gate consumes verdicts independently assessed by the caller. A span
    describes certified coverage for one claim, not a pair of point samples.
    An empty event list or a rollout start cannot certify any window. PASS
    requires every named window to be covered without gaps or foreign scope.
    """
    if (not all(_id(key) for key in (context.run_id, context.profile_id, context.clock_id))
            or type(context.reset_epoch) is not int or context.reset_epoch < 0):
        raise ValueError('evidence requires a registered run, profile, clock and epoch')
    names = set()
    per_requirement = {}
    for requirement in requirements:
        name = requirement.name
        if (not _id(name) or name in names or not _id(requirement.source_id)
                or not _interval(requirement.start_s, requirement.end_s)):
            raise ValueError('invalid or duplicate evidence requirement')
        names.add(name)
        claim = claims.get(name)
        if claim is None:
            per_requirement[name] = ('BLOCKED', 'missing required evidence claim')
            continue
        if claim.verdict not in _PRECEDENCE:
            raise ValueError('unknown evidence verdict')
        spans = claim.spans
        if any(span.context != context or span.source_id != requirement.source_id
               or not _id(span.record_id) or not _interval(span.start_s, span.end_s) for span in spans):
            per_requirement[name] = ('BLOCKED', 'unregistered or malformed evidence span')
            continue
        intersecting = [span for span in spans if span.end_s >= requirement.start_s
                        and span.start_s <= requirement.end_s]
        if claim.verdict == 'NOT_RUN':
            per_requirement[name] = ('NOT_RUN', 'required check was not run')
        elif claim.verdict == 'FAIL' and intersecting:
            # A documented counterexample need not wait for the rest of a run.
            per_requirement[name] = ('FAIL', 'reported failure with in-window evidence')
        elif claim.verdict != 'PASS':
            per_requirement[name] = ('BLOCKED', 'required check lacks usable evidence')
        else:
            covered_end = requirement.start_s
            for span in sorted(intersecting, key=lambda span: span.start_s):
                if span.start_s > covered_end:
                    break
                covered_end = max(covered_end, span.end_s)
            per_requirement[name] = (
                ('PASS', 'required window covered') if intersecting and covered_end >= requirement.end_s
                else ('BLOCKED', 'required window only partially observed')
            )
    if not names or set(claims) - names:
        raise ValueError('no required checks or unexpected evidence claim')
    overall = min((verdict for verdict, _ in per_requirement.values()), key=_PRECEDENCE.index)
    return overall, per_requirement
