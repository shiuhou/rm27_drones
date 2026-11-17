"""Negative and positive coverage cases with no flight scorer involved."""

from dataclasses import replace
import unittest

from rm27_exchange.evidence import (
    EvidenceClaim, EvidenceContext, EvidenceRequirement, EvidenceSpan, assess_coverage,
)


class EvidenceCoverageTests(unittest.TestCase):
    context = EvidenceContext('run-1', 'acceptance-profile-1', 'replay-clock', 2)
    requirements = tuple(EvidenceRequirement(name, source, 1., 3.) for name, source in (
        ('observations', 'sensor'), ('events', 'event-monitor'), ('rollout', 'recorder'),
    ))

    def span(self, name, start, end, record_id='record'):
        req = next(req for req in self.requirements if req.name == name)
        return EvidenceSpan(self.context, req.source_id, record_id, start, end)

    def complete(self):
        return {req.name: EvidenceClaim('PASS', (self.span(req.name, 1., 3.),))
                for req in self.requirements}

    def assess(self, claims):
        return assess_coverage(self.context, self.requirements, claims)

    def test_complete_claims_join_only_touching_or_overlapping_intervals(self):
        claims = self.complete()
        # Shuffled, overlapping and exactly touching all represent a covered window.
        claims['observations'] = EvidenceClaim('PASS', (
            self.span('observations', 2.5, 3., 'c'), self.span('observations', 1.6, 2.5, 'b'),
            self.span('observations', .8, 1.8, 'a'),
        ))
        overall, breakdown = self.assess(claims)
        self.assertEqual(overall, 'PASS')
        self.assertEqual({verdict for verdict, _ in breakdown.values()}, {'PASS'})

    def test_partial_observations_events_and_rollouts_never_pass(self):
        for name, intervals in (
            ('observations', [(1., 1.), (2., 2.), (3., 3.)]),
            ('observations', [(1., 1.9), (2.1, 3.)]),
            ('events', []),  # Empty list is not proof that the monitor ran.
            ('events', [(1.1, 3.)]),
            ('rollout', [(1., 2.5)]),
        ):
            claims = self.complete()
            claims[name] = EvidenceClaim('PASS', tuple(self.span(name, start, end) for start, end in intervals))
            with self.subTest(name=name, intervals=intervals):
                overall, breakdown = self.assess(claims)
                self.assertEqual(overall, 'BLOCKED')
                self.assertEqual(breakdown[name][0], 'BLOCKED')

    def test_foreign_lineage_cannot_fill_a_gap(self):
        span = self.span('rollout', 1., 3.)
        for foreign in (replace(span, source_id='other'), replace(span, record_id=''),
                        replace(span, context=replace(self.context, run_id='other')),
                        replace(span, context=replace(self.context, profile_id='other')),
                        replace(span, context=replace(self.context, clock_id='other')),
                        replace(span, context=replace(self.context, reset_epoch=1)),
                        replace(span, end_s=float('inf'))):
            with self.subTest(foreign=foreign):
                self.assertEqual(self.assess(self.complete() | {'rollout': EvidenceClaim('PASS', (foreign,))})[0],
                                 'BLOCKED')

    def test_missing_not_run_and_observed_failure_stay_distinct(self):
        claims = self.complete()
        claims['rollout'] = EvidenceClaim('NOT_RUN')
        self.assertEqual(self.assess(claims)[0], 'NOT_RUN')
        del claims['events']
        self.assertEqual(self.assess(claims)[0], 'BLOCKED')
        claims['observations'] = EvidenceClaim('FAIL')
        self.assertEqual(self.assess(claims)[1]['observations'][0], 'BLOCKED')
        claims['observations'] = EvidenceClaim('FAIL', (self.span('observations', 2., 2.),))
        self.assertEqual(self.assess(claims)[0], 'FAIL')
