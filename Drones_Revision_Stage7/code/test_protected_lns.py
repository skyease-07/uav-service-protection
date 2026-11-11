import unittest
from dataclasses import replace
from unittest.mock import patch
import final_model as fm
import lns_resource_baseline as base
import protected_lns as plns
from indexed_scheduler import IntervalIndex


class ProtectionTests(unittest.TestCase):
    def test_capped_response_matches_exhaustive_grid(self):
        fs,net,types=fm.flights(4,941,4),replace(fm.network(),max_delay_sec=80),fm.types()
        templates=base.make_templates(fs,net,types,fm.builder()); index=IntervalIndex(); occupied=[]
        for f in fs:
            for cap in (0,7,20,79,80):
                got=base.best_response(templates[f.flight_id],index,net,max_delay=cap)
                feasible=[]
                for choice in fm.ref.enumerate_structural_choices(f,net):
                    for delay in range(0,min(cap,80)+1,2):
                        candidate=fm.build_candidate(f,choice,delay,net,types)
                        if all(candidate.resources.isdisjoint(s.resources) for s in occupied):
                            feasible.append(delay); break
                self.assertEqual(got.delay_sec if got else None,min(feasible) if feasible else None)
            chosen=base.best_response(templates[f.flight_id],index,net)
            if chosen:
                index.reserve(chosen); occupied.append(chosen)

    def test_free_mode_is_legacy_algorithm_at_fixed_iterations(self):
        args=(fm.flights(14,942,20),fm.network(),fm.types())
        kwargs=dict(candidate_builder=fm.builder(),seed=947,improvement_limit=60,iteration_limit=70)
        old,old_meta,old_initial=base.solve(*args,**kwargs)
        new,new_meta,new_initial=plns.solve(*args,mode='free',**kwargs)
        self.assertEqual(fm.signature(old),fm.signature(new))
        self.assertEqual(fm.signature(old_initial),fm.signature(new_initial))
        self.assertEqual(old_meta['iterations'],new_meta['completed_iterations'])
        self.assertEqual([x['weighted_delay_sec'] for x in old_meta['trace']],
                         [x['weighted_delay_sec'] for x in new_meta['trace']])

    def test_both_protection_invariants_and_group_feasibility(self):
        # Development-only fixture with accepted moves in both protected modes.
        fs,net,types=fm.flights(30,941,60),fm.network(),fm.types()
        for mode in ('initial','monotone'):
            final,meta,initial=plns.solve(fs,net,types,candidate_builder=fm.builder(),
                mode=mode,seed=948,improvement_limit=60,iteration_limit=1000)
            self.assertGreater(meta['accepted_moves'],0)
            current=dict(initial)
            for move in meta['updates']:
                replacement=fm.rebuild(fs,move['replacements'],net,types)
                for fid,s in replacement.items():
                    self.assertLessEqual(s.delay_sec,initial[fid].delay_sec)
                    if mode=='monotone':
                        self.assertLessEqual(s.delay_sec,current[fid].delay_sec)
                self.assertLess(base.weighted(replacement),base.weighted({fid:current[fid] for fid in replacement}))
                current.update(replacement)
                self.assertFalse(fm.ref.audit_reservation_conflicts(current))
            self.assertEqual(fm.signature(current),fm.signature(final))
            self.assertTrue(fm.audit_speed(fs,final,net,types)['passed'])

    def test_failed_or_expired_group_fully_restores_initial(self):
        args=(fm.flights(12,944,20),fm.network(),fm.types())
        original=base.best_response
        for mode in ('initial','monotone'):
            for failure in ('none','expiry'):
                calls=0
                def fake(*a,**kw):
                    nonlocal calls
                    calls+=1
                    if calls==1:
                        return original(*a,**kw)
                    if failure=='expiry':
                        raise base.BudgetExpired
                    return None
                with patch.object(base,'best_response',side_effect=fake):
                    final,meta,initial=plns.solve(*args,candidate_builder=fm.builder(),
                        mode=mode,seed=949,improvement_limit=60,iteration_limit=1)
                self.assertGreaterEqual(calls,2)
                self.assertEqual(meta['accepted_moves'],0)
                self.assertEqual(fm.signature(initial),fm.signature(final))
                self.assertFalse(fm.ref.audit_reservation_conflicts(final))
                self.assertEqual(meta['status'],'TIME_LIMIT' if failure=='expiry' else 'ITERATION_LIMIT')

    def test_zero_budget_and_repeatable_protected_prefix(self):
        args=(fm.flights(12,945,20),fm.network(),fm.types())
        for mode in ('initial','monotone'):
            final,meta,initial=plns.solve(*args,candidate_builder=fm.builder(),mode=mode,improvement_limit=0)
            self.assertEqual(fm.signature(final),fm.signature(initial))
            kw=dict(candidate_builder=fm.builder(),mode=mode,seed=950,improvement_limit=60,iteration_limit=50)
            a,_,_=plns.solve(*args,**kw); b,_,_=plns.solve(*args,**kw)
            self.assertEqual(fm.signature(a),fm.signature(b))


if __name__=='__main__':
    unittest.main(verbosity=2)
