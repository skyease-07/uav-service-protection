"""Development-only correctness tests, run before the new local lock."""
from types import SimpleNamespace
import unittest
import numpy as np
import final_model as fm
import confirmation_solver as original
import objective_solver as solver
from analyze_objectives import gain,summarize
from objective_common import ROOT,dump,source_hashes


class ObjectiveTests(unittest.TestCase):
    def test_actual_tradeoff(self):
        a=SimpleNamespace(delay_sec=0.,selection_cost=30.,finish_sec=100.,route_index=0,layer_plan=(1,),lane_plan=(0,))
        b=SimpleNamespace(delay_sec=10.,selection_cost=15.,finish_sec=100.,route_index=0,layer_plan=(1,),lane_plan=(0,))
        self.assertIs(min([a,b],key=lambda s:solver.construction_key(s,'composite')),b)
        self.assertIs(min([a,b],key=lambda s:solver.construction_key(s,'delay_first')),a)

    def test_development_parity_and_shared_start(self):
        for name,seed in [('single_merge',100),('layered20',0),('layered40',0)]:
            fs,net,types=fm.flights(12,seed,60,name),fm.network(name),fm.types()
            templates,_=solver.prepare(fs,net,types)
            initial,failures,_=solver.construct(fs,net,templates,'composite')
            frozen=fm.signature(initial)
            for method in ['SPC','SPC-UR','SPC-WSR']:
                ours,meta=solver.improve(initial,net,templates,method,failures)
                old,old_meta,old_initial=original.solve(fs,net,types,method)
                self.assertEqual(frozen,fm.signature(old_initial))
                self.assertEqual(fm.signature(ours),fm.signature(old))
                self.assertEqual(meta['passes'],old_meta['passes'])
                self.assertEqual([(u['before'],u['after']) for u in meta['updates']],[(u['before'],u['after']) for u in old_meta['updates']])
                self.assertEqual(frozen,fm.signature(initial))

    def test_delay_first_against_exhaustive_resource_search(self):
        fs,net,types=fm.flights(4,107,20),fm.network(),fm.types()
        templates,_=solver.prepare(fs,net,types)
        actual,failures,_=solver.construct(fs,net,templates,'delay_first')
        policy,_=fm.policy('SPC')
        occupied=set(); expected={}
        for f in sorted(fs,key=lambda f:fm.ref._dispatch_key(f,policy)):
            candidates=[]
            for choice in fm.ref.enumerate_structural_choices(f,net):
                for d in range(0,int(net.max_delay_sec)+1,int(net.time_slot_sec)):
                    c=fm.build_candidate(f,choice,float(d),net,types)
                    if not (c.resources & occupied):
                        candidates.append(c); break
            best=min(candidates,key=lambda c:(c.delay_sec,c.selection_cost,c.finish_sec,c.route_index,c.layer_plan,c.lane_plan))
            expected[f.flight_id]=best; occupied.update(best.resources)
        self.assertFalse(failures)
        self.assertEqual(fm.signature(expected),fm.signature(actual))

    def test_signed_paired_statistics_and_zero_handling(self):
        self.assertEqual(gain(100,90),10.)
        self.assertEqual(gain(100,110),-10.)
        self.assertEqual(gain(0,0),0.)
        self.assertIsNone(gain(0,1))
        indices=np.array([[0,1],[1,0],[0,0],[1,1]])
        a=np.array([10,20]); b=a+3
        s=summarize(b-a,indices)
        self.assertEqual(s['mean'],3.)
        self.assertEqual(s['ci95'],[3.,3.])


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ObjectiveTests))
    dump(ROOT/'validation/development_tests.json',{'passed':result.wasSuccessful(),'tests':result.testsRun,
          'failures':len(result.failures),'errors':len(result.errors),'source_hashes':source_hashes()})
    raise SystemExit(0 if result.wasSuccessful() else 1)
