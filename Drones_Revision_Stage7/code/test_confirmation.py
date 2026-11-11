"""Adversarial checks before any formal holdout scenario is generated."""
import json
import math
import unittest
from dataclasses import asdict,replace
import numpy as np
from scipy import stats
import final_model as fm
from indexed_scheduler import schedule_flights
import confirmation_solver as cs
import confirmation_stats as st
import service_metrics as sm
import independent_geometry as ig


class ConfirmationTests(unittest.TestCase):
    def test_guard_rejects_each_hidden_cost(self):
        before={'finish_sec':100.,'airborne_sec':50.,'horizontal_m':600.,'vertical_m':100.}
        for field in before:
            after={**before,field:before[field]+1.}
            self.assertFalse(sm.allowed(after,before),field)
        self.assertTrue(sm.allowed(before,before))
        self.assertTrue(sm.allowed({k:v-1 for k,v in before.items()},before))

    def test_curved_length_is_arc_not_bound(self):
        # x=2u, y=2u(1-u), analytic length sqrt(2)+asinh(1).
        length=sm.arc_length((0.,0.),(1.,1.),(2.,0.),0.,True)
        self.assertAlmostEqual(length,math.sqrt(2)+math.asinh(1),places=10)
        self.assertGreater(length,2.)
        self.assertEqual(sm.arc_length((0.,0.),(0.,0.),(3.,4.),0.,False),5.)

    def test_original_adapter_parity(self):
        for seed,name,n in ((9001,'single_merge',35),(9002,'single_merge',35),(9003,'layered20',25)):
            fs,net,types=fm.flights(n,seed,60,name),fm.network(name),fm.types()
            for method in ('SPC','SPC-UR','SPC-WSR'):
                policy,_=fm.policy(method)
                original=schedule_flights(fs,net,types,policy,candidate_builder=fm.builder())
                adapted,meta,initial=cs.solve(fs,net,types,method)
                self.assertEqual(fm.signature(adapted),fm.signature(original.schedules))
                self.assertEqual(meta['passes'],len(original.improvement_history))

    def test_guard_preserves_all_updates_and_initialization(self):
        # Tiny synthetic fixture forces a protected layer reassignment: SPC
        # prefers 4 s delay to the 8 s upper-layer penalty, reinsertion does not.
        net=replace(fm.network(),nodes_xy={'A':(0.,0.),'B':(12.,0.)},
            edges={'E':fm.ref.EdgeConfig('E','A','B',(1,2),2.,(0.,))},
            node_buffer_before_sec=0.,node_buffer_after_sec=0.,vertical_buffer_sec=0.)
        fs=[fm.ref.FlightPlan('A',2,0.,'A',(('E',),),'rescue'),
            fm.ref.FlightPlan('B',5,0.,'A',(('E',),),'rescue')]
        types=fm.types()
        final,meta,initial=cs.solve(fs,net,types,'SPC-WSR',True)
        self.assertGreater(len(meta['updates']),0)
        for update in meta['updates']:
            before=fm.rebuild(fs,[update['before']],net,types)[update['before']['id']]
            after=fm.rebuild(fs,[update['after']],net,types)[update['after']['id']]
            self.assertLessEqual(after.delay_sec,before.delay_sec)
            self.assertTrue(sm.allowed(sm.schedule_costs(after,net),sm.schedule_costs(before,net)))
        self.assertFalse(fm.ref.audit_reservation_conflicts(final))
        plain,_,plain_initial=cs.solve(fs,net,types,'SPC-WSR')
        self.assertEqual(fm.signature(initial),fm.signature(plain_initial))

    def test_serialized_service_metrics(self):
        fs,net,types=fm.flights(20,9005,60),fm.network(),fm.types()
        final,meta,initial=cs.solve(fs,net,types,'SPC-WSR')
        scenario=[]
        for f in fs:
            row=asdict(f); row['id']=row.pop('flight_id'); scenario.append(row)
        rows=[cs.selection(s) for s in final.values()]
        motions,_=ig.reconstruct(scenario,rows,json.loads(json.dumps(asdict(net))),{k:asdict(v) for k,v in types.items()})
        for f in fs:
            actual=sm.from_motion(motions[f.flight_id],f.requested_start_sec)
            expected=sm.schedule_costs(final[f.flight_id],net)
            for field in expected: self.assertAlmostEqual(expected[field],actual[field],places=7)

    def test_mean_test_and_multiplicity(self):
        values=np.array([0.,1.,2.,4.,7.,9.]); margin=1.
        actual=st.contrast(values,margin)
        self.assertAlmostEqual(actual['p_raw'],stats.ttest_1samp(values,margin,alternative='greater').pvalue)
        self.assertEqual(st.holm([.03,.01]),[.03,.02])
        self.assertEqual(st.contrast([0.,0.,0.],0)['p_raw'],1.)
        self.assertEqual(st.contrast([2.,2.,2.],1)['p_raw'],0.)


if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(ConfirmationTests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    fm.dump(fm.ROOT/'results/confirmation_tests.json',{'passed':result.wasSuccessful(),'tests':result.testsRun,
        'development_seeds':[100,9001,9002,9003,9004,9005],
        'source_hashes':{n:fm.sha(fm.ROOT/'code'/n) for n in ('confirmation_solver.py','service_metrics.py','confirmation_stats.py','test_confirmation.py')}})
    raise SystemExit(0 if result.wasSuccessful() else 1)
