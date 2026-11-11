"""Development parity and counterexamples before locking formal execution."""
import unittest
import tempfile
import gzip
import numpy as np
import final_model as fm
import confirmation_solver as old
import split_solver as solver
from split_common import *
from analyze_split import gain,summarize
from audit_split import audit_events

class SplitTests(unittest.TestCase):
    def test_guard_counterexamples(self):
        b=dict(zip(METRICS,[100.,40.,1000.,100.]))
        # Waiting saved permits longer airborne time without later completion.
        a={**b,'finish_sec':95.,'airborne_sec':45.}
        self.assertEqual(solver.violations(a,b),2)
        # Multi-label overlap must retain all causes, including redundant F.
        a={k:v+1 for k,v in b.items()}
        self.assertEqual(solver.violations(a,b),15)
        self.assertEqual(solver.violations({**b,'horizontal_m':1001.,'vertical_m':99.},b),4)
        self.assertEqual(solver.violations({**b,'finish_sec':100.+1e-9,'horizontal_m':1000.+1e-7},b),0)
        for d in (0.,10.,30.):
            for nd in (0.,d):
                for t in (20.,40.):
                    self.assertLessEqual(nd+t,d+40.)

    def test_parity_and_logging(self):
        for name,seed in [('single_merge',100),('layered20',0),('layered40',0)]:
            fs,net,types=fm.flights(12,seed,60,name),fm.network(name),fm.types()
            templates,_=solver.prepare(fs,net,types); initial,failures,_=solver.construct(fs,net,templates)
            frozen=fm.signature(initial)
            for m in P['methods']:
                events=[]; ours,meta=solver.improve(initial,net,templates,m,failures,events.append)
                silent,_=solver.improve(initial,net,templates,m,failures)
                self.assertEqual(fm.signature(ours),fm.signature(silent))
                self.assertEqual(fm.signature(initial),frozen)
                with tempfile.TemporaryDirectory() as temporary:
                    trace=Path(temporary)/'events.jsonl.gz'
                    with gzip.open(trace,'wt',encoding='utf-8') as f:
                        for event in events: f.write(json.dumps(event)+'\n')
                    record={'method':m,'solver':meta,'schedule_signature':fm.signature(ours)}
                    visits,_=audit_events(trace,record,initial,templates,net)
                    self.assertEqual(visits,len(events))
                for e in events:
                    self.assertEqual(len(e['steps']),e['infeasible']+e['nonimproving']+len(e['improving']))
                    self.assertEqual(e['accepted'],e['best'][m])
                    if e['best']['T'] is not None: self.assertIsNotNone(e['best']['F'])
                    if e['best']['ALL'] is not None: self.assertIsNotNone(e['best']['T'])
                if m in ('SPC','WSR','ALL'):
                    baseline,bmeta,binit=old.solve(fs,net,types,'SPC' if m=='SPC' else 'SPC-WSR',guard=m=='ALL')
                    self.assertEqual(fm.signature(binit),frozen)
                    self.assertEqual(fm.signature(baseline),fm.signature(ours))
                    self.assertEqual([(u['before'],u['after']) for u in meta['updates']],[(u['before'],u['after']) for u in bmeta['updates']])
                    self.assertEqual(meta['history'],bmeta['history'])

    def test_earliest_against_full_resource_keys(self):
        fs,net,types=fm.flights(4,107,20),fm.network(),fm.types()
        templates,_=solver.prepare(fs,net,types); initial,_,_=solver.construct(fs,net,templates)
        for current in initial.values():
            index=solver.IntervalIndex(); occupied=set()
            for s in initial.values():
                if s.flight_id!=current.flight_id: index.reserve(s); occupied.update(s.resources)
            bound=int(current.delay_sec/net.time_slot_sec)
            for base,intervals,_ in templates[current.flight_id]:
                expected=next((step for step in range(bound+1)
                    if all(k[:-1]+(k[-1]+step,) not in occupied for k in base.resources)),None)
                self.assertEqual(index.earliest(intervals,bound),expected)

    def test_statistics(self):
        self.assertEqual(gain(100,90),10); self.assertEqual(gain(100,110),-10)
        self.assertEqual(gain(0,0),0); self.assertIsNone(gain(0,1))
        idx=np.array([[0,1],[0,0],[1,1]])
        self.assertEqual(summarize([3,3],idx)['ci95'],[3,3])

if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(SplitTests))
    dump(ROOT/'validation/development_tests.json',{'passed':result.wasSuccessful(),'tests':result.testsRun,
        'failures':len(result.failures),'errors':len(result.errors),'source_hashes':source_hashes()})
    raise SystemExit(0 if result.wasSuccessful() else 1)
