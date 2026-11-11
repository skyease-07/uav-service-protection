"""Adversarial checks with analytically known outcomes, plus reconstruction parity."""
import unittest
import math
from dataclasses import asdict,replace
import independent_geometry as ig
import final_model as fm


def line(start,end,p0,p2,z=100,name='test'):
    return ig.Motion(start,end,p0,p0,p2,z,z,name=name)


class GeometryTests(unittest.TestCase):
    def test_collision_between_quarter_second_samples(self):
        a=line(0,.25,(-10,0),(10,0)); b=line(0,.25,(0,-10),(0,10))
        self.assertGreater(ig.distances(a,b,0)[0],1)
        self.assertGreater(ig.distances(a,b,.25)[0],1)
        result=ig.check_motion_pair(a,b,1,1)
        self.assertEqual(result['status'],'violation')
        self.assertEqual(result['witness']['time_sec'],.125)

    def test_closed_endpoint_and_vertical_crossing(self):
        a=line(0,1,(0,0),(10,0)); b=line(1,2,(10,0),(20,0))
        self.assertEqual(ig.check_motion_pair(a,b,1,1)['status'],'violation')
        vertical=ig.Motion(0,10,(5,0),(5,0),(5,0),50,150,kind='vertical')
        horizontal=line(0,10,(0,0),(10,0))
        self.assertEqual(ig.check_motion_pair(vertical,horizontal,1,1)['status'],'violation')
        self.assertEqual(ig.check_motion_pair(horizontal,replace(horizontal,z0=200,z1=200),25,35)['status'],'separated')

    def test_curved_bounds_and_adaptive_refinement(self):
        a=ig.Motion(0,10,(0,0),(50,100),(100,0),100,100,30,True)
        b=replace(a,p0=(0,50),p1=(50,150),p2=(100,50))
        result=ig.check_motion_pair(a,b,25,35)
        self.assertEqual(result['status'],'separated')
        self.assertGreater(result['subdivisions'],0)
        self.assertEqual(ig.check_motion_pair(a,a,25,35)['status'],'violation')
        self.assertEqual(ig.check_motion_pair(a,b,25,35,max_cells=1)['status'],'unresolved')

    def test_numerical_boundary_is_not_silently_safe(self):
        a=line(0,1,(0,0),(10,0)); b=line(0,1,(0,25),(10,25))
        self.assertEqual(ig.check_motion_pair(a,b,25,35,max_cells=10)['status'],'unresolved')
        self.assertEqual(ig.check_motion_pair(a,replace(b,p0=(0,25.01),p2=(10,25.01)),25,35)['status'],'separated')

    def test_resources_do_not_cover_unrelated_crossing(self):
        net=replace(fm.network(),nodes_xy={'A':(-100,0),'B':(100,0),'C':(0,-100),'D':(0,100)},
            edges={'X':fm.ref.EdgeConfig('X','A','B',(1,),20,(0.,)),
                   'Y':fm.ref.EdgeConfig('Y','C','D',(1,),20,(0.,))})
        fs=[fm.ref.FlightPlan('A',2,0,'A',(('X',),),'cargo'),
            fm.ref.FlightPlan('B',2,0,'C',(('Y',),),'cargo')]
        ss=[fm.build_candidate(f,fm.ref.enumerate_structural_choices(f,net)[0],0,net,fm.types()) for f in fs]
        self.assertFalse(ss[0].resources & ss[1].resources)
        scenario=[{'id':f.flight_id,'requested_start_sec':0,'candidate_paths':f.candidate_paths,'uav_type':f.uav_type} for f in fs]
        rows=[row(s) for s in ss]
        motions,thresholds=ig.reconstruct(scenario,rows,asdict(net),{k:asdict(v) for k,v in fm.types().items()})
        self.assertEqual(ig.audit(motions,thresholds)['status'],'violation')

    def test_safe_following_is_rejected_by_whole_edge(self):
        net=fm.network(); types=fm.types()
        fs=[fm.ref.FlightPlan(fid,2,0,'C',(('E3',),),'cargo') for fid in ('A','B')]
        ss=[fm.build_candidate(f,fm.ref.enumerate_structural_choices(f,net)[0],d,net,types) for f,d in zip(fs,(0,16))]
        self.assertTrue(ss[0].resources & ss[1].resources)
        scenario=[{'id':f.flight_id,'requested_start_sec':0,'candidate_paths':f.candidate_paths,'uav_type':f.uav_type} for f in fs]
        motions,thresholds=ig.reconstruct(scenario,[row(s) for s in ss],asdict(net),{k:asdict(v) for k,v in types.items()})
        self.assertEqual(ig.audit(motions,thresholds)['status'],'separated')

    def test_reconstruction_all_types_routes_layers_and_modes(self):
        net=fm.network(); types=fm.types()
        checks=0
        for name in types:
            f=fm.ref.FlightPlan('TEST',5,7,'O2',(('E2','E3'),('E2','E4')),name)
            scenario=[{'id':'TEST','requested_start_sec':7,'candidate_paths':f.candidate_paths,'uav_type':name}]
            for choice in fm.ref.enumerate_structural_choices(f,net):
                s=fm.build_candidate(f,choice,14,net,types)
                motions,_=ig.reconstruct(scenario,[row(s)],asdict(net),{k:asdict(v) for k,v in types.items()})
                self.assertEqual(len(motions['TEST']),len(s.segments))
                for m,segment in zip(motions['TEST'],s.segments):
                    self.assertEqual((m.start,m.end),(segment.start_sec,segment.end_sec))
                    for fraction in (0,.13,.5,.91,1):
                        t=m.start+fraction*(m.end-m.start)
                        self.assertLess(math.dist(m.point(t),fm.ref.segment_position(segment,t,net)),1e-8)
                        checks+=1
        self.assertGreater(checks,500)


def row(s):
    return {'id':s.flight_id,'delay_sec':s.delay_sec,'finish_sec':s.finish_sec,
            'route_index':s.route_index,'layers':list(s.layer_plan),'lanes':list(s.lane_plan),'speed_mode':s.speed_mode}


if __name__=='__main__':
    unittest.main(verbosity=2)
