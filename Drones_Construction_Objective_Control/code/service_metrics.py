"""Dimensioned service proxies; no power or monetary interpretation."""
import math
from functools import lru_cache
from scipy.integrate import quad


@lru_cache(maxsize=None)
def arc_length(p0,p1,p2,offset,curved):
    if not curved:
        return math.dist(p0,p2)
    def speed(u):
        dx=2*((1-u)*(p1[0]-p0[0])+u*(p2[0]-p1[0]))
        dy=2*((1-u)*(p1[1]-p0[1])+u*(p2[1]-p1[1]))+offset*math.pi*math.cos(math.pi*u)
        return math.hypot(dx,dy)
    value,error=quad(speed,0.,1.,epsabs=1e-8,epsrel=1e-10)
    if error>1e-6:
        raise ValueError('Arc-length integration unresolved')
    return value


def schedule_costs(s,net):
    horizontal=vertical=0.
    for seg in s.segments:
        if seg.kind=='vertical':
            vertical+=abs(net.layer_altitude_m[seg.layer_to]-net.layer_altitude_m[seg.layer_from])
        else:
            edge=net.edges[seg.edge_id]; p0=tuple(net.nodes_xy[edge.start_node]); p2=tuple(net.nodes_xy[edge.end_node])
            p1=tuple(edge.control_point_xy) if edge.geometry=='quadratic_bezier' else p0
            horizontal+=arc_length(p0,p1,p2,edge.lane_offsets_m[seg.lane],edge.geometry=='quadratic_bezier')
    return {'finish_sec':s.finish_sec,'airborne_sec':s.finish_sec-s.scheduled_start_sec,
            'horizontal_m':horizontal,'vertical_m':vertical}


def allowed(after,before):
    return all(after[k]<=before[k]+(1e-6 if k.endswith('_m') else 1e-8)
               for k in ('finish_sec','airborne_sec','horizontal_m','vertical_m'))


def from_motion(motions,request):
    horizontal=sum(arc_length(m.p0,m.p1,m.p2,m.offset,m.curved) for m in motions if m.kind=='edge')
    vertical=sum(abs(m.z1-m.z0) for m in motions)
    return {'finish_sec':motions[-1].end,'airborne_sec':motions[-1].end-motions[0].start,
        'horizontal_m':horizontal,'vertical_m':vertical,
        'completion_from_request_sec':motions[-1].end-request,
        'vertical_transitions':sum(m.kind=='vertical' for m in motions)}
