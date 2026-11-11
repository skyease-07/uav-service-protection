"""Nominal continuous-interval audit independent of scheduler geometry code.

Only standard-library functions are used. Bounds are analytic; evaluation uses
binary64 and an explicit length guard, not directed-rounding interval arithmetic.
"""
from dataclasses import dataclass, asdict
from collections import Counter
import csv
import hashlib
import json
import math


@dataclass(frozen=True)
class Motion:
    start: float
    end: float
    p0: tuple
    p1: tuple
    p2: tuple
    z0: float
    z1: float
    offset: float = 0.
    curved: bool = False
    kind: str = 'edge'
    name: str = ''
    layer: int = 1

    def point(self,t):
        u=(t-self.start)/(self.end-self.start)
        if u < -1e-8 or u > 1+1e-8:
            raise ValueError('Time outside motion')
        u=min(1.,max(0.,u))
        if self.curved:
            v=1-u
            x=v*v*self.p0[0]+2*v*u*self.p1[0]+u*u*self.p2[0]
            y=v*v*self.p0[1]+2*v*u*self.p1[1]+u*u*self.p2[1]+self.offset*math.sin(math.pi*u)
        else:
            x=self.p0[0]+u*(self.p2[0]-self.p0[0])
            y=self.p0[1]+u*(self.p2[1]-self.p0[1])
        return x,y,self.z0+u*(self.z1-self.z0)

    def horizontal_speed_bound(self):
        if self.curved:
            derivative=2*max(math.dist(self.p0,self.p1),math.dist(self.p1,self.p2))+math.pi*abs(self.offset)
        else:
            derivative=math.dist(self.p0,self.p2)
        return derivative/(self.end-self.start)


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def read_scenario(path):
    with path.open(encoding='utf-8-sig',newline='') as stream:
        rows=list(csv.DictReader(stream))
    for row in rows:
        row['requested_start_sec']=float(row['requested_start_sec'])
        row['candidate_paths']=json.loads(row['candidate_paths'])
        row['priority']=int(row['priority'])
    return rows


def reconstruct(scenario,rows,net,types):
    """Rebuild motion directly from serialized input, not saved scheduler segments."""
    flights={r['id']:r for r in scenario}
    if len(flights)!=len(scenario) or len({r['id'] for r in rows})!=len(rows):
        raise ValueError('Duplicate flight ID')
    layers={int(k):float(v) for k,v in net['layer_altitude_m'].items()}
    step=net['time_slot_sec']
    result={}; thresholds={}
    for row in rows:
        f=flights[row['id']]; vehicle=types[f['uav_type']]
        path=f['candidate_paths'][row['route_index']]
        assert len(path)==len(row['layers'])==len(row['lanes'])
        assert 0<=row['delay_sec']<=net['max_delay_sec']
        assert abs(row['delay_sec']/step-round(row['delay_sec']/step))<1e-8
        assert row['speed_mode'] in ('nominal','metered')
        current=f['requested_start_sec']+row['delay_sec']; motions=[]
        for i,name in enumerate(path):
            edge=net['edges'][name]; layer=row['layers'][i]; lane=row['lanes'][i]
            assert layer in edge['allowed_layers'] and 0<=lane<len(edge['lane_offsets_m'])
            p0=tuple(net['nodes_xy'][edge['start_node']]); p2=tuple(net['nodes_xy'][edge['end_node']])
            curved=edge['geometry']=='quadratic_bezier'
            assert curved or edge['geometry']=='line'
            offset=edge['lane_offsets_m'][lane]
            if not curved:
                # Current model's line lanes have zero offset and meet the nodes.
                assert offset==0
            p1=tuple(edge['control_point_xy']) if curved else p0
            factor=vehicle['cruise_factor']*(.82 if i==0 and row['speed_mode']=='metered' else 1)
            duration=max(step,math.ceil((edge['base_duration_sec']/factor-1e-9)/step)*step)
            motions.append(Motion(current,current+duration,p0,p1,p2,layers[layer],layers[layer],
                                  offset,curved,'edge',name,layer))
            current+=duration
            if i+1<len(path):
                assert edge['end_node']==net['edges'][path[i+1]]['start_node']
                next_layer=row['layers'][i+1]
                if next_layer!=layer:
                    duration=max(step,math.ceil((abs(layers[next_layer]-layers[layer])/vehicle['vertical_speed_mps']-1e-9)/step)*step)
                    motions.append(Motion(current,current+duration,p2,p2,p2,layers[layer],layers[next_layer],
                                          kind='vertical',name=edge['end_node'],layer=layer))
                    current+=duration
        assert abs(current-row['finish_sec'])<1e-7,(row['id'],current,row['finish_sec'])
        for a,b in zip(motions,motions[1:]):
            assert a.end==b.start and math.dist(a.point(a.end),b.point(b.start))<1e-8
        result[row['id']]=motions
        thresholds[row['id']]=(vehicle['horizontal_separation_m'],vehicle['vertical_separation_m'])
    return result,thresholds


def distances(a,b,t):
    x,y,z=a.point(t); X,Y,Z=b.point(t)
    return math.hypot(x-X,y-Y),abs(z-Z)


def lower_bounds(a,b,lo,hi,guard):
    pa,pb=a.point(lo),b.point(lo); qa,qb=a.point(hi),b.point(hi)
    zlo,zhi=pa[2]-pb[2],qa[2]-qb[2]
    vertical=0. if zlo*zhi<=0 else min(abs(zlo),abs(zhi))
    if not a.curved and not b.curved:
        x,y=pa[0]-pb[0],pa[1]-pb[1]
        dx,dy=qa[0]-qb[0]-x,qa[1]-qb[1]-y
        norm=dx*dx+dy*dy
        u=min(1.,max(0.,-(x*dx+y*dy)/norm)) if norm else 0.
        horizontal=math.hypot(x+u*dx,y+u*dy)
    else:
        horizontal=distances(a,b,(lo+hi)/2)[0]-(a.horizontal_speed_bound()+b.horizontal_speed_bound())*(hi-lo)/2
    return max(0.,horizontal-guard),max(0.,vertical-guard)


def check_motion_pair(a,b,horizontal,vertical,*,guard=1e-6,min_interval=1e-6,max_depth=30,max_cells=100000):
    lo,hi=max(a.start,b.start),min(a.end,b.end)
    if lo>hi:
        return {'status':'separated','cells':0,'subdivisions':0,'leaves':0}
    stack=[(lo,hi,0)]; cells=subdivisions=leaves=unresolved=0
    while stack:
        left,right,depth=stack.pop(); cells+=1
        hbound,vbound=lower_bounds(a,b,left,right,guard)
        if hbound>=horizontal or vbound>=vertical:
            leaves+=1; continue
        for t in (left,(left+right)/2,right):
            h,v=distances(a,b,t)
            if h<horizontal-guard and v<vertical-guard:
                return {'status':'violation','cells':cells,'subdivisions':subdivisions,'leaves':leaves,
                        'witness':{'time_sec':t,'horizontal_m':h,'vertical_m':v,
                                   'threshold_horizontal_m':horizontal,'threshold_vertical_m':vertical,
                                   'motion_a':a.name,'motion_b':b.name}}
        if right-left<=min_interval or depth>=max_depth or cells>=max_cells:
            unresolved+=1
            if cells>=max_cells:
                unresolved+=len(stack); break
            continue
        middle=(left+right)/2
        stack.extend(((middle,right,depth+1),(left,middle,depth+1))); subdivisions+=1
    return {'status':'unresolved' if unresolved else 'separated','cells':cells,
            'subdivisions':subdivisions,'leaves':leaves,'unresolved_cells':unresolved}


def category(a,b):
    if a.kind=='vertical' or b.kind=='vertical':
        return 'involving_vertical'
    if a.name==b.name:
        return 'same_edge'
    return 'different_edges'


def audit(motions,thresholds,**settings):
    ids=sorted(motions); counts=Counter(); categories=Counter(); violations=[]; unresolved=[]
    for j,fid in enumerate(ids):
        for other in ids[j+1:]:
            first,second=motions[fid],motions[other]
            if first[-1].end<second[0].start or second[-1].end<first[0].start:
                continue
            counts['overlapping_flight_pairs']+=1
            h=max(thresholds[fid][0],thresholds[other][0]); v=max(thresholds[fid][1],thresholds[other][1])
            bad=None; unknown=False
            for a in first:
                for b in second:
                    if max(a.start,b.start)>min(a.end,b.end):
                        continue
                    counts['segment_pairs']+=1
                    counts['curved_pairs' if a.curved or b.curved else 'affine_pairs']+=1
                    categories[category(a,b)]+=1
                    result=check_motion_pair(a,b,h,v,**settings)
                    for key in ('cells','subdivisions','leaves'):
                        counts[key]+=result[key]
                    if result['status']=='violation':
                        bad={'flight_a':fid,'flight_b':other,**result['witness']}; break
                    if result['status']=='unresolved':
                        unknown=True
                if bad:
                    break
            if bad:
                violations.append(bad)
            elif unknown:
                unresolved.append({'flight_a':fid,'flight_b':other})
    status='violation' if violations else 'unresolved' if unresolved else 'separated'
    return {'status':status,'counts':dict(counts),'categories':dict(categories),
            'violating_pairs':len(violations),'unresolved_pairs':len(unresolved),
            'witnesses':violations,'unresolved':unresolved,'settings':settings,
            'interpretation':'Nominal continuous-interval numerical audit with analytic bounds and explicit guard; not flight certification or directed-rounding proof.'}


def motion_signature(motions,thresholds):
    payload={'motions':{fid:[asdict(m) for m in values] for fid,values in sorted(motions.items())},
             'thresholds':thresholds}
    return hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest()
