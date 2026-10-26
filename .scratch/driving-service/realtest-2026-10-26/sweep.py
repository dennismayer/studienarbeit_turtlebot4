import numpy as np, math, sys
import driving_service.visitor_detector as vd
CAPS={'together 1.2m 0deg':('person_1p2m_0deg.npz',(1.0,1.5),(-10,10)),
      'legs apart 1.2m 0deg':('legs_apart.npz',(1.0,1.5),(-10,10)),
      'left 1.5m +20deg':('left_1p5m_20deg.npz',(1.2,1.9),(8,32)),
      'empty':('empty.npz',None,None)}
def run(gap,wmin,wmax,conf=0.75,show=False):
    vd.MAX_POINT_GAP=gap
    p=vd.DetectionParams(min_distance=0.8,max_distance=2.5,angle_range=60.0,person_width_min=wmin,person_width_max=wmax,stability_threshold=8,visitor_timeout=10.0,cooldown_period=5.0,min_confidence=0.0,frame_skip_rate=1,lidar_front_angle=-math.pi/2)
    det=vd.VisitorDetector(p); res=[]
    for name,(f,dr,ar) in CAPS.items():
        d=np.load(f); S=d['ranges']; amin=float(d['angle_min']); inc=float(d['angle_increment'])
        hit=other=0; conf_me=[]; others=[]
        for r in S:
            angles=amin+np.arange(len(r))*inc
            rel=np.arctan2(np.sin(angles-p.lidar_front_angle),np.cos(angles-p.lidar_front_angle))
            m=np.isfinite(r)&(r>=float(d['range_min']))&(r<=float(d['range_max']))&(np.abs(rel)<=math.radians(30))
            o=np.argsort(rel[m]); rr,aa=r[m][o],rel[m][o]
            cands=[c for cr,ca in vd._clusters(rr,aa) if (c:=det._score(cr,ca)) is not None]
            me=[c for c in cands if dr and dr[0]<=c.distance<=dr[1] and ar[0]<=math.degrees(c.angle)<=ar[1]]
            if me and max(c.confidence for c in me)>=conf: hit+=1; conf_me.append(max(me,key=lambda c:c.confidence))
            oth=[c for c in cands if c not in me and c.confidence>=conf]
            if oth: other+=1; others+=oth
        res.append((name,len(S),hit,other,conf_me,others))
    return res
if __name__=='__main__':
    print('gap  width       | together | apart | left | empty(false scans)')
    for gap in (0.1,0.2,0.3):
        for wmin,wmax in ((0.15,0.5),(0.15,0.45),(0.2,0.5),(0.12,0.5)):
            res=run(gap,wmin,wmax)
            cells=[f'{h}/{n}+{o}f' for _,n,h,o,_,_ in res[:3]]+[f'{res[3][3]}/{res[3][1]}']
            print(f'{gap:.1f}  {wmin:.2f}-{wmax:.2f} | '+' | '.join(cells))
