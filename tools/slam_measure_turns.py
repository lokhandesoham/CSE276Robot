"""Measure actual turn angles from a HW3 run log, without the filter: when the same tag is seen
before and after a turn, its bearing atan2(l, f) from the turning centre (and its yaw) change by
the angle turned.   tools/.venv/bin/python tools/slam_measure_turns.py RUN.jsonl [...]"""
import sys, json, math, statistics as st
w=lambda a:(a+math.pi)%(2*math.pi)-math.pi
for path in sys.argv[1:]:
    recs=[json.loads(l) for l in open(path)]
    h=recs[0]; ts=h.get('turn_scale',1.0)
    print('==',path.split('/')[-1],'turn_scale',ts)
    prev=None; motion=None; ratios_b=[]; ratios_p=[]
    for r in recs:
        if r['type']=='motion': motion=r; continue
        if r['type']!='look': continue
        cur={}
        for t,raw in r['raw'].items():
            if len(raw)<2: continue
            md=lambda i: st.median(x[i] for x in raw)
            f,l=md(2)+0.10,-md(0); phi=w(raw[0][3]+st.median(w(x[3]-raw[0][3]) for x in raw))
            cur[t]=(math.atan2(l,f),phi,math.hypot(f,l))
        if prev is not None and motion and motion['kind']=='turn' and abs(motion['value'])>math.radians(20):
            for t in set(cur)&set(prev):
                db=-w(cur[t][0]-prev[t][0]); dp=-w(cur[t][1]-prev[t][1])
                cmd=motion['value']/ts
                ratios_b.append(db/cmd); ratios_p.append(dp/cmd)
                print(f'  turn planned {math.degrees(motion["value"]):5.1f} cmd {math.degrees(cmd):5.1f} dur {motion.get("duration",0):.2f}s | tag {t} r {cur[t][2]:.1f}m: bearing says {math.degrees(db):5.1f}, tag-yaw says {math.degrees(dp):5.1f}')
        prev=cur; motion=None
    if ratios_b: print(f'  actual/commanded: bearing median {st.median(ratios_b):.2f}, tag-yaw median {st.median(ratios_p):.2f}, n={len(ratios_b)}')
