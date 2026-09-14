"""Recompute report statistics from packaged, unmodified simulation records."""
import json
import math
import statistics
from collections import Counter
from pathlib import Path

DATA = Path(__file__).resolve().parent / 'supporting_data'

def inspect(name):
    data = json.loads((DATA/name/'results.json').read_text())
    assert data['status'] == 'COMPLETED'
    assert data['requested_cycles'] == data['completed_cycles'] == 5
    target = data['configuration']['b_world_m']
    errors=[]
    for row in data['cycles']:
        assert row['success'] and row['home_verified']
        p=row['placement_world_m']
        err=1000*math.hypot(p[0]-target[0],p[1]-target[1])
        errors.append(err)
        print(name,row['cycle'],f'error_mm={err:.3f}',
              f'heading_deg={abs(math.degrees(row["heading_error_rad"])):.3f}',
              f'cycle_wall_seconds={row["duration_seconds"]:.2f}')
    print('mean_mm',statistics.mean(errors),'max_mm',max(errors))
    print('sum_cycle_wall_seconds',sum(r['duration_seconds'] for r in data['cycles']))

if __name__ == '__main__':
    for name in ['20260913_153855_682865','20260913_151835_077998']:
        inspect(name)
    events=[json.loads(line) for line in
            (DATA/'20260913_153855_682865'/'events.jsonl').read_text().splitlines()]
    counts=Counter(e['event'] for e in events)
    assert counts['B_PLACEMENT_VERIFIED']==counts['CYCLE_HOME_VERIFIED']==5
    assert counts['SCENE_REFILL_REQUESTED_AFTER_HOME']==counts['SCENE_REFILL_ACKNOWLEDGED']==4
    assert counts['SAFE_STOP']==0
    start=next(e['monotonic_seconds'] for e in events
               if e['event']=='PHASE_ENTER' and e['phase']=='INITIAL_HOME')
    end=next(e['monotonic_seconds'] for e in events if e['event']=='FIVE_CYCLES_COMPLETED_AT_HOME')
    print('events',len(events),dict(counts),'initial_home_to_completion_seconds',end-start)
