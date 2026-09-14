"""Validate a saved five-cycle run without starting Gazebo or moving hardware."""
import json
import sys
from pathlib import Path

def verify(folder):
    root=Path(folder)
    result=json.loads((root/'results.json').read_text())
    events=[json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
    assert result['status']=='COMPLETED',result['status']
    assert result['completed_cycles']==5
    sequence=['HOVER','DESCEND','CLOSE','HOLD','LIFT','TURN','LOWER','OPEN','RETREAT','RETURN_HEADING','RETURN_HOME']
    for cycle in range(1,6):
        current=[e for e in events if e['cycle']==cycle]
        phases=[e['phase'] for e in current if e['event']=='PHASE_ENTER' and e['phase'] not in ('INITIAL_HOME','REFILL_WAIT')]
        assert phases==sequence,(cycle,phases)
        names=[e['event'] for e in current]
        assert names.count('B_PLACEMENT_VERIFIED')==1
        assert names.count('CYCLE_HOME_VERIFIED')==1
        assert names.index('B_PLACEMENT_VERIFIED')<names.index('CYCLE_HOME_VERIFIED')
        if cycle<5 and result['configuration']['refill_mode']=='automatic_after_home':
            assert names.index('CYCLE_HOME_VERIFIED')<names.index('SCENE_REFILL_REQUESTED_AFTER_HOME')<names.index('SCENE_REFILL_ACKNOWLEDGED')
    assert not any(e['event']=='SAFE_STOP' for e in events)
    print('PASS: 5 placements, 5 measured returns home, refill only between completed cycles.')
    return result

if __name__=='__main__':verify(sys.argv[1])
