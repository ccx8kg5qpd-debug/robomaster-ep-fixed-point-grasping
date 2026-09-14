"""Stop only this V2 launch tree, not other ROS / Gazebo sessions."""
import os,signal,subprocess
from pathlib import Path
target=str(Path(__file__).resolve().parent/'ep_transfer.launch.py')
rows=[line.strip().split(None,2) for line in subprocess.check_output(['ps','-eo','pid,ppid,args'],text=True).splitlines()[1:]]
roots=[int(pid) for pid,ppid,args in rows if args.split()[-2:]==['launch',target]]
found=set(roots)
while True:
 children={int(pid) for pid,ppid,args in rows if int(ppid) in found}
 if children<=found:break
 found.update(children)
print('Stopping V2 process IDs:',sorted(found))
for pid in sorted(found,reverse=True):
 try:os.kill(pid,signal.SIGTERM)
 except ProcessLookupError:pass
