"""Generate a relocatable runtime copy; never edit the archived V2 source."""
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET


def main():
    root = Path(__file__).resolve().parent
    source = root / '源文件'
    resources = root / '模型依赖' / 'robomaster_description'
    replacements = {}
    for sdf in source.glob('*.sdf'):
        for uri in ET.parse(sdf).iter('uri'):
            value = (uri.text or '').strip()
            marker = '/robomaster_description/'
            if marker in value:
                suffix = value.split(marker, 1)[1]
                target = (resources / suffix).resolve()
                if not target.is_file():
                    raise FileNotFoundError(target)
                replacements[value] = target.as_uri()
    runtime = root / 'runtime'
    if runtime.exists():
        backup = root / ('runtime_previous_' + str(time.time_ns()))
        runtime.rename(backup)
        print('Previous runtime preserved:', backup)
    shutil.copytree(source, runtime, ignore=shutil.ignore_patterns('__pycache__'))
    for sdf in runtime.glob('*.sdf'):
        content = sdf.read_text()
        for old, new in replacements.items():
            content = content.replace(old, new)
        sdf.write_text(content)
    print('READY:', runtime)
    print('Resource URIs resolved:', len(replacements))


if __name__ == '__main__':
    main()
