"""Package the existing WideView geometry for the requested glider recognizer."""
import importlib.util
import json
from pathlib import Path
import shutil
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'Glide350'


def load_airframe():
    spec = importlib.util.spec_from_file_location('_glide350_airframe', ROOT / 'build_glider.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    OUT.mkdir(exist_ok=True)
    airframe = load_airframe()
    reports = {name: part['mesh'].validate() for name, part in airframe.PARTS.items()}
    for dirname in ('print_files', 'foam_templates'):
        shutil.copytree(ROOT / dirname, OUT / dirname, dirs_exist_ok=True)
    shutil.copy2(ROOT / 'START_HERE.md', OUT / 'ASSEMBLY_AND_PRINTING.md')
    shutil.copy2(ROOT / 'design_preview.png', OUT / 'preview.png')
    shutil.copy2(ROOT / 'print_layouts.png', OUT / 'print_layouts.png')
    items = [(mesh, np.zeros(3)) for _, mesh, _ in airframe.assembly_meshes()]
    airframe.model3mf(OUT / 'Glide350_ASSEMBLED_VIEW_ONLY.3mf', items,
                     'Glide350 assembly reference - print separate plate files')
    vertices, faces, offset = [], [], 0
    for mesh, _ in items:
        vertices.extend(mesh.v)
        faces.extend(mesh.f + offset)
        offset += len(mesh.v)
    # Assembly reference contains separate touching/intersecting component
    # shells. Each individual printable part is independently watertight.
    assembled = airframe.Mesh(vertices, faces)
    airframe.stl(OUT / 'Glide350_ASSEMBLED_VIEW_ONLY.stl', assembled)
    report = {'name': 'Glide350', 'based_on': 'existing WideView 350 airframe',
              'span_mm': 350, 'length_mm': 310, 'parts': reports,
              'assembly_reference_component_shells': len(items),
              'physical_flight_tested': False, 'camera_visibility_measured': False}
    (OUT / 'geometry_checks.json').write_text(json.dumps(report, indent=2)+'\n')
    guide = '''# Glide350 — hand-launched glider prototype

Open Glide350_ASSEMBLED_VIEW_ONLY.3mf to inspect the whole glider in 3D.
For printing, use the separate STLs or numbered plates under print_files;
the assembled reference is not a one-piece print layout.

Nominal span: 350 mm. Body length: 310 mm. Print area: 180 x 180 mm.
The package reuses the existing validated WideView 350 geometry.
Read ASSEMBLY_AND_PRINTING.md for dimensions, fit coupon, assembly and balance.

For the lighter option, print the body, saddles and joiners, then use the
foam_templates outlines for the two wing halves and horizontal tail.
For an entirely printed prototype, use all the separate flight-part STLs.
The thin printed skins require the documented 0.20 mm first and subsequent
layers; inspect the slicer preview and print the coupon before full plates.

Use a consistent opaque, high-contrast finish such as orange wings and a
dark body. The preview palette is a suggested finish, not embedded filament
instructions. No colour guarantees camera recognition. Balance the actual
build at the documented starting point; use gentle hand launches and adjust
trim after measured tests. No flight time is promised or measured.

The supplied initial recognition model is trained on synthetic renderings of
this geometry. Render-only scores do not measure recognition of a real print.
Real camera photos and separate test flights are needed before claiming
working recognition while airborne.

Aerodynamic background: NASA, Three Forces on a Glider and Wing Geometry:
https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/three-forces-on-a-glider/
https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/wing-geometry/
These explain general principles and do not validate this specific airframe.
'''
    (OUT / 'READ_ME.md').write_text(guide, encoding='utf-8')
    archive = ROOT / 'Glide350_3D_and_print_files.zip'
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as zipped:
        for path in OUT.rglob('*'):
            if path.is_file():
                zipped.write(path, path.relative_to(OUT))
    print('Validated', len(reports), 'individual parts; packaged', archive)


if __name__ == '__main__':
    main()
