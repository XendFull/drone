"""Build two externally identical unpowered test gliders with concealed ballast.

No classification/tracking inputs. Both variants use the same base airframe,
opaque outer pod faces, and equal-volume built-in printed ballast. Only the
longitudinal position of the ballast differs. The names are experiment labels.
"""
from pathlib import Path
import importlib.util
import json
import shutil
import sys
import zipfile
import xml.etree.ElementTree as ET
import struct

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "source"
SOURCE.mkdir(exist_ok=True)
BASE_PATH = SOURCE / "base_glider.py"
original = ROOT.parent / "build_glider.py"
if not BASE_PATH.exists():
    if not original.exists():
        raise FileNotFoundError("The source/base_glider.py file must accompany this generator")
    shutil.copyfile(original, BASE_PATH)
spec = importlib.util.spec_from_file_location("base_glider", BASE_PATH)
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)

# The forward body remains the closure plate between the two matching pods.
POD_X = 15.
POD_Y_DOWN = 4.
POD_LENGTH = 85.
POD_HEIGHT = 18.
POD_DEPTH = 4.
FLOOR = .8
WALL = .8
ACTUAL_START = 14.
DECOY_START = 4.   # Same block translated 10 mm forward, not further aft.
BASE_ASSEMBLY = list(g.assembly_meshes())


def pod(start, length):
    end = start + length
    assert start - .8 > 1.6 and end + .8 < POD_LENGTH - 1.6
    xs = sorted(set([0., .8, 1.6, start-.8, start, end, end+.8, 83.4, 84.2, 85.]))
    ys = [0., .8, 1.6, 16.4, 17.2, 18.]
    def height(x, y):
        distance_to_edge = min(x, POD_LENGTH-x, y, POD_HEIGHT-y)
        rim = max(0., min(1., (1.6-distance_to_edge)/.8))
        block = max(0., min(1., (x-start+.8)/.8, (end+.8-x)/.8))
        return FLOOR + (POD_DEPTH-FLOOR)*max(rim, block)
    return g.height_surface(xs, ys, lambda x,y:(x,y), height)


def pod_assembly(mesh, sign):
    return mesh.mapped(lambda x,y,z:(POD_X+x, sign*(.4+POD_DEPTH-z), -POD_Y_DOWN-y))


def assembled(mesh):
    return BASE_ASSEMBLY + [
        ("concealed_pod_right", pod_assembly(mesh, 1), "#176e7a"),
        ("concealed_pod_left", pod_assembly(mesh, -1), "#176e7a")]


def mass_properties(mesh):
    solids = assembled(mesh)
    volume = sum(m.vol for _,m,_ in solids)
    centre = sum(m.vol*m.centroid() for _,m,_ in solids)/volume
    return volume, centre


def solve_ballast_length():
    low, high = 20., 65.
    assert mass_properties(pod(ACTUAL_START, low))[1][0] > g.CG_X
    assert mass_properties(pod(ACTUAL_START, high))[1][0] < g.CG_X
    for _ in range(55):
        mid = (low+high)/2
        if mass_properties(pod(ACTUAL_START, mid))[1][0] > g.CG_X:
            low = mid
        else:
            high = mid
    return (low+high)/2


def render(variants, reports, length):
    plt.rcParams.update({"font.family":"DejaVu Sans", "font.size":10})
    fig = plt.figure(figsize=(14,9), facecolor="#f5f3ed")
    fig.text(.045,.94,"TWO GLIDERS. SAME OUTSIDE.",fontsize=25,fontweight="bold",color="#17323b")
    fig.text(.045,.90,"Concealed printed ballast differs in position; wings, body, colour and total modelled mass match.",fontsize=11,color="#52636a")
    for idx,(name,mesh) in enumerate(variants.items()):
        left = .025 + idx*.5
        ax = fig.add_axes([left,.39,.48,.45],projection="3d",computed_zorder=False)
        ax.set_facecolor("#f5f3ed")
        for _,m,colour in assembled(mesh):
            ax.add_collection3d(Poly3DCollection(m.v[m.f],facecolor=colour,edgecolor="none"))
        ax.set_xlim(-20,330);ax.set_ylim(-185,185);ax.set_zlim(-35,60)
        ax.set_box_aspect([350,370,95],zoom=1.6);ax.view_init(elev=28,azim=-54)
        ax.set_proj_type("ortho");ax.set_axis_off()
        fig.text(left+.04,.84,name.upper(),fontsize=16,fontweight="bold",color="#17323b")
        fig.text(left+.04,.405,"Same exterior / no printed class label",fontsize=10,color="#52636a")
        start = ACTUAL_START if name=="Actual" else DECOY_START
        cut = fig.add_axes([left+.04,.18,.41,.15])
        cut.set_aspect("equal")
        cut.add_patch(Rectangle((0,0),POD_LENGTH,POD_HEIGHT,fc="#176e7a"))
        cut.add_patch(Rectangle((1.6,1.6),POD_LENGTH-3.2,POD_HEIGHT-3.2,fc="#e3e7e4"))
        cut.add_patch(Rectangle((start,1.6),length,POD_HEIGHT-3.2,fc="#ec762e"))
        cut.text(POD_LENGTH/2,-5,"INTERNAL CUTAWAY — hidden after assembly",ha="center",fontsize=8,color="#52636a")
        cut.text(start+length/2,8.5,"fixed printed ballast",ha="center",va="center",fontsize=8,color="#17323b")
        cut.text(0,23,"nose end",fontsize=8,color="#52636a")
        cut.set_xlim(-2,87);cut.set_ylim(-8,27);cut.axis("off")
        fig.text(left+.04,.12,f"Calculated CG: {reports[name]['cg_mm_from_nose']:.2f} mm from nose",fontsize=11,color="#17323b")
    fig.text(.045,.055,"Experimental configurations: different motion, glide stability and recognition are not yet demonstrated.",fontsize=10,color="#8a4333")
    fig.savefig(ROOT/"Actual_and_Decoy_preview.png",dpi=160,facecolor=fig.get_facecolor())
    plt.close(fig)


def roundtrip_check(folder):
    namespace={"m":g.NS}
    counts={"stl":0,"3mf":0}
    for path in folder.glob("*.stl"):
        data=path.read_bytes()
        count=struct.unpack_from("<I",data,80)[0]
        assert len(data)==84+50*count
        vertices=[]; faces=[]; ids={}
        for i in range(count):
            record=struct.unpack_from("<12fH",data,84+50*i)
            face=[]
            for j in (3,6,9):
                vertex=tuple(record[j:j+3])
                if vertex not in ids:
                    ids[vertex]=len(vertices); vertices.append(vertex)
                face.append(ids[vertex])
            faces.append(face)
        g.Mesh(vertices,faces).validate();counts["stl"]+=1
    for path in folder.glob("*.3mf"):
        with zipfile.ZipFile(path) as z:
            assert z.testzip() is None
            root=ET.fromstring(z.read("3D/3dmodel.model"))
            assert root.attrib["unit"]=="millimeter"
            objects=root.findall("m:resources/m:object",namespace)
            assert len(objects)==len(root.findall("m:build/m:item",namespace))
            for obj in objects:
                vertices=[[float(v.attrib[k]) for k in ("x","y","z")] for v in obj.findall("m:mesh/m:vertices/m:vertex",namespace)]
                faces=[[int(f.attrib[k]) for k in ("v1","v2","v3")] for f in obj.findall("m:mesh/m:triangles/m:triangle",namespace)]
                mesh=g.Mesh(vertices,faces);mesh.validate()
                assert min(mesh.v.min(0))>=0 and max(mesh.v.max(0)[:2])<=180
            counts["3mf"]+=1
    return counts


def instructions(name, report):
    return f'''# {name} — WideView 350, concealed-ballast version

This is a complete print set for ONE glider. Print the separate Actual set and Decoy set to build the pair. Each glider is modular across five 180 x 180 mm plates; it is not a one-piece print. The original v1 files are preserved separately.

## What is different?

Actual and Decoy have identical designed exterior surfaces and equal modelled amounts of the same material. They use the same airframe parts plus two side pods on the forward body. The body closes each pod's open side, hiding its fixed printed ballast. The ballast in Decoy is 10 mm farther forward inside the pod than in Actual. There is no sliding weight, extra metal, external class label, or different outline.

The calculated {name} balance point is **{report['cg_mm_from_nose']:.2f} mm from the nose**. This is a material-volume calculation, not a measured or approved flight limit. Both versions need individual weighing, balance checks and glide testing. The names Actual/Decoy are experiment labels; different or recognisable flight paths are not guaranteed.

## Printing

- Use the SAME opaque filament, printer, orientation, scale and settings for both versions. Use solid/opaque material; transparent or translucent PETG may reveal the internal ballast. If the internals show through, cover BOTH pods with the same opaque finish and rebalance both models.
- 0.4 mm nozzle, **0.20 mm layers AND 0.20 mm first layer**. Keep 100% scale and adaptive layers off.
- **100% infill**, 2 walls, 3 top/bottom layers. The hidden ballast must be printed solid; low or different infill undermines the comparison. Inspect the sliced preview.
- Use your filament manufacturer's profile. PETG remains an experimental thin-wing option; PLA is the preferred wing material for stiffness. Do not mix materials between the two configurations.
- No supports/raft. The pods print with their closed outside face on the bed and open, stepped face upward. Keep their supplied orientation. Saddles print on their sides.
- Use a PETG-compatible build plate; follow the plate maker's release-layer guidance. Let the bed cool before releasing thin panels.
- The 3MF files contain geometry in millimetres, not machine-specific settings or G-code. Choose your actual printer.

| Plate | Print once for this glider |
|---|---|
| 1 | Right wing half |
| 2 | Left wing half |
| 3 | Both body pieces, both saddles, two joint plates |
| 4 | Horizontal tailplane |
| 5 | TWO concealed-ballast side pods for {name} |

That is 11 physical flight parts per glider: 9 base airframe pieces and 2 pods. The one pod STL is printed twice. Both open faces go inward, against the body. The seam side is hidden after assembly.

## Assembly

Follow the included `BASE_AIRFRAME_ASSEMBLY.md` for the body, wings, tail and saddles. **Replace its old nose-clay balancing step with the pod instructions here. Do not add the original 7 g nose ballast.** These versions already include calculated printed ballast.

1. Assemble the base airframe straight and square. The nose is the rounded end, the triangular fin is the rear. Use equal, small adhesive quantities on the two models.
2. Orient each pod with its LONG axis along the body. One long outside wall is unbroken and smooth; the opposite face is open and reveals the printed ballast. Keep that open face against the body.
3. For this {name} set, the pod's outside face reaches {POD_DEPTH:g} mm out from each side of the body. Fit ONE pod on each side, aligned to the SAME fore-aft position. Each pod starts **15 mm from the nose** and ends at 100 mm. Its upper edge is **4 mm below the body's straight top edge**; its lower edge is 22 mm below it. The 0.8 mm body profile serves as the closing plate between the two pods.
4. The asymmetric internal block identifies the nose end BEFORE sealing. In {name}, the full-height internal block starts **{report['block_start_mm_inside_pod']:.2f} mm from the pod's NOSE end**. Verify against the cutaway image and measure it before gluing. Flipping a pod end-for-end changes the balance; do not do that. Mirror left/right only.
5. Bond the open perimeter lips to the body with a thin, continuous bond line and a compatible adhesive. The concealed ballast is integral with the pod and cannot rattle or slide. Do not glue until you have checked its orientation and the adhesive on a small sample. Check cured retention before flight.
6. Weigh the completed glider and physically measure its balance point. Modelled mass in PETG is approximately **{report['petg_solid_mass_g']:.2f} g before adhesive/finish**. Actual extrusion and glue can change mass and balance. The Actual/Decoy modelled masses match, but printed weights are not guaranteed equal.
7. For exterior matching, use the same colour/finish, seams and assembly methods on both. Put any experiment identifier on a removable storage tag or log, not a different visible marking on one model. The preview's captions are not embossed into the parts.

## Flight and presentation limits

Test each glider independently with gentle hand launches over a forgiving landing surface and without the drone in the corridor. These are unpowered rigid printed prototypes. No safe CG envelope or usable trajectory difference has been established. Stop if either stalls, dives, tumbles, sheds a pod or shows panel flutter. Do not use an elastic/powered launcher.

Compare repeat launches from the same point and height, with the same launch procedure and lighting. Measure the real trajectories before training a motion classifier. Use separate flights for evaluation and randomise the order. The drone camera's own movement and distance can mimic changes in apparent speed; a stable external camera is useful for ground-truth measurement, even if the demonstration uses only the drone's camera.

The exact outside geometry is the same by design. Physical appearance can still differ through print defects, translucent material, glue, wear, or inconsistent finishing. Equal geometry and mass do not guarantee recognisable flight behaviour.

## Checks completed

Closed connected mesh, winding, nondegenerate triangles, export round-trip, bed-fit and non-overlap checks were performed. Both pod variants have matching modelled volume and exterior surfaces; only their hidden geometry differs. Structural retention, printing, real mass, glide stability and recognition have not been physically tested.
'''


def main():
    length=solve_ballast_length()
    variants={"Actual":pod(ACTUAL_START,length),"Decoy":pod(DECOY_START,length)}
    # Check equality of all exposed pod faces independent of triangulation.
    def exterior_signature(mesh):
        surface=[]
        for tri in mesh.v[mesh.f]:
            outside=any(np.allclose(tri[:,axis],value,atol=1e-9) for axis,value in
                        [(0,0.),(0,POD_LENGTH),(1,0.),(1,POD_HEIGHT),(2,0.)])
            if outside: surface.append(tri)
        # Equality of external triangles can be checked on total areas and moments;
        # the face boundaries are also identical rectangles by construction.
        area=0.;moment=np.zeros(3)
        for tri in surface:
            a=np.linalg.norm(np.cross(tri[1]-tri[0],tri[2]-tri[0]))/2
            area+=a;moment+=a*tri.mean(0)
        return np.r_[area,moment]
    assert np.allclose(exterior_signature(variants['Actual']),exterior_signature(variants['Decoy']),rtol=1e-10,atol=1e-7)
    assert abs(variants['Actual'].vol-variants['Decoy'].vol)<1e-7
    reports={}
    for name,mesh in variants.items():
        folder=ROOT/name
        files=folder/"print_files";files.mkdir(parents=True,exist_ok=True)
        for part_name,part in g.PARTS.items():
            part['mesh'].validate()
            g.stl(files/(part_name+".stl"),part['mesh'])
        pod_info=mesh.validate()
        g.stl(files/("09_"+name.upper()+"_concealed_pod_PRINT_TWO.stl"),mesh)
        for plate,items in g.PLATES.items():
            g.validate_plate(items)
            g.model3mf(files/(plate+".3mf"),items,name+" / "+plate)
        items=[(mesh,np.array([5,5,0])),(mesh,np.array([5,35,0]))]
        g.validate_plate(items)
        g.model3mf(files/("plate_5_"+name.upper()+"_ballast_pods.3mf"),items,name+" / TWO sealed-on-assembly ballast pods")
        volume,centre=mass_properties(mesh)
        reports[name]={"cg_mm_from_nose":float(centre[0]),"lateral_cg_mm":float(centre[1]),
            "petg_solid_mass_g":volume*.00127,"pla_solid_mass_g":volume*.00124,
            "model_volume_mm3":volume,"block_length_mm":length,
            "block_start_mm_inside_pod":ACTUAL_START if name=="Actual" else DECOY_START,
            "pod":pod_info,"closed_meshes_and_exports":roundtrip_check(files)}
        (folder/"START_HERE.md").write_text(instructions(name,reports[name]),encoding="utf-8")
        guide_path=ROOT.parent/"START_HERE.md" if (ROOT.parent/"START_HERE.md").exists() else SOURCE/"BASE_AIRFRAME_ASSEMBLY.md"
        guide=guide_path.read_text(encoding="utf-8")
        section=guide.split("## Assembly",1)[1].split("## First glide checks",1)[0]
        lines=[]
        for line in section.splitlines():
            if line.startswith("6. **Balance."):
                line="6. **Concealed ballast and balance.** Follow this version's START_HERE.md for the two integral-ballast pods and the measured balance checks. Do not add the old v1 nose-clay ballast."
            lines.append(line)
        clean_guide="# Base airframe assembly for the concealed-ballast pair\n\n## Assembly\n"+"\n".join(lines)+"\n"
        (folder/"BASE_AIRFRAME_ASSEMBLY.md").write_text(clean_guide,encoding="utf-8")
    assert abs(reports['Actual']['petg_solid_mass_g']-reports['Decoy']['petg_solid_mass_g'])<1e-8
    assert reports['Decoy']['cg_mm_from_nose']<reports['Actual']['cg_mm_from_nose']
    assert abs(reports['Actual']['cg_mm_from_nose']-125.)<1e-6
    # The mounting rectangles are inside the forward body's side profile.
    assert POD_X>=14 and POD_X+POD_LENGTH<=100
    assert POD_Y_DOWN+POD_HEIGHT<=24
    report={"design":"WideView350 fixed concealed ballast pair v2",
            "exterior_match":"identical airframe; identical rectangular pod outer faces and bounds",
            "equal_modelled_mass":True,"physical_testing":"not performed","versions":reports}
    (ROOT/"pair_validation.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    render(variants,reports,length)
    shutil.copyfile(ROOT/"Actual"/"BASE_AIRFRAME_ASSEMBLY.md",SOURCE/"BASE_AIRFRAME_ASSEMBLY.md")
    for name in variants:
        folder=ROOT/name
        shutil.copyfile(ROOT/"Actual_and_Decoy_preview.png",folder/"pair_preview.png")
        (folder/"geometry_validation.json").write_text(json.dumps(reports[name],indent=2),encoding="utf-8")
        archive=ROOT/(name.upper()+"_glider_print_pack.zip")
        with zipfile.ZipFile(archive,"w",zipfile.ZIP_DEFLATED) as z:
            for path in sorted(folder.rglob("*")):
                if path.is_file(): z.write(path,path.relative_to(folder))
        with zipfile.ZipFile(archive) as z: assert z.testzip() is None
    with zipfile.ZipFile(ROOT/"Actual_and_Decoy_COMPLETE.zip","w",zipfile.ZIP_DEFLATED) as z:
        for name in variants:
            for path in sorted((ROOT/name).rglob("*")):
                if path.is_file():z.write(path,path.relative_to(ROOT))
        for filename in ("Actual_and_Decoy_preview.png","pair_validation.json","build_two_gliders.py"):
            z.write(ROOT/filename,filename)
        for filename in ("base_glider.py","BASE_AIRFRAME_ASSEMBLY.md"):
            z.write(SOURCE/filename,"source/"+filename)
    print(json.dumps({name:{key:round(value,3) for key,value in props.items() if isinstance(value,float)} for name,props in reports.items()},indent=2))
    print("Two complete print packs exported and validated; original v1 retained.")


if __name__=="__main__":main()
