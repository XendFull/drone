"""Generate an original, unpowered experimental glider. No tracker/model inputs.

Only Python, numpy, and matplotlib are used. Dimensions are millimetres.
STL meshes are closed solids, not overlapping component shells.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import struct
import zipfile
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "print_files"
OUT.mkdir(exist_ok=True)
SKIN = 0.20
SPAN = 350.0
HALF = SPAN / 2
DIHEDRAL = math.radians(5)
INCIDENCE = math.radians(2)
WING_LE = 100.0
WING_MOUNT_X = 130.0
WING_ROOT_Z = 3.8
TAIL_LE = 230.0
TAIL_Z = 2.6
CG_X = 125.0
PARTS = {}


class Mesh:
    def __init__(self, vertices, faces):
        self.v = np.asarray(vertices, dtype=float)
        self.f = np.asarray(faces, dtype=int)
        a, b, c = self.v[self.f].transpose((1, 0, 2))
        self.vol = float(np.sum(np.einsum("ij,ij->i", a, np.cross(b, c))) / 6)
        if self.vol < 0:
            self.f = self.f[:, ::-1]
            self.vol *= -1

    def mapped(self, func):
        return Mesh([func(*v) for v in self.v], self.f.copy())

    def shifted(self, xyz):
        return Mesh(self.v + np.asarray(xyz), self.f.copy())

    def bed(self):
        return self.shifted(-self.v.min(axis=0))

    def centroid(self):
        a, b, c = self.v[self.f].transpose((1, 0, 2))
        w = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6
        return np.sum(w[:, None] * (a + b + c) / 4, axis=0) / np.sum(w)

    def validate(self):
        edges = Counter()
        directions = Counter()
        tri = self.v[self.f]
        areas = np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1) / 2
        assert np.isfinite(self.v).all() and np.min(areas) > 1e-9
        for face in self.f:
            for a, b in zip(face, np.roll(face, -1)):
                edges[tuple(sorted((int(a), int(b))))] += 1
                directions[(int(a), int(b))] += 1
        assert set(edges.values()) == {2}, f"Open/nonmanifold edges: {Counter(edges.values())}"
        assert all(directions[(a, b)] == directions[(b, a)] for a, b in edges), "Inconsistent winding"
        adjacency = defaultdict(set)
        for a, b in edges:
            adjacency[a].add(b)
            adjacency[b].add(a)
        reached, pending = set(), [0]
        while pending:
            node = pending.pop()
            if node not in reached:
                reached.add(node)
                pending.extend(adjacency[node] - reached)
        assert len(reached) == len(self.v), "Disconnected solid"
        assert len(self.v) - len(edges) + len(self.f) == 2, "Unexpected topology"
        assert self.vol > 0
        return {"vertices": len(self.v), "triangles": len(self.f), "watertight": True,
                "consistent_winding": True, "connected_solids": 1,
                "bounds_mm": np.round(self.v.max(0) - self.v.min(0), 3).tolist(),
                "volume_mm3": round(self.vol, 3), "petg_solid_mass_g_at_1_27": round(self.vol * .00127, 3)}


def triangulate(poly):
    """Ear clipping for a simple 2D polygon, including concave outlines."""
    p = np.asarray(poly, float)
    area = np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1]) / 2
    ids = list(range(len(p))) if area > 0 else list(reversed(range(len(p))))
    faces = []
    cross = lambda u, v: u[0] * v[1] - u[1] * v[0]
    def inside(q, a, b, c):
        return min(cross(b-a, q-a), cross(c-b, q-b), cross(a-c, q-c)) >= -1e-9
    while len(ids) > 3:
        for k in range(len(ids)):
            ia, ib, ic = ids[k-1], ids[k], ids[(k+1) % len(ids)]
            a, b, c = p[[ia, ib, ic]]
            if cross(b-a, c-b) <= 1e-10:
                continue
            if any(inside(p[j], a, b, c) for j in ids if j not in (ia, ib, ic)):
                continue
            faces.append((ia, ib, ic))
            ids.pop(k)
            break
        else:
            raise ValueError("Polygon could not be triangulated")
    faces.append(tuple(ids))
    return p, faces


def extrusion(poly, thickness):
    p, triangles = triangulate(poly)
    n = len(p)
    verts = [(x, y, 0) for x, y in p] + [(x, y, thickness) for x, y in p]
    faces = [tuple(reversed(t)) for t in triangles] + [tuple(i+n for i in t) for t in triangles]
    perimeter = list(range(n))
    if np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1]) < 0:
        perimeter.reverse()
    for a, b in zip(perimeter, perimeter[1:] + perimeter[:1]):
        faces += [(a, b, b+n), (a, b+n, a+n)]
    return Mesh(verts, faces)


def height_surface(ss, ts, xy, height, keep=lambda s, t: True):
    """A closed, continuous, reinforced sheet on a flat print bed."""
    points, grid, surface = [], {}, []
    def vertex(i, j):
        key = (i, j)
        if key not in grid:
            grid[key] = len(points)
            s, t = ss[i], ts[j]
            x, y = xy(s, t)
            points.append((x, y, height(s, t)))
        return grid[key]
    for i in range(len(ss)-1):
        for j in range(len(ts)-1):
            if not keep((ss[i]+ss[i+1])/2, (ts[j]+ts[j+1])/2):
                continue
            a, b, c, d = vertex(i,j), vertex(i+1,j), vertex(i+1,j+1), vertex(i,j+1)
            surface += [(a,b,c), (a,c,d)]
    verts = [(x,y,0) for x,y,z in points] + points
    n = len(points)
    faces = [tuple(reversed(t)) for t in surface] + [tuple(i+n for i in t) for t in surface]
    boundary = Counter()
    oriented = {}
    for face in surface:
        for a,b in zip(face, (face[1],face[2],face[0])):
            key = tuple(sorted((a,b)))
            boundary[key] += 1
            oriented[key] = (a,b)
    for edge, count in boundary.items():
        if count == 1:
            a,b = oriented[edge]
            faces += [(a,b,b+n),(a,b+n,a+n)]
    return Mesh(verts, faces)


def tent(x, centre, halfwidth):
    return max(0., 1-abs(x-centre)/halfwidth)


def wing():
    ribs = [12., 60., 115., 165.]
    ss = sorted(set([0., 8., 10., 14., 35., 85., 140., 170., HALF] +
                    [v for r in ribs for v in [r-.6, r, r+.6]]))
    ts = [0., .015, .02, .266, .28, .294, .55, .634, .64, .646, .82, .99, 1.]
    def xy(s,t):
        le = 12*s/HALF
        chord = 100-30*s/HALF
        if s > 170:
            clip = (s-170)*.8
            le += clip
            chord -= 2*clip
        return s, le+t*chord
    def height(s,t):
        root = .2*max(0., min(1., (14-s)/4))
        main = 1.8*tent(t,.28,.014)*(1-.45*s/HALF)
        aft = .6*tent(t,.64,.006)
        rib = max(.6*tent(s,r,.6) for r in ribs)
        return SKIN + max(root,main,aft,rib)
    return height_surface(ss, ts, xy, height)


def tail():
    ss = [-85., -80., -55., -25., -8., -.6, 0., .6, 8., 25., 55., 80., 85.]
    ts = [0., .23, .25, .27, .45, .65, 1.]
    def xy(s,t):
        le = 6*abs(s)/85
        chord = 60-20*abs(s)/85
        return s, le+t*chord
    def height(s,t):
        return SKIN + max(.8*tent(t,.25,.02), .2*max(0.,1-abs(s)/8))
    return height_surface(ss,ts,xy,height,lambda s,t: not (abs(s)<.6 and t>.45))


def saddle(width, length, centre_height, dihedral, incidence, slot_width=1.1, slot_height=2.4):
    """Thin cap on a U stem. Export on its side to avoid shelf overhangs."""
    x = width/2
    h = centre_height
    dh = math.tan(dihedral)*x
    stem=slot_width/2+1.0
    under=h-.8
    p = [(-stem,0),(-slot_width/2,0),(-slot_width/2,slot_height),
         (slot_width/2,slot_height),(slot_width/2,0),(stem,0),
         (stem,under+math.tan(dihedral)*stem),(x,under+dh),
         (x,h+dh),(0,h),(-x,h+dh),(-x,under+dh),
         (-stem,under+math.tan(dihedral)*stem)]
    base = extrusion(p,length)
    def remap(a,b,c):
        # Original polygon coordinate b is height. c becomes chord direction.
        # Sloping the upper vertices only leaves the feet flat on the print bed.
        upper = b >= h-1e-8
        z = b + ((length/2-c)*math.tan(incidence) if upper else 0)
        return a, c, z
    return base.mapped(remap)


front_poly = [(0,9),(4,3),(14,0),(155,0),(155,14),(130,18),(127,18.48),
              (125,15.8),(123,19.12),(100,24),(35,28),(12,24),(0,17)]
rear_poly = [(155,0),(258,0),(272,-38),(281,-44),(291,-35),(310,0),
             (310,7),(230,10),(155,14)]

w = wing()
PARTS["01_wing_right"] = {"mesh": w.bed(), "assembly": w.mapped(lambda s,c,h:
    (WING_LE+c, s*math.cos(DIHEDRAL), WING_ROOT_Z+s*math.sin(DIHEDRAL)-(c-30)*math.tan(INCIDENCE)+h)),
    "qty":1,"color":"#ec762e"}
PARTS["02_wing_left"] = {"mesh": w.mapped(lambda s,c,h:(-s,c,h)).bed(), "assembly":w.mapped(lambda s,c,h:
    (WING_LE+c,-s*math.cos(DIHEDRAL), WING_ROOT_Z+s*math.sin(DIHEDRAL)-(c-30)*math.tan(INCIDENCE)+h)),
    "qty":1,"color":"#ec762e"}

for name,poly in [("03_fuselage_front",front_poly),("04_fuselage_rear_with_fin",rear_poly)]:
    mesh=extrusion(poly,.8)
    PARTS[name]={"mesh":mesh.bed(),"assembly":mesh.mapped(lambda x,y,t:(x,t-.4,-y)),"qty":1,"color":"#176e7a"}

t=tail()
PARTS["05_tailplane"]={"mesh":t.bed(),"assembly":t.mapped(lambda s,c,h:(TAIL_LE+c,s,TAIL_Z+h)),"qty":1,"color":"#eca12e"}
mount=saddle(24,12,WING_ROOT_Z+2.4,DIHEDRAL,INCIDENCE)
PARTS["06_wing_saddle"]={"mesh":mount.mapped(lambda s,c,h:(s,h,c)).bed(),"assembly":mount.mapped(lambda s,c,h:(124+c,s,h-2.4)),"qty":1,"color":"#314250"}
mount=saddle(18,10,TAIL_Z+1.6,0,0,slot_height=1.6)
PARTS["07_tail_saddle"]={"mesh":mount.mapped(lambda s,c,h:(s,h,c)).bed(),"assembly":mount.mapped(lambda s,c,h:(237+c,s,h-1.6)),"qty":1,"color":"#314250"}
joiner=extrusion([(2,0),(26,0),(28,2),(28,6),(26,8),(2,8),(0,6),(0,2)],.6)
PARTS["08_fuselage_joiner_PRINT_TWO"]={"mesh":joiner,"assembly":joiner.mapped(lambda x,y,t:(141+x,.4+t,-3-y)),"qty":2,"color":"#314250"}


def stl(path, mesh):
    with path.open("wb") as f:
        f.write(b"WideView 350 experimental unpowered glider; units mm".ljust(80,b" "))
        f.write(struct.pack("<I",len(mesh.f)))
        for face in mesh.f:
            pts=mesh.v[face]
            normal=np.cross(pts[1]-pts[0],pts[2]-pts[0])
            normal/=np.linalg.norm(normal)
            f.write(struct.pack("<12fH",*normal,*pts.flatten(),0))


NS="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
ET.register_namespace("",NS)
def model3mf(path, items, title):
    q=lambda key:f"{{{NS}}}{key}"
    model=ET.Element(q("model"),{"unit":"millimeter","xml:lang":"en-US"})
    ET.SubElement(model,q("metadata"),{"name":"Title"}).text=title
    ET.SubElement(model,q("metadata"),{"name":"Description"}).text="Geometry only; choose your own printer and PETG profile. No G-code."
    res=ET.SubElement(model,q("resources")); build=ET.SubElement(model,q("build"))
    for i,(mesh,offset) in enumerate(items,1):
        obj=ET.SubElement(res,q("object"),{"id":str(i),"type":"model"})
        mel=ET.SubElement(obj,q("mesh")); vs=ET.SubElement(mel,q("vertices")); fs=ET.SubElement(mel,q("triangles"))
        for x,y,z in mesh.v+offset:
            ET.SubElement(vs,q("vertex"),{"x":f"{x:.6f}","y":f"{y:.6f}","z":f"{z:.6f}"})
        for a,b,c in mesh.f:
            ET.SubElement(fs,q("triangle"),{"v1":str(a),"v2":str(b),"v3":str(c)})
        ET.SubElement(build,q("item"),{"objectid":str(i)})
    with zipfile.ZipFile(path,"w",zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml",'<?xml version="1.0"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
        z.writestr("_rels/.rels",'<?xml version="1.0"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        z.writestr("3D/3dmodel.model",ET.tostring(model,encoding="utf-8",xml_declaration=True))


def validate_plate(items):
    boxes=[]
    for mesh,offset in items:
        lo=mesh.v.min(0)+offset; hi=mesh.v.max(0)+offset
        assert np.min(lo)>=-1e-6 and np.max(hi[:2])<=180
        for other_lo,other_hi in boxes:
            overlap=np.minimum(hi[:2],other_hi[:2])-np.maximum(lo[:2],other_lo[:2])
            assert not np.all(overlap>0),"Parts overlap on plate"
        boxes.append((lo,hi))


def assembly_meshes():
    for name,part in PARTS.items():
        yield name,part["assembly"],part["color"]
        if part["qty"]==2:
            mesh=part["assembly"].mapped(lambda x,y,z:(x,-y,z))
            yield name+"_second",mesh,part["color"]


def render():
    meshes=list(assembly_meshes())
    plt.rcParams.update({"font.family":"DejaVu Sans","font.size":10})
    fig=plt.figure(figsize=(15,10),facecolor="#f5f3ed")
    fig.text(.055,.945,"WIDEVIEW 350",fontsize=29,fontweight="bold",color="#17323b")
    fig.text(.055,.91,"A larger, unpowered glider prototype for moving-target camera trials",fontsize=12,color="#52636a")
    ax=fig.add_axes([.025,.32,.69,.57],projection="3d",computed_zorder=False)
    ax.set_facecolor("#f5f3ed")
    for name,m,color in meshes:
        coll=Poly3DCollection(m.v[m.f],facecolor=color,edgecolor="none",alpha=1,shade=False)
        ax.add_collection3d(coll)
    ax.set_xlim(-20,330);ax.set_ylim(-185,185);ax.set_zlim(-35,60)
    ax.set_box_aspect([350,370,95],zoom=1.65);ax.view_init(elev=28,azim=-54);ax.set_proj_type("ortho");ax.set_axis_off()
    fig.text(.73,.80,"350 mm",fontsize=28,fontweight="bold",color="#17323b")
    fig.text(.73,.765,"projected span ~349 mm",fontsize=10,color="#52636a")
    fig.text(.73,.69,"310 mm",fontsize=28,fontweight="bold",color="#17323b")
    fig.text(.73,.655,"overall body length",fontsize=10,color="#52636a")
    fig.text(.73,.57,"180 x 180 mm",fontsize=18,fontweight="bold",color="#17323b")
    fig.text(.73,.535,"minimum clear print area",fontsize=10,color="#52636a")
    fig.text(.73,.46,"PETG / PLA",fontsize=20,fontweight="bold",color="#17323b")
    fig.text(.73,.425,"0.4 mm nozzle / 0.2 mm layers",fontsize=10,color="#52636a")
    # Orthographic top silhouette uses actual mesh projections.
    top=fig.add_axes([.06,.095,.32,.21]);top.set_aspect("equal");top.set_axis_off()
    from matplotlib.collections import PolyCollection
    for name,m,color in meshes:
        top.add_collection(PolyCollection(m.v[m.f][:,:,:2],facecolors=color,edgecolors="none"))
    top.set_xlim(-10,320);top.set_ylim(-185,185)
    top.plot([CG_X,CG_X],[-90,90],"--",color="#182a31",lw=1)
    top.annotate("initial balance: 125 mm from nose",(CG_X,0),(15,-165),fontsize=8,
                 arrowprops={"arrowstyle":"->","color":"#182a31"})
    side=fig.add_axes([.45,.13,.48,.15]);side.set_aspect("equal");side.set_axis_off()
    for name,m,color in meshes:
        side.add_collection(PolyCollection(m.v[m.f][:,:,[0,2]],facecolors=color,edgecolors="none"))
    side.set_xlim(-10,320);side.set_ylim(-35,55)
    side.annotate("5 degree wing dihedral / 2 degree incidence",(160,18),(5,48),fontsize=9)
    fig.text(.055,.045,"EXPERIMENTAL: geometry checked; flight, camera visibility and drone contact are unverified.",fontsize=10,color="#8a4333")
    fig.savefig(ROOT/"design_preview.png",dpi=170,facecolor=fig.get_facecolor())
    plt.close(fig)
    # Four bed layouts, including parts actually packed in each geometry-only 3MF.
    fig,axes=plt.subplots(2,2,figsize=(12,11),facecolor="#f5f3ed")
    for ax,(plate,items) in zip(axes.flat,PLATES.items()):
        ax.set_facecolor("#fffefa")
        ax.plot([0,180,180,0,0],[0,0,180,180,0],color="#a9b1b2")
        for mesh,offset in items:
            tris=mesh.v[mesh.f]+np.asarray(offset)
            ax.add_collection(PolyCollection(tris[:,:,:2],facecolors="#eb8038",edgecolors="none"))
        ax.set_xlim(-3,183);ax.set_ylim(-3,183);ax.set_aspect("equal")
        ax.set_title(plate.replace("_"," "),loc="left",fontweight="bold",fontsize=12,pad=10)
        ax.set_xlabel("mm");ax.set_ylabel("mm")
    fig.suptitle("PRINT FOUR PLATES AT 100% SCALE",fontsize=20,fontweight="bold",x=.08,ha="left")
    fig.tight_layout(rect=[0,0,1,.95]);fig.savefig(ROOT/"print_layouts.png",dpi=150,facecolor=fig.get_facecolor());plt.close(fig)


PLATES={
    "plate_1_right_wing":[(PARTS["01_wing_right"]["mesh"],np.array([2.5,20,0]))],
    "plate_2_left_wing":[(PARTS["02_wing_left"]["mesh"],np.array([2.5,20,0]))],
    "plate_3_body_and_connectors":[
        (PARTS["03_fuselage_front"]["mesh"],np.array([10,5,0])),
        (PARTS["04_fuselage_rear_with_fin"]["mesh"],np.array([10,45,0])),
        (PARTS["06_wing_saddle"]["mesh"],np.array([10,115,0])),
        (PARTS["07_tail_saddle"]["mesh"],np.array([42,115,0])),
        (PARTS["08_fuselage_joiner_PRINT_TWO"]["mesh"],np.array([70,115,0])),
        (PARTS["08_fuselage_joiner_PRINT_TWO"]["mesh"],np.array([106,115,0])),
    ],
    "plate_4_tailplane":[(PARTS["05_tailplane"]["mesh"],np.array([5,45,0]))],
}


def main():
    report={"design":"WideView 350 v1","units":"mm","status":"experimental; not flight-tested",
            "minimum_print_area_mm":[180,180],"parts":{}}
    for name,part in PARTS.items():
        info=part["mesh"].validate()
        assert max(info["bounds_mm"][:2])<=175.001
        info["print_quantity"]=part["qty"]
        report["parts"][name]=info
        stl(OUT/(name+".stl"),part["mesh"])
    for plate,items in PLATES.items():
        validate_plate(items)
        model3mf(OUT/(plate+".3mf"),items,"WideView 350 / "+plate)
    coupon=height_surface([0.,12.,14.,15.,16.,18.,30.],[0.,.25,.28,.31,1.],
        lambda s,t:(s,t*30),lambda s,t:SKIN+max(1.8*tent(t,.28,.03),.6*tent(s,15.,1.)))
    tab=extrusion([(0,0),(20,0),(20,8),(0,8)],.8)
    coupon.validate();tab.validate()
    stl(OUT/"00_skin_coupon.stl",coupon)
    stl(OUT/"00_keel_fit_tab.stl",tab)
    checks=[(coupon,np.array([5,5,0])),(tab,np.array([45,5,0])),
            (PARTS["06_wing_saddle"]["mesh"],np.array([5,45,0]))]
    validate_plate(checks)
    model3mf(OUT/"plate_0_TEST_FIRST.3mf",checks,"WideView 350 / PETG skin and saddle fit check")
    assembled=list(assembly_meshes())
    volume=sum(m.vol for _,m,_ in assembled)
    cg=sum(m.vol*m.centroid() for _,m,_ in assembled)/volume
    ballast=volume*.00127*(cg[0]-CG_X)/(CG_X-15)
    report["assembly"]={"geometric_petg_mass_g_at_1_27":round(volume*.00127,2),
        "geometric_pla_mass_g_at_1_24":round(volume*.00124,2),
        "geometric_unballasted_cg_mm_from_nose":round(float(cg[0]),2),
        "initial_balance_mm_from_nose":CG_X,
        "estimated_nose_ballast_g_at_x15_petg":round(float(ballast),2),
        "caveat":"Solid-volume estimates exclude glue, tape, extrusion variation and trim. Weigh and physically balance the actual build."}
    report["plates"]={k:{"parts":len(v),"within_180mm":True,"bounding_boxes_nonoverlapping":True} for k,v in PLATES.items()}
    (ROOT/"geometry_validation.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    templates=ROOT/"foam_templates";templates.mkdir(exist_ok=True)
    def svg_template(path,outline,title,w,h):
        pts=" ".join(f"{x+5:.3f},{y+8:.3f}" for x,y in outline)
        text=f'''<svg xmlns="http://www.w3.org/2000/svg" width="{w}mm" height="{h}mm" viewBox="0 0 {w} {h}">
<rect width="100%" height="100%" fill="white"/><polygon points="{pts}" fill="none" stroke="black" stroke-width="0.25"/>
<text x="5" y="5" font-family="sans-serif" font-size="3">{title}</text>
<path d="M 5,{h-10} h 50 m -50,-2 v 4 m 50,-4 v 4" stroke="black" stroke-width="0.25" fill="none"/>
<text x="5" y="{h-3}" font-family="sans-serif" font-size="3">Scale check: 50 mm. Print at 100% / actual size.</text></svg>'''
        path.write_text(text,encoding="utf-8")
    svg_template(templates/"wing_half_1to1.svg",[(0,0),(170,12*170/175),(175,16),(175,78),(170,100-18*170/175),(0,100)],
                 "Wing half: cut TWO, mirror one; use light model foam",185,130)
    # Tail outline includes a narrow clearance notch for the vertical fin.
    outline=[(-85,6),(0,0),(85,6),(85,46),(.6,60-14*.6/85),(.6,27),(-.6,27),(-.6,60-14*.6/85),(-85,46)]
    svg_template(templates/"tailplane_1to1.svg",[(s+85,c) for s,c in outline],
                 "Tailplane: cut ONE; centre notch clears the fin",180,90)
    render()
    archive=ROOT/"WideView350_print_pack.zip"
    with zipfile.ZipFile(archive,"w",zipfile.ZIP_DEFLATED) as z:
        for folder in (OUT,templates):
            for path in sorted(folder.iterdir()):
                if path.is_file(): z.write(path,path.relative_to(ROOT))
        for filename in ("START_HERE.md","design_preview.png","print_layouts.png","geometry_validation.json","build_glider.py"):
            path=ROOT/filename
            if path.exists(): z.write(path,filename)
    print(json.dumps(report["assembly"],indent=2))
    print(f"Wrote {len(PARTS)} validated STL parts and {len(PLATES)} geometry-only 3MF plates.")


if __name__=="__main__":
    main()
