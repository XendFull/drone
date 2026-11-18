# WideView 350 — experimental glider prototype

Original design made for this project. **350 mm nominal wingspan, about 349 mm projected span after dihedral, 310 mm body length.** It is larger than the previously suggested 170 mm glider. Its broad wing, deep forward keel and tall tail provide surfaces visible from different directions, but a head-on/edge-on view can still be difficult. Camera visibility has not been measured.

**This is a printable prototype, not a flight-tested design.** Mesh checks do not establish glide performance, structural strength, successful recognition or drone contact. Establish the glide and camera performance separately. The rigid printed version is not a soft target.

## Which material?

**PLA is my preference for the thin wing panels because stiffness matters here. PETG is usable as a prototype with the files provided, but its thin panels can flex more.** Use your existing PETG for the small test plate before committing to the wings.

Material background: [Prusa PLA guide](https://help.prusa3d.com/article/pla_2062), [Prusa PETG guide](https://help.prusa3d.com/article/petg_2059).

## Print the small check first

Open `print_files/plate_0_TEST_FIRST.3mf` as geometry in your slicer. It contains a thin reinforced skin coupon, a 0.8 mm keel tab, and the actual wing saddle.

1. Choose your actual printer and its manufacturer-recommended PETG profile.
2. Use a **0.4 mm nozzle, 0.20 mm layer height AND 0.20 mm first-layer height**. The thin skin is deliberately one 0.20 mm layer. Turn off adaptive layers. A different first-layer height can omit or overbuild the skin.
3. Use 2 walls, 100% rectilinear infill, and 3 top/bottom layers. Thin geometry is bounded by the model; do not add thickness or shells with an external modifier. Inspect the sliced preview: the skin must be fully filled, with ribs continuing above it.
4. No supports or raft. The saddles are already oriented on their sides, so their shelves do not require support. Start without a brim, especially on a 180 mm bed.
5. Use your filament maker's temperatures and your printer's PETG cooling settings. Do not substitute a PLA temperature profile.
6. Use a PETG-compatible build surface. On smooth PEI, follow your sheet maker's release-layer guidance: PETG can bond strongly enough to damage the sheet. Let the bed cool and peel the coupon gently. [Prusa's surface guidance](https://help.prusa3d.com/article/print-surface-preparation_686951).
7. The coupon should release as a continuous film, and the 0.8 mm tab should slide into the saddle's 1.1 mm slot with a small glue clearance. Do not force it. Check dimensions/flow before printing the full set if it does not fit.

**Use the same settings on the four full plates. Keep 100% scale.** The 3MFs contain geometry and units only, not a machine-specific slicer project or G-code. If your slicer does not read them, import the individual STLs instead.

## Full print set

Every individual part fits within a clear **175 x 100 mm** footprint or smaller. Each pre-arranged plate fits within **180 x 180 mm**. Printer exclusion zones or bed clips may require rearrangement in a larger clear area. STLs use millimetres.

| Plate | Contents |
|---|---|
| `plate_1_right_wing.3mf` | One right wing half |
| `plate_2_left_wing.3mf` | One left wing half |
| `plate_3_body_and_connectors.3mf` | Front body, rear body with fin, wing saddle, tail saddle, TWO joint plates |
| `plate_4_tailplane.3mf` | One horizontal tailplane with fin-clearance notch |

There are 8 distinct flight-part STLs, with the joint plate printed twice: 9 physical flight parts. The spare saddle from the test plate can be used in the build if it fits.

The model's solid-volume estimate is **about 19.4 g in PETG before glue, tape and balancing material**. The unballasted design is tail-heavy; the computed starting estimate is roughly **7 g of nose ballast at 15 mm from the nose**. That makes approximately **26–27 g before adhesive and trim**. Do not automatically add 7 g: physically balance the actual build. Extrusion widths, material density and slicing change the result. These are mass estimates, not measured weights. Print time depends on your slicer and machine and has not been measured.

## Assembly

Use a small amount of adhesive explicitly suitable for PETG or PLA, following its instructions. Test the joint on the coupons first; smooth PETG is difficult to bond. Lightly scuff glue surfaces if the adhesive instructions recommend it. Heavy glue or large tape patches increase flying weight.

The nose is the rounded end of the deep body half. The tall triangular fin is the rear. Lengthwise positions below are measured **from the nose**, along the straight top of the body.

1. **Join the body.** Stand the front and rear profiles vertically, with their straight top edges aligned. They meet at 155 mm from the nose. Glue one 28 x 8 mm joint plate onto each side, centred across this butt joint. Keep the body straight while the adhesive cures. The plates fit below the top edge and must not twist the tail.
2. **Fit the main-wing saddle.** It is the wider saddle with a shallow V on top. Slide its underside slot onto the body's top edge. Its 12 mm chordwise length occupies **124–136 mm from the nose**. The shelf's HIGHER chordwise end faces the nose; it is about 0.42 mm higher than the rear end. The slot roof seats against the top edge of the body. Glue and hold square.
3. **Attach both main-wing halves.** Smooth bed faces go DOWN, ribs UP. The two straight 100 mm root edges meet on the body centreline. Root leading edges sit at **100 mm from the nose**, trailing edges at 200 mm. The smaller 62 mm clipped tips face outward. Glue only the root contact areas to the saddle. The saddle establishes **5 degrees dihedral per half** and approximately **2 degrees positive incidence**. At the same chord station, the tips should be about **15 mm higher** than the wing centre. Check left/right symmetry before the glue sets.
4. **Fit the tail saddle.** This is the smaller, flat-topped saddle. Seat it on the body's top edge over **237–247 mm from the nose**. It has no nose-up slope. Glue square to the body.
5. **Attach the horizontal tailplane.** Smooth bed face DOWN, rib UP. Its centre leading edge is at **230 mm from the nose**; the centre trailing edge is at 290 mm. The narrow notch opens REARWARD so the upright fin passes through it. Glue the centre forward area to the small saddle. Keep both tail tips at equal height.
6. **Balance.** The small notch in the underside of the front body marks **125 mm from the nose**, 25 mm behind the root leading edge. Support the assembled model at this longitudinal position on both sides. Add the minimum securely retained soft modelling clay near the nose until it balances approximately level or slightly nose-down. Keep ballast centred and recheck lateral balance. Do not fly it in its tail-heavy, unballasted condition.

The preview colours only distinguish parts; a multicolour printer is not needed. A consistent contrasting filament colour can help camera testing. Any added marking or covering must be included when weighing, balancing and collecting the new training images.

## First glide checks

Test without the drone, people or fragile objects in the flight corridor, over a forgiving landing surface. Use gentle hand launches from a repeatable height. Do not use a powered or elastic launcher for this prototype.

Check body alignment, mount orientation, balance and equal wing/tail angles before each change. Use small trim changes and record the resulting path. If it stalls, dives, twists, or the panels visibly flutter, stop and correct the airframe rather than throwing harder. Do not assume a flight speed or duration: measure multiple glides. A fully printed glider may need more speed than your drone can follow.

Take the new images only after the physical design and colour are stable. Cover the actual viewing angles, distances, lighting and motion blur of the planned demonstration. No existing CNN or tracking code was read or modified to produce this design.

## Lighter, softer wing option

For a slower-flight experiment, use the **same printed body and saddles with lightweight model-aircraft foam panels** instead of the printed wing and tail skins. `foam_templates/` contains 1:1 SVG outlines. Use light model foam such as 2–3 mm Depron-type sheet rather than dense craft foam. Cut two mirrored main-wing halves and one tailplane. The main-wing upper reinforcement ribs are not needed in the foam option. Keep the dihedral, incidence and longitudinal assembly positions above, and physically rebalance after substitution.

Print paper templates at **100% / actual size** and measure the 50 mm scale line before cutting. The tail's centre notch must clear the 0.8 mm vertical fin; cut it as a slit matching the actual fit rather than forcing the foam. First glide-test this variant separately; the foam option has not been flight-tested either.

## What was checked

- Every STL is a single connected, closed, consistently oriented mesh, with no degenerate faces.
- Plate dimensions and non-overlapping part bounding boxes were checked in millimetres.
- 3MF geometry is in millimetres, and the package contains no printer commands.
- Assembly views and print-bed layouts were rendered and visually reviewed.
- Estimated material volume, mass and unballasted centre of gravity were calculated.

No slicing on your printer, physical fit, structural test, aerodynamic simulation, real flight or camera detection test has been completed. `geometry_validation.json` contains the geometry results. `build_glider.py` regenerates this original design without accessing any detector or tracker files.

Background references: [NASA wing geometry](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/wing-geometry/), [NASA forces on a glider](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/three-forces-on-a-glider/). These explain the general principles; they do not validate this particular design.
