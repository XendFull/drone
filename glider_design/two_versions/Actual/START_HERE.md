# Actual — WideView 350, concealed-ballast version

This is a complete print set for ONE glider. Print the separate Actual set and Decoy set to build the pair. Each glider is modular across five 180 x 180 mm plates; it is not a one-piece print. The original v1 files are preserved separately.

## What is different?

Actual and Decoy have identical designed exterior surfaces and equal modelled amounts of the same material. They use the same airframe parts plus two side pods on the forward body. The body closes each pod's open side, hiding its fixed printed ballast. The ballast in Decoy is 10 mm farther forward inside the pod than in Actual. There is no sliding weight, extra metal, external class label, or different outline.

The calculated Actual balance point is **125.00 mm from the nose**. This is a material-volume calculation, not a measured or approved flight limit. Both versions need individual weighing, balance checks and glide testing. The names Actual/Decoy are experiment labels; different or recognisable flight paths are not guaranteed.

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
| 5 | TWO concealed-ballast side pods for Actual |

That is 11 physical flight parts per glider: 9 base airframe pieces and 2 pods. The one pod STL is printed twice. Both open faces go inward, against the body. The seam side is hidden after assembly.

## Assembly

Follow the included `BASE_AIRFRAME_ASSEMBLY.md` for the body, wings, tail and saddles. **Replace its old nose-clay balancing step with the pod instructions here. Do not add the original 7 g nose ballast.** These versions already include calculated printed ballast.

1. Assemble the base airframe straight and square. The nose is the rounded end, the triangular fin is the rear. Use equal, small adhesive quantities on the two models.
2. Orient each pod with its LONG axis along the body. One long outside wall is unbroken and smooth; the opposite face is open and reveals the printed ballast. Keep that open face against the body.
3. For this Actual set, the pod's outside face reaches 4 mm out from each side of the body. Fit ONE pod on each side, aligned to the SAME fore-aft position. Each pod starts **15 mm from the nose** and ends at 100 mm. Its upper edge is **4 mm below the body's straight top edge**; its lower edge is 22 mm below it. The 0.8 mm body profile serves as the closing plate between the two pods.
4. The asymmetric internal block identifies the nose end BEFORE sealing. In Actual, the full-height internal block starts **14.00 mm from the pod's NOSE end**. Verify against the cutaway image and measure it before gluing. Flipping a pod end-for-end changes the balance; do not do that. Mirror left/right only.
5. Bond the open perimeter lips to the body with a thin, continuous bond line and a compatible adhesive. The concealed ballast is integral with the pod and cannot rattle or slide. Do not glue until you have checked its orientation and the adhesive on a small sample. Check cured retention before flight.
6. Weigh the completed glider and physically measure its balance point. Modelled mass in PETG is approximately **30.21 g before adhesive/finish**. Actual extrusion and glue can change mass and balance. The Actual/Decoy modelled masses match, but printed weights are not guaranteed equal.
7. For exterior matching, use the same colour/finish, seams and assembly methods on both. Put any experiment identifier on a removable storage tag or log, not a different visible marking on one model. The preview's captions are not embossed into the parts.

## Flight and presentation limits

Test each glider independently with gentle hand launches over a forgiving landing surface and without the drone in the corridor. These are unpowered rigid printed prototypes. No safe CG envelope or usable trajectory difference has been established. Stop if either stalls, dives, tumbles, sheds a pod or shows panel flutter. Do not use an elastic/powered launcher.

Compare repeat launches from the same point and height, with the same launch procedure and lighting. Measure the real trajectories before training a motion classifier. Use separate flights for evaluation and randomise the order. The drone camera's own movement and distance can mimic changes in apparent speed; a stable external camera is useful for ground-truth measurement, even if the demonstration uses only the drone's camera.

The exact outside geometry is the same by design. Physical appearance can still differ through print defects, translucent material, glue, wear, or inconsistent finishing. Equal geometry and mass do not guarantee recognisable flight behaviour.

## Checks completed

Closed connected mesh, winding, nondegenerate triangles, export round-trip, bed-fit and non-overlap checks were performed. Both pod variants have matching modelled volume and exterior surfaces; only their hidden geometry differs. Structural retention, printing, real mass, glide stability and recognition have not been physically tested.
