# Constrained Mesh Enclosure Lab

> Optimize a closed surface mesh around a keep-out constraint volume.

A deformable triangulated mesh is driven by gradient descent to extremize
geometric fitness functionals (area, volume, edge-length entropy, angular
regularity, Laplacian smoothness) while a hard, exact continuous
collision-detection (CCD) pipeline guarantees the mesh never intrudes into an
inner constraint mesh `K`, and (approximately) never intersects itself.

This is the mesh analogue of a shrink-wrap: the outer surface flows downhill
on its energy landscape, but every step is filtered through CCD that clips
motion at the moment any vertex would violate the keep-out geometry.

See [`idea.md`](./idea.md) for the full concept, [`math_outline.md`](./math_outline.md)
for the formulas/derivations, and [`novelty_utility.md`](./novelty_utility.md)
/ [`use_cases.md`](./use_cases.md) for context and applications.

## Running

This is a single-page, dependency-light web app. Because it uses ES modules,
serve the directory over HTTP rather than opening `index.html` from `file://`:

```sh
# from the repository root
python3 -m http.server 8000
# then open http://localhost:8000/index.html
```

Any static file server works. TensorFlow.js is bundled locally
(`js/tf.min.js`) and provides the autodiff gradients of the soft energies;
the collision pipeline is hand-rolled plain JS.

## Controls

- **Run / Step / Reset** — start/pause the optimization loop, take a single
  step, or rebuild the initial meshes.
- **Optimizer** — Adam, L-BFGS, or QQN. Different optimizers select different
  converged enclosures at equal fitness (the "fingerprinting" effect).
- **Learn rate / Steps per frame** — step size and how many optimization
  steps run per rendered frame.
- **Energy weights** — the λ sliders for area, volume, fidelity, length
  entropy, angular quality, and Laplacian smoothness. **Target Vol** sets the
  volume-matching target `V*`.
- **δ_safe** — outward clearance kept from the keep-out mesh `K`.
- **Self-collision (approx)** — enable an approximate point-vs-face
  self-collision pass.
- **Reset optimizer on truncation** — reset stateful optimizer curvature
  across a projection discontinuity (recommended).
- **Retriangulate (edge flips)** — one conservative Delaunay-style edge-flip
  pass outside the constraint ε-shell.

### Viewport

- **Drag** to orbit.
- **Mouse wheel** to zoom in/out.

## Metrics

Live readouts include surface area, signed volume, edge-length entropy (vs.
its `ln(#edges)` ceiling), angular energy, Laplacian energy, minimum
clearance to `K`, active contacts, step/reject counts, and the max trust
radius.

## Files

```
index.html               # Single-file app: viewport, controls, loop
js/
  geometry.js            # Mesh generation + topology + clearance helpers
  mesh-energy.js         # Soft energy functionals (tfjs autodiff)
  ccd.js                 # Broad + narrow phase continuous collision
  resolve.js             # Projection & delta-scaling resolution
  trust.js               # Per-vertex trust radius bookkeeping
  retriangulate.js       # Conservative connectivity edits
  optimizer-adam.js      # Optimizers (shared with sibling labs)
  optimizer-lbfgs.js
  optimizer-qqn.js
  vec.js                 # 3-vector helpers
  tf.min.js              # TensorFlow.js (autodiff)
```

## Math report

Symbolic derivations and formula checks live in `math_report.mac` (Maxima).
Regenerate the log with:

```sh
./run_report.sh   # runs: maxima -b math_report.mac | tee math_report.log
```

## License

Part of the experiments collection. See the repository root for license
details.