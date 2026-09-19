// js/face-index.js
// Uniform-grid acceleration for proximity / broad-phase queries against a
// STATIC face soup (the keep-out volume K). Needed once K can be an imported
// STL with thousands of triangles: the per-vertex clearance test in §4.1 and
// the swept-AABB broad phase of §3.1 were previously O(|F_K|) per vertex.
//
//   nearest(p)          -> { dist, sign, close, face, index } | null
//   queryAABB(lo, hi)   -> array of candidate face indices
//
// Duck-typed by geometry.signedClearance / geometry.enforceClearance.

import { pointTriInfo } from './geometry.js';

export class FaceIndex {
  constructor(faces, opts = {}) {
    this.faces = faces || [];
    const n = this.faces.length;
    this.aabbs = new Array(n);

    const lo = [Infinity, Infinity, Infinity];
    const hi = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < n; i++) {
      const f = this.faces[i];
      const flo = [
        Math.min(f.a[0], f.b[0], f.c[0]),
        Math.min(f.a[1], f.b[1], f.c[1]),
        Math.min(f.a[2], f.b[2], f.c[2]),
      ];
      const fhi = [
        Math.max(f.a[0], f.b[0], f.c[0]),
        Math.max(f.a[1], f.b[1], f.c[1]),
        Math.max(f.a[2], f.b[2], f.c[2]),
      ];
      this.aabbs[i] = { lo: flo, hi: fhi };
      for (let k = 0; k < 3; k++) {
        if (flo[k] < lo[k]) lo[k] = flo[k];
        if (fhi[k] > hi[k]) hi[k] = fhi[k];
      }
    }
    if (!n) for (let k = 0; k < 3; k++) ((lo[k] = 0), (hi[k] = 0));
    this.lo = lo;
    this.hi = hi;

    const ext = [
      Math.max(hi[0] - lo[0], 1e-9),
      Math.max(hi[1] - lo[1], 1e-9),
      Math.max(hi[2] - lo[2], 1e-9),
    ];
    const maxDim = opts.maxDim || 96;
    const perCell = opts.perCell || 2;
    let cell = Math.cbrt((ext[0] * ext[1] * ext[2] * perCell) / Math.max(n, 1));
    if (!isFinite(cell) || cell <= 0) cell = Math.max(ext[0], ext[1], ext[2]);
    cell = Math.max(cell, Math.max(ext[0], ext[1], ext[2]) / maxDim);
    this.cell = cell;
    this.dims = [
      Math.max(1, Math.min(maxDim, Math.ceil(ext[0] / cell))),
      Math.max(1, Math.min(maxDim, Math.ceil(ext[1] / cell))),
      Math.max(1, Math.min(maxDim, Math.ceil(ext[2] / cell))),
    ];

    this.cells = new Map();
    this.big = []; // faces spanning too many cells: always tested
    const bigSpan = opts.bigSpan || 1024;
    for (let i = 0; i < n; i++) {
      const a = this._clamped(this.aabbs[i].lo);
      const b = this._clamped(this.aabbs[i].hi);
      const span = (b[0] - a[0] + 1) * (b[1] - a[1] + 1) * (b[2] - a[2] + 1);
      if (span > bigSpan) {
        this.big.push(i);
        continue;
      }
      for (let ix = a[0]; ix <= b[0]; ix++)
        for (let iy = a[1]; iy <= b[1]; iy++)
          for (let iz = a[2]; iz <= b[2]; iz++) {
            const k = this._key(ix, iy, iz);
            let l = this.cells.get(k);
            if (!l) {
              l = [];
              this.cells.set(k, l);
            }
            l.push(i);
          }
    }

    this._seen = new Int32Array(n);
    this._stamp = 0;
  }

  _key(ix, iy, iz) {
    return (ix * this.dims[1] + iy) * this.dims[2] + iz;
  }
  _coord(p) {
    const c = this.cell;
    return [
      Math.floor((p[0] - this.lo[0]) / c),
      Math.floor((p[1] - this.lo[1]) / c),
      Math.floor((p[2] - this.lo[2]) / c),
    ];
  }
  _clamped(p) {
    const c = this._coord(p);
    return [
      Math.min(this.dims[0] - 1, Math.max(0, c[0])),
      Math.min(this.dims[1] - 1, Math.max(0, c[1])),
      Math.min(this.dims[2] - 1, Math.max(0, c[2])),
    ];
  }
  _maxRing(c) {
    const d = this.dims;
    return Math.max(
      Math.abs(0 - c[0]),
      Math.abs(d[0] - 1 - c[0]),
      Math.abs(0 - c[1]),
      Math.abs(d[1] - 1 - c[1]),
      Math.abs(0 - c[2]),
      Math.abs(d[2] - 1 - c[2])
    );
  }

  // Cells at Chebyshev distance exactly r from c, clipped to the grid.
  _shell(c, r, cb) {
    const d = this.dims;
    const x0 = Math.max(0, c[0] - r),
      x1 = Math.min(d[0] - 1, c[0] + r);
    const y0 = Math.max(0, c[1] - r),
      y1 = Math.min(d[1] - 1, c[1] + r);
    const z0 = Math.max(0, c[2] - r),
      z1 = Math.min(d[2] - 1, c[2] + r);
    if (x0 > x1 || y0 > y1 || z0 > z1) return;
    if (r === 0) {
      cb(this._key(x0, y0, z0));
      return;
    }
    for (let ix = x0; ix <= x1; ix++) {
      const ax = Math.abs(ix - c[0]) === r;
      for (let iy = y0; iy <= y1; iy++) {
        const ay = Math.abs(iy - c[1]) === r;
        if (ax || ay) {
          for (let iz = z0; iz <= z1; iz++) cb(this._key(ix, iy, iz));
        } else {
          const za = c[2] - r,
            zb = c[2] + r;
          if (za >= z0 && za <= z1) cb(this._key(ix, iy, za));
          if (zb >= z0 && zb <= z1) cb(this._key(ix, iy, zb));
        }
      }
    }
  }

  // Nearest surface feature, with the plane sign of the closest face.
  nearest(p) {
    const n = this.faces.length;
    if (!n) return null;
    this._stamp++;
    const stamp = this._stamp;
    let best = Infinity;
    let out = null;

    const test = (fi) => {
      if (this._seen[fi] === stamp) return;
      this._seen[fi] = stamp;
      const f = this.faces[fi];
      const info = pointTriInfo(p, f.a, f.b, f.c);
      const d = Math.sqrt(info.d2);
      if (d < best) {
        best = d;
        out = { dist: d, sign: info.sign, close: info.close, face: f, index: fi };
      }
    };

    for (const fi of this.big) test(fi);

    const c = this._coord(p); // unclamped: monotone ring bound stays valid
    const maxR = this._maxRing(c);
    for (let r = 0; r <= maxR; r++) {
      if ((r - 1) * this.cell >= best) break; // no closer face can remain
      this._shell(c, r, (key) => {
        const list = this.cells.get(key);
        if (!list) return;
        for (let m = 0; m < list.length; m++) test(list[m]);
      });
    }
    return out;
  }

  // Candidate face indices whose cells overlap [lo, hi].
  queryAABB(lo, hi) {
    const out = [];
    if (!this.faces.length) return out;
    this._stamp++;
    const stamp = this._stamp;
    for (const fi of this.big) {
      this._seen[fi] = stamp;
      out.push(fi);
    }
    for (let k = 0; k < 3; k++) if (lo[k] > this.hi[k] || hi[k] < this.lo[k]) return out;

    const a = this._clamped(lo),
      b = this._clamped(hi);
    for (let ix = a[0]; ix <= b[0]; ix++)
      for (let iy = a[1]; iy <= b[1]; iy++)
        for (let iz = a[2]; iz <= b[2]; iz++) {
          const list = this.cells.get(this._key(ix, iy, iz));
          if (!list) continue;
          for (let m = 0; m < list.length; m++) {
            const fi = list[m];
            if (this._seen[fi] === stamp) continue;
            this._seen[fi] = stamp;
            out.push(fi);
          }
        }
    return out;
  }
}