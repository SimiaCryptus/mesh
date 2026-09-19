// js/stl.js
// STL I/O for the mesh lab.
//   parseSTL(arrayBuffer)  -> { P, F }  (welded, orientation-normalized)
//   exportBinarySTL(P, F)  -> Blob      (binary STL, per-face normals)
//   downloadBlob(blob, name)
//
// Import notes:
//   * binary vs. ASCII is detected by the 84 + 50*n size identity, with a
//     printable-ASCII fallback heuristic;
//   * vertices are welded on exact coordinate equality (what CAD exporters
//     emit) so the result is an indexed mesh usable by geometry.js;
//   * triangle winding is made consistent with the stored facet normal, and
//     the whole mesh is flipped if its signed volume is negative, so that
//     pointTriInfo()'s plane sign means "outside = +".

import { cross, sub, dot, normalize } from './vec.js';

function decode(buffer) {
  return new TextDecoder('utf-8').decode(new Uint8Array(buffer));
}

function looksBinary(buffer) {
  if (buffer.byteLength < 84) return false;
  const dv = new DataView(buffer);
  const tris = dv.getUint32(80, true);
  if (84 + tris * 50 === buffer.byteLength) return true;

  const head = new Uint8Array(buffer, 0, Math.min(512, buffer.byteLength));
  let tag = '';
  for (let i = 0; i < 5 && i < head.length; i++) tag += String.fromCharCode(head[i]);
  if (tag.toLowerCase() !== 'solid') return true;
  for (let i = 5; i < head.length; i++) {
    const b = head[i];
    if (b === 0 || b > 126) return true; // non-printable => binary payload
  }
  return false;
}

function parseBinary(buffer) {
  const dv = new DataView(buffer);
  const tris = dv.getUint32(80, true);
  const out = [];
  let o = 84;
  for (let i = 0; i < tris && o + 50 <= buffer.byteLength; i++, o += 50) {
    const n = [dv.getFloat32(o, true), dv.getFloat32(o + 4, true), dv.getFloat32(o + 8, true)];
    const v = [];
    for (let k = 0; k < 3; k++) {
      const b = o + 12 + k * 12;
      v.push([dv.getFloat32(b, true), dv.getFloat32(b + 4, true), dv.getFloat32(b + 8, true)]);
    }
    out.push({ n, v });
  }
  return out;
}

function parseAscii(text) {
  const out = [];
  let n = [0, 0, 0];
  let v = [];
  for (const raw of text.split('\n')) {
    const line = raw.trim();
    if (!line) continue;
    if (line.startsWith('facet')) {
      const t = line.split(/\s+/);
      n = [+t[2] || 0, +t[3] || 0, +t[4] || 0];
      v = [];
    } else if (line.startsWith('vertex')) {
      const t = line.split(/\s+/);
      v.push([+t[1], +t[2], +t[3]]);
    } else if (line.startsWith('endfacet')) {
      if (v.length === 3) out.push({ n, v });
      v = [];
    }
  }
  return out;
}

export function signedVolume(P, F) {
  let vol = 0;
  for (const [ia, ib, ic] of F) {
    const a = P[ia],
      b = P[ib],
      c = P[ic];
    vol +=
      (a[0] * (b[1] * c[2] - b[2] * c[1]) +
        a[1] * (b[2] * c[0] - b[0] * c[2]) +
        a[2] * (b[0] * c[1] - b[1] * c[0])) /
      6;
  }
  return vol;
}

function buildIndexed(tris, { weld = true } = {}) {
  const P = [];
  const F = [];
  const map = new Map();
  const idxOf = (p) => {
    if (!weld) {
      P.push(p);
      return P.length - 1;
    }
    const k = `${p[0]},${p[1]},${p[2]}`;
    let i = map.get(k);
    if (i === undefined) {
      i = P.length;
      P.push(p);
      map.set(k, i);
    }
    return i;
  };

  for (const t of tris) {
    if (!t.v || t.v.length !== 3) continue;
    if (!t.v.every((p) => p.every(Number.isFinite))) continue;
    const a = idxOf(t.v[0]),
      b = idxOf(t.v[1]),
      c = idxOf(t.v[2]);
    if (a === b || b === c || c === a) continue; // degenerate
    const nw = cross(sub(P[b], P[a]), sub(P[c], P[a]));
    if (dot(nw, nw) < 1e-24) continue; // zero area
    if (t.n && dot(t.n, nw) < 0) F.push([a, c, b]);
    else F.push([a, b, c]);
  }

  // Make the global orientation outward (closed meshes).
  if (signedVolume(P, F) < 0) {
    for (const f of F) {
      const tmp = f[1];
      f[1] = f[2];
      f[2] = tmp;
    }
  }
  return { P, F };
}

export function parseSTL(buffer, opts = {}) {
  const tris = looksBinary(buffer) ? parseBinary(buffer) : parseAscii(decode(buffer));
  const mesh = buildIndexed(tris, opts);
  if (!mesh.F.length) throw new Error('no usable triangles found');
  return mesh;
}

// --- export --------------------------------------------------------------
export function exportBinarySTL(P, F, header = 'mesh') {
  const buf = new ArrayBuffer(84 + 50 * F.length);
  const dv = new DataView(buf);
  const bytes = new Uint8Array(buf);
  const h = String(header).slice(0, 79);
  for (let i = 0; i < h.length; i++) bytes[i] = h.charCodeAt(i) & 0x7f;
  dv.setUint32(80, F.length, true);

  let o = 84;
  for (const [ia, ib, ic] of F) {
    const a = P[ia],
      b = P[ib],
      c = P[ic];
    const n = normalize(cross(sub(b, a), sub(c, a)));
    dv.setFloat32(o, n[0], true);
    dv.setFloat32(o + 4, n[1], true);
    dv.setFloat32(o + 8, n[2], true);
    o += 12;
    for (const p of [a, b, c]) {
      dv.setFloat32(o, p[0], true);
      dv.setFloat32(o + 4, p[1], true);
      dv.setFloat32(o + 8, p[2], true);
      o += 12;
    }
    dv.setUint16(o, 0, true);
    o += 2;
  }
  return new Blob([buf], { type: 'model/stl' });
}

export function downloadBlob(blob, filename) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}