// Headless level verification for bridge-builder.html.
// Extracts the DOM-free physics block, builds a reference bridge for every level,
// checks it obeys the in-game build rules, and simulates the crossing.
//
//   node games/bridge-builder/tools/test-levels.mjs            # all levels
//   node games/bridge-builder/tools/test-levels.mjs harbour -v # one level, verbose
//   ... --dump designs.json                                   # save the reference bridges
import { readFileSync, writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const src = process.env.PHYSICS_SRC || join(here, '..', 'bridge-builder.html');
const text = readFileSync(src, 'utf8');
const block = text.slice(text.indexOf('// PHYSICS-START'), text.indexOf('// PHYSICS-END'));
const P = new Function(block + '\nreturn { MATERIALS, VEHICLES, LEVELS, buildSim, stepSim, designCost, inTerrain, segmentHitsTerrain, beamLength };')();
const { MATERIALS, LEVELS, buildSim, stepSim, designCost, inTerrain, segmentHitsTerrain, beamLength } = P;

// ---------- design builder ----------
function builder(lv) {
  const nodes = lv.anchors.map((a, i) => ({ id: 'a' + i, x: a[0], y: a[1], anchor: true }));
  const beams = [];
  const errors = [];
  let next = 1;
  const node = (x, y) => {
    let n = nodes.find(n => Math.abs(n.x - x) < 1e-6 && Math.abs(n.y - y) < 1e-6);
    if (!n) {
      if (!Number.isInteger(x) || !Number.isInteger(y)) errors.push(`off-grid joint (${x},${y})`);
      if (inTerrain(lv, x, y)) errors.push(`joint inside terrain (${x},${y})`);
      n = { id: 'n' + next++, x, y, anchor: false };
      nodes.push(n);
    }
    return n;
  };
  const D = {
    nodes, beams, errors,
    add(x1, y1, x2, y2, mat) {
      const A = node(x1, y1), B = node(x2, y2);
      if (A === B) return D;
      if (beams.some(b => (b.a === A.id && b.b === B.id) || (b.a === B.id && b.b === A.id))) return D;
      const L = beamLength(A, B);
      if (L > MATERIALS[mat].maxLen + 1e-6) errors.push(`${mat} too long ${L.toFixed(2)} (${x1},${y1})-(${x2},${y2})`);
      if (!lv.materials.includes(mat)) errors.push(`${mat} not allowed`);
      if (segmentHitsTerrain(lv, A, B)) errors.push(`member hits terrain (${x1},${y1})-(${x2},${y2})`);
      beams.push({ a: A.id, b: B.id, mat });
      return D;
    },
    chain(pts, mat) { for (let i = 0; i + 1 < pts.length; i++) D.add(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], mat); return D; },
    // road deck along y from x0 to x1 with joints every `step`
    deck(x0, x1, y, step, mat = 'road') { const xs = range(x0, x1, step); D.chain(xs.map(x => [x, y]), mat); return xs; },
    // truss under (depth<0) or over (depth>0) a deck: chord joints above/below interior deck joints, Pratt/Howe diagonals.
    truss(xs, y, depth, chordMat, webMat, opts = {}) {
      const cy = y + depth;
      const inner = xs.slice(1, -1);
      const chord = opts.fullChord ? xs : inner;
      D.chain(chord.map(x => [x, cy]), chordMat);
      for (const x of inner) D.add(x, y, x, cy, webMat);
      if (!opts.fullChord) { D.add(xs[0], y, inner[0], cy, opts.endMat || webMat); D.add(xs[xs.length - 1], y, inner[inner.length - 1], cy, opts.endMat || webMat); }
      else { D.add(xs[0], y, xs[0], cy, webMat); D.add(xs[xs.length - 1], y, xs[xs.length - 1], cy, webMat); }
      const mid = (xs[0] + xs[xs.length - 1]) / 2;
      for (let i = opts.fullChord ? 0 : 1; i + 1 < xs.length - (opts.fullChord ? 0 : 1); i++) {
        const xa = xs[i], xb = xs[i + 1];
        // diagonals slope toward the middle (Pratt for over-deck, Howe-like for under-deck)
        if ((xa + xb) / 2 < mid) D.add(xa, cy, xb, y, webMat); else D.add(xa, y, xb, cy, webMat);
      }
      return D;
    },
    design() { return { nodes, beams, nextId: next }; },
  };
  return D;
}
function range(a, b, step) { const r = []; for (let x = a; x < b - 1e-9; x += step) r.push(x); r.push(b); return r; }

// ---------- reference solutions ----------
// Truss between a top line and a bottom line sharing x positions. Diagonals slope down toward the
// nearest span centre so they work in tension. m = { top, bot, vert, diag } materials (top/bot optional).
function truss(D, xs, yTop, yBot, m, centers) {
  const yt = typeof yTop === 'function' ? yTop : () => yTop;
  const yb = typeof yBot === 'function' ? yBot : () => yBot;
  const pick = (v, x) => typeof v === 'function' ? v(x) : v;
  for (let i = 0; i + 1 < xs.length; i++) {
    const xa = xs[i], xb = xs[i + 1], xm = (xa + xb) / 2;
    if (m.top) D.add(xa, yt(xa), xb, yt(xb), pick(m.top, xm));
    if (m.bot) D.add(xa, yb(xa), xb, yb(xb), pick(m.bot, xm));
  }
  for (const x of xs) if (m.vert && Math.abs(yt(x) - yb(x)) > 1e-9 && !(m.skipEnds && (x === xs[0] || x === xs[xs.length - 1]))) D.add(x, yt(x), x, yb(x), pick(m.vert, x));
  for (let i = 0; i + 1 < xs.length; i++) {
    const xa = xs[i], xb = xs[i + 1], xm = (xa + xb) / 2;
    const c = centers.reduce((best, c) => Math.abs(c - xm) < Math.abs(best - xm) ? c : best, centers[0]);
    const mat = pick(m.diag, xm);
    if (m.cross) { D.add(xa, yt(xa), xb, yb(xb), mat); D.add(xa, yb(xa), xb, yt(xb), mat); }
    else if (xm < c) D.add(xa, yt(xa), xb, yb(xb), mat); else D.add(xa, yb(xa), xb, yt(xb), mat);
  }
}
const evens = (a, b) => range(a, b, 2);

const SOLUTIONS = {
  first(D) {
    const xs = D.deck(0, 8, 0, 2);
    truss(D, xs, 0, -2, { bot: 'wood', vert: 'wood', diag: 'wood', skipEnds: true }, [4]);
  },
  wider(D) {
    const xs = D.deck(0, 12, 0, 3);
    truss(D, xs, 0, -3, { bot: 'wood', vert: 'wood', diag: 'wood', skipEnds: true }, [6]);
  },
  pillar(D) {
    const xs = D.deck(0, 20, 0, 2);
    const nearSupport = x => x < 4 || (x > 6 && x < 14) || x > 16;
    truss(D, xs, 0, -2, { bot: x => (x > 6 && x < 14) ? 'steel' : 'wood', vert: x => (x >= 8 && x <= 12) ? 'steel' : 'wood', diag: x => nearSupport(x) ? 'steel' : 'wood', skipEnds: true }, [5, 15]);
    D.add(10, -4, 8, -2, 'steel').add(10, -4, 10, -2, 'steel').add(10, -4, 12, -2, 'steel');
  },

  uphill(D) {
    D.chain([[0, 0], [4, 1], [8, 2], [12, 3], [14, 3]], 'road');
    D.chain([[0, -3], [4, -2], [8, -1], [12, 0], [14, 0]], 'wood');
    D.add(4, 1, 4, -2, 'wood').add(8, 2, 8, -1, 'wood').add(12, 3, 12, 0, 'wood');
    D.add(0, 0, 4, -2, 'wood').add(4, 1, 8, -1, 'wood').add(8, 2, 12, 0, 'steel').add(12, 0, 14, 3, 'wood');
  },
  suspension(D) {
    D.deck(0, 24, 0, 2);
    for (let x = 2; x <= 12; x += 2) D.add(-1, 8, x, 0, 'cable');
    for (let x = 12; x <= 22; x += 2) D.add(25, 8, x, 0, 'cable');
  },
  gorge(D) {
    D.deck(0, 22, 0, 2);
    // braced-spandrel arch springing from the upper cliff anchors
    const arch = [[0, -6], [2, -4], [4, -3], [6, -2], [8, -1], [10, -1], [12, -1], [14, -1], [16, -2], [18, -3], [20, -4], [22, -6]];
    D.chain(arch, 'steel');
    for (const [x, y] of arch.slice(1, -1)) D.add(x, y, x, 0, 'steel');
    for (let i = 1; i + 2 < arch.length; i++) {
      const [xa, ya] = arch[i], [xb, yb] = arch[i + 1];
      if (xb <= 11) D.add(xa, 0, xb, yb, 'steel'); else D.add(xa, ya, xb, 0, 'steel');
    }
    D.add(0, 0, 2, -4, 'steel').add(22, 0, 20, -4, 'steel');
  },




  convoy(D) {
    const xs = D.deck(0, 18, 0, 2);
    const end = x => x < 5 || x > 13;
    truss(D, xs, 0, -2, { bot: 'wood', vert: x => end(x) ? 'steel' : 'wood', diag: x => end(x) ? 'steel' : 'wood', skipEnds: true }, [9]);
    truss(D, xs, -2, -4, { bot: 'steel', vert: x => end(x) ? 'steel' : 'wood', diag: x => end(x) ? 'steel' : 'wood', skipEnds: true }, [9]);
  },

  harbour(D) {
    D.deck(0, 26, 0, 2);
    // two-hinged truss arch (like the real one): lower chord springs from below the deck
    const low = [[0, -3], [1, -1], [2, 1], [4, 3], [6, 4], [8, 5], [10, 6], [12, 6], [14, 6], [16, 6], [18, 5], [20, 4], [22, 3], [24, 1], [25, -1], [26, -3]];
    const up = [[0, 0], [1, 2], [2, 3], [4, 5], [6, 6], [8, 7], [10, 8], [12, 8], [14, 8], [16, 8], [18, 7], [20, 6], [22, 5], [24, 3], [25, 2], [26, 0]];
    D.chain(low, 'steel'); D.chain(up, 'steel');
    const L = Object.fromEntries(low.map(([x, y]) => [x, y])), U = Object.fromEntries(up.map(([x, y]) => [x, y]));
    const xs = [1, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 25];
    const endW = x => (x < 5 || x > 21) ? 'steel' : 'wood';
    for (const x of xs) D.add(x, L[x], x, U[x], endW(x));
    for (let i = 0; i + 1 < xs.length; i++) { const a = xs[i], b = xs[i + 1]; if (a < 13) D.add(a, U[a], b, L[b], endW(a)); else D.add(a, L[a], b, U[b], endW(b)); }
    for (let x = 2; x <= 24; x += 2) D.add(x, L[x], x, 0, 'cable');
    D.add(0, 0, 1, -1, 'steel').add(26, 0, 25, -1, 'steel');
  },




  stayed(D) {
    const xs = [0, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 30];
    D.chain(xs.map(x => [x, 0]), 'road');
    for (const x of xs) if (x !== 15 && x >= 5 && x <= 25) D.add(15, 12, x, 0, 'cable');
    D.add(15, -2, 15, 0, 'steel').add(0, -2, 3, 0, 'steel').add(30, -2, 27, 0, 'steel');
  },
  storm(D) {
    const xs = D.deck(0, 24, 0, 2);
    for (let x = 2; x <= 12; x += 2) D.add(-1, 8, x, 0, 'cable');
    for (let x = 12; x <= 22; x += 2) D.add(25, 8, x, 0, 'cable');
    truss(D, xs, 0, -2, { bot: 'wood', vert: 'wood', diag: 'wood', skipEnds: true }, [12]);
  },
  cantilever(D) {
    const xs = D.deck(0, 32, 0, 2);
    truss(D, xs, 0, -3, { bot: 'steel', vert: 'steel', diag: 'steel', skipEnds: true }, [5, 16, 27]);
    for (const px of [10, 22]) {
      D.add(px, -5, px - 2, -3, 'steel').add(px, -5, px, -3, 'steel').add(px, -5, px + 2, -3, 'steel');
      // cantilever towers above the piers
      truss(D, [px - 6, px - 4, px - 2, px, px + 2, px + 4, px + 6], 0, x => 4 - Math.abs(x - px) * 0.5, { top: 'steel', vert: 'wood', diag: 'wood', skipEnds: true }, [px - 9, px + 9]);
    }
  },

  clifton(D) {
    const xs = D.deck(0, 28, 0, 2);
    const cab = [[-1, 8], [2, 6], [4, 5], [6, 4], [8, 3], [10, 2], [12, 2], [14, 2], [16, 2], [18, 2], [20, 3], [22, 4], [24, 5], [26, 6], [29, 8]];
    D.chain(cab, 'cable');
    for (const [x, y] of cab.slice(1, -1)) D.add(x, y, x, 0, 'cable');
    for (const x of [2, 4, 6, 8, 10, 12]) { D.add(-1, 8, x, 0, 'cable'); D.add(29, 8, 28 - x, 0, 'cable'); }
    truss(D, xs.slice(1, -1), 0, -2, { bot: 'steel', vert: 'steel', diag: 'steel' }, [14]);
    D.add(0, 0, 2, -2, 'steel').add(28, 0, 26, -2, 'steel');
  },




  tower(D) {
    D.deck(0, 30, 0, 2);
    truss(D, [0, 2, 4, 6], 0, -2, { bot: 'steel', vert: 'steel', diag: 'steel', skipEnds: true }, [3]);
    truss(D, [24, 26, 28, 30], 0, -2, { bot: 'steel', vert: 'steel', diag: 'steel', skipEnds: true }, [27]);
    D.add(6, -2, 9, -2, 'steel').add(24, -2, 21, -2, 'steel').add(6, -2, 8, 0, 'steel').add(24, -2, 22, 0, 'steel');
    D.add(9, -2, 8, 0, 'steel').add(9, -2, 10, 0, 'steel').add(21, -2, 20, 0, 'steel').add(21, -2, 22, 0, 'steel');
    for (const x of [10, 12, 14, 16]) D.add(9, 8, x, 0, 'cable');
    for (const x of [14, 16, 18, 20]) D.add(21, 8, x, 0, 'cable');
    // high-level walkway truss between the towers stiffens the centre span
    truss(D, [10, 12, 14, 16, 18, 20], 2, 0, { top: 'steel', vert: 'steel', diag: 'steel' }, [15]);
  },


  strait(D) {
    const xs = D.deck(0, 36, 0, 2);
    for (let x = 0; x <= 18; x += 2) if (x !== 8) D.add(8, 12, x, 0, 'cable');
    for (let x = 18; x <= 36; x += 2) if (x !== 28) D.add(28, 12, x, 0, 'cable');
    D.add(8, -2, 8, 0, 'steel').add(28, -2, 28, 0, 'steel');
    truss(D, xs.slice(1, -1), 0, -2, { bot: 'steel', vert: 'steel', diag: 'steel' }, [4, 18, 32]);
    D.add(0, 0, 2, -2, 'steel').add(36, 0, 34, -2, 'steel').add(0, -3, 2, -2, 'steel').add(36, -3, 34, -2, 'steel');
  },

  sandbox(D) {
    const xs = D.deck(0, 40, 0, 2);
    truss(D, xs, 0, -3, { bot: 'steel', vert: 'steel', diag: 'steel', skipEnds: true }, [10, 30]);
    truss(D, xs, -3, -6, { bot: 'steel', vert: 'steel', diag: 'steel', skipEnds: true }, [10, 30]);
    D.add(20, -6, 18, -3, 'steel').add(20, -6, 22, -3, 'steel');
    for (let x = 2; x <= 10; x += 2) D.add(-1, 10, x, 0, 'cable');
    for (let x = 30; x <= 38; x += 2) D.add(41, 10, x, 0, 'cable');
  },
  rush(D) {
    const xs = D.deck(0, 34, 0, 2);
    truss(D, xs, 0, -3, { bot: 'steel', vert: 'steel', diag: 'steel', skipEnds: true }, [8, 26]);
    D.add(17, -4, 16, -3, 'steel').add(17, -4, 18, -3, 'steel');
    for (let x = 2; x <= 12; x += 2) D.add(-1, 8, x, 0, 'cable');
    for (let x = 22; x <= 32; x += 2) D.add(35, 8, x, 0, 'cable');
  },
};

// Designs that must FAIL, proving a level's twist actually matters.
const MUST_FAIL = {
  storm(D) { // the plain Level-5 fan with no stiffening truss gallops apart in the gusts
    D.deck(0, 24, 0, 2);
    for (let x = 2; x <= 12; x += 2) D.add(-1, 8, x, 0, 'cable');
    for (let x = 12; x <= 22; x += 2) D.add(25, 8, x, 0, 'cable');
  },
  harbour(D) { // a pin-jointed arch without bracing folds up
    D.deck(0, 26, 0, 2);
    const arch = [[0, -3], [1, -1], [2, 1], [4, 3], [6, 5], [8, 6], [10, 7], [12, 8], [14, 8], [16, 7], [18, 6], [20, 5], [22, 3], [24, 1], [25, -1], [26, -3]];
    D.chain(arch, 'steel');
    for (const [x, y] of arch.slice(2, -2)) D.add(x, y, x, 0, 'cable');
  },
};

// ---------- runner ----------
const woodCap = (f, L) => f >= 0 ? MATERIALS.wood.strength : MATERIALS.wood.strength * Math.min(1, (3 / L) ** 2);
function run(lv, design, verbose, peaks) {
  const sim = buildSim(lv, design);
  const dt = 1 / 60;
  const nOrig = design.beams.length;
  while (!sim.result && sim.time < 200) {
    stepSim(sim, dt, 40);
    if (peaks) for (let i = 0; i < nOrig; i++) { const b = sim.beams[i]; if (!b.broken) peaks[i] = Math.max(peaks[i] || 0, Math.abs(b.force) / woodCap(b.force, b.L0)); }
    if (verbose) for (const ev of sim.events) {
      if (ev.type === 'break') {
        const b = sim.beams[ev.i];
        const A = sim.nodes[b.a], B = sim.nodes[b.b];
        console.log(`  t=${sim.time.toFixed(2)} BREAK ${b.mat} (${A.x.toFixed(0)},${A.y.toFixed(0)})-(${B.x.toFixed(0)},${B.y.toFixed(0)}) L=${b.L0.toFixed(2)} force=${(b.force / 1000).toFixed(1)}kN cap=${((b.force >= 0 ? b.strength : b.comp) / 1000).toFixed(1)}kN at (${ev.x.toFixed(1)},${ev.y.toFixed(1)})`);
      }
    }
    sim.events.length = 0;
    if (verbose && Math.round(sim.time * 60) % 60 === 0) {
      const v = sim.vehicles.map(v => `${v.V.key}:${v.state}@${v.x.toFixed(1)}`).join(' ');
      console.log(`  t=${sim.time.toFixed(1)} maxR=${sim.maxRatio.toFixed(2)} broken=${sim.brokenCount} ${v}`);
    }
  }
  return sim;
}

// Swap steel members to wood wherever wood would have coped (with margin), then re-verify.
function optimize(lv, design) {
  let cur = design;
  for (let iter = 0; iter < 8; iter++) {
    const peaks = [];
    const sim = run(lv, cur, false, peaks);
    if (!sim.result || !sim.result.ok) return null;
    const next = { ...cur, beams: cur.beams.map((b, i) => (b.mat === 'steel' && peaks[i] < 0.72) ? { ...b, mat: 'wood' } : b) };
    if (next.beams.every((b, i) => b.mat === cur.beams[i].mat)) return cur;
    const check = run(lv, next, false);
    if (!check.result || !check.result.ok) {
      // too aggressive: only downgrade the clearly idle members
      const safer = { ...cur, beams: cur.beams.map((b, i) => (b.mat === 'steel' && peaks[i] < 0.45) ? { ...b, mat: 'wood' } : b) };
      const c2 = run(lv, safer, false);
      return (c2.result && c2.result.ok) ? safer : cur;
    }
    cur = next;
  }
  return cur;
}

const args = process.argv.slice(2);
const verbose = args.includes('-v');
const noOpt = args.includes('--no-opt');
const dumpAt = args.indexOf('--dump');
const dumpFile = dumpAt >= 0 ? args[dumpAt + 1] : null;
const only = args.filter((a, i) => !a.startsWith('-') && i !== dumpAt + 1);
const dumped = {};
let failures = 0;
for (const lv of LEVELS) {
  if (only.length && !only.includes(lv.id)) continue;
  const sol = SOLUTIONS[lv.id];
  if (!sol) { console.log(`${lv.id.padEnd(11)} FAIL (no reference solution)`); failures++; continue; }
  const D = builder(lv);
  sol(D);
  let design = D.design();
  const raw = designCost(design);
  if (!noOpt) { const o = optimize(lv, design); if (o) design = o; }
  dumped[lv.id] = design;
  const cost = designCost(design);
  const t0 = Date.now();
  const sim = run(lv, design, verbose);
  const ms = Date.now() - t0;
  const finite = isFinite(lv.budget);
  const thresholdsOk = !finite || (lv.star3 < lv.star2 && lv.star2 < lv.budget);
  const ok = sim.result && sim.result.ok && D.errors.length === 0 && cost <= lv.budget && thresholdsOk;
  if (!ok) failures++;
  const stars = !finite ? '-' : cost <= lv.star3 ? '3' : cost <= lv.star2 ? '2' : '1';
  console.log(`${lv.id.padEnd(11)} ${ok ? 'PASS' : 'FAIL'} ref $${cost}${raw !== cost ? ` (from $${raw})` : ''} / budget ${finite ? '$' + lv.budget : 'none'}` +
    (finite ? ` [2* <= $${lv.star2}, 3* <= $${lv.star3}] -> ${stars}*` : '') +
    ` | peak ${(sim.maxRatio * 100).toFixed(0)}% broken ${sim.brokenCount} members ${design.beams.length} t=${sim.time.toFixed(1)}s ${ms}ms` +
    (sim.result && !sim.result.ok ? ` -> ${sim.result.why}` : '') + (thresholdsOk ? '' : ' (bad star thresholds)') +
    (D.errors.length ? `\n   rule errors: ${D.errors.slice(0, 6).join('; ')}` : ''));
  if (MUST_FAIL[lv.id]) {
    const N = builder(lv); MUST_FAIL[lv.id](N);
    const ns = run(lv, N.design(), false);
    const held = ns.result && ns.result.ok;
    if (held || N.errors.length) failures++;
    console.log(`${''.padEnd(11)} ${held || N.errors.length ? 'FAIL' : 'PASS'} naive design correctly fails: ${ns.result ? ns.result.why || 'it held!' : 'no result'}`);
  }
}
if (dumpFile) writeFileSync(dumpFile, JSON.stringify(dumped));
console.log(failures ? `\n${failures} problem(s)` : '\nAll levels verified.');
process.exit(failures ? 1 : 0);
