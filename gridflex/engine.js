/* Adaptive GridFlex: simulation + constraint-aware dispatch engine.
 *
 * Radial 11 kV feeder, five buses, 5-minute steps (288 per day).
 * Two controllers run side by side on the same physical day:
 *   - "baseline": legacy behaviour. Fixed-schedule flexible loads, tariff-driven batteries,
 *     and inverter curtailment as the only answer to a network violation.
 *   - "gridflex": forecast-driven planning of flexible loads and storage, then a real-time
 *     constraint solver that relieves each violation with the cheapest effective resource
 *     (flexible load, then battery, then curtailment as a last resort).
 *
 * Plain JS, no dependencies. Works in the browser (window.GridFlex) and in Node (module.exports).
 */
(function (root) {
  'use strict';

  const STEPS = 288, DT = 1 / 12; // 5-minute steps, hours per step

  // ---------- helpers ----------
  function rng(seed) {
    let s = seed >>> 0;
    return function () {
      s = (s + 0x6D2B79F5) >>> 0; let t = s;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  const gauss = r => Math.sqrt(-2 * Math.log(r() + 1e-12)) * Math.cos(2 * Math.PI * r());
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  const bump = (h, c, w) => Math.exp(-Math.pow((h - c) / w, 2));
  const hourOf = k => k * DT;

  // ---------- network ----------
  // Line k connects bus k-1 -> bus k. s = voltage sensitivity (pu per MW of flow).
  const NET = {
    v0: 1.01,                       // substation OLTC setpoint (pu)
    vmin: 0.95, vmax: 1.05,
    lines: [null,
      { id: 'L1', amp: 24, s: 0.0012 },
      { id: 'L2', amp: 20, s: 0.0016 },
      { id: 'L3', amp: 14, s: 0.0018 },
      { id: 'L4', amp: 14, s: 0.0022 }],
    trafo: { rating: 20, reverse: 12 }, // MVA rating; reverse-power relay setting (MW)
    buses: ['Substation', 'Town centre', 'Industrial park', 'Solar hub', 'Wind end']
  };
  const MARGIN_V = 0.002, MARGIN_I = 0.97; // optimizer keeps a little headroom inside hard limits

  const GENS = [
    { id: 'solar', name: 'Solar plant', bus: 3, cap: 15 },
    { id: 'wind', name: 'Wind farm', bus: 4, cap: 12 }
  ];
  const FLEX = [
    { id: 'cold', name: 'Cold-storage hub', bus: 2, pmin: 0.3, pmax: 1.6, energy: 12, win: [0, 24],
      base: h => 0.5, note: 'Thermal mass, any time of day' },
    { id: 'ev', name: 'E-bus depot', bus: 3, pmin: 0, pmax: 3, energy: 13, win: [9, 22],
      base: h => (h >= 17 && h < 17 + 13 / 3 ? 3 : 0), note: 'Buses between shifts 09:00-22:00' },
    { id: 'pump', name: 'Irrigation pumping', bus: 4, pmin: 0, pmax: 1.2, energy: 7.2, win: [0, 24],
      base: h => (h < 6 ? 1.2 : 0), note: 'Reservoir fill, any time of day' }
  ];
  const BESS = [
    { id: 'A', name: 'BESS-A', bus: 2, pmax: 4, cap: 12, soh: 0.962, soc0: 0.45, eta: 0.97, degCost: 22, cycles0: 410 },
    { id: 'B', name: 'BESS-B', bus: 4, pmax: 6, cap: 20, soh: 0.908, soc0: 0.40, eta: 0.965, degCost: 34, cycles0: 1480 }
  ];
  const SOC_MIN = 0.10, SOC_MAX = 0.90;

  // ---------- scenarios ----------
  const SCENARIOS = {
    sunny:   { name: 'Clear spring day', desc: 'Strong midday solar, light demand: classic export congestion at the solar hub.', start: 8 },
    windy:   { name: 'Windy & bright', desc: 'Overnight wind plus midday sun: surplus at both feeder ends.', start: 3 },
    cloudy:  { name: 'Broken cloud', desc: 'Fast solar ramps that forecasts only partly capture.', start: 9 },
    heatwave:{ name: 'Heat wave', desc: 'High cooling demand, hot batteries, evening import peak.', start: 12 }
  };

  function makeDay(scenario, seed) {
    const r = rng(seed);
    const solar = new Float64Array(STEPS), wind = new Float64Array(STEPS);
    const load = [0, 1, 2, 3, 4].map(() => new Float64Array(STEPS));
    const ci = new Float64Array(STEPS), price = new Float64Array(STEPS);
    const hot = scenario === 'heatwave';
    let cloud = 0, cloudDip = 0, w = scenario === 'windy' ? 0.15 : 0;
    for (let k = 0; k < STEPS; k++) {
      const h = hourOf(k) + DT / 2;
      // solar
      const shape = h < 6.2 || h > 18.8 ? 0 : Math.pow(Math.sin(Math.PI * (h - 6.2) / 12.6), 1.25);
      let cf;
      if (scenario === 'cloudy') {
        cloud = 0.92 * cloud + 0.39 * gauss(r);
        if (cloudDip > 0) cloudDip--; else if (r() < 0.035) cloudDip = 2 + Math.floor(r() * 7);
        cf = clamp(0.78 + 0.12 * cloud - (cloudDip > 0 ? 0.45 : 0), 0.15, 1);
      } else cf = (hot ? 0.9 : 0.97) + 0.015 * gauss(r);
      solar[k] = Math.max(0, GENS[0].cap * shape * cf);
      // wind (random walk around a diurnal mean)
      let mean;
      if (scenario === 'windy') mean = 0.58 + 0.3 * Math.cos(2 * Math.PI * (h - 3) / 24);
      else if (scenario === 'heatwave') mean = 0.12 + 0.1 * bump(h, 17, 3);
      else mean = 0.26 + 0.12 * Math.cos(2 * Math.PI * (h - 4) / 24);
      w = 0.94 * w + 0.06 * 0 + 0.05 * gauss(r);
      wind[k] = GENS[1].cap * clamp(mean + w, 0, 0.96);
      // fixed loads
      let town = 3.2 + 1.4 * bump(h, 8, 1.5) + 4.2 * bump(h, 19.5, 2.2) + 0.6 * bump(h, 13, 3);
      let ind = 1.6 + (h >= 7 && h < 19 ? 3.2 * (1 - 0.18 * bump(h, 12.5, 0.6)) : 0);
      if (hot) { town = town * 1.15 + 2.6 * bump(h, 15.5, 3.2); ind *= 1.1; }
      load[1][k] = Math.max(0, town * (1 + 0.025 * gauss(r)));
      load[2][k] = Math.max(0, ind * (1 + 0.03 * gauss(r)));
      load[3][k] = 0.6 + 0.05 * gauss(r);
      load[4][k] = 0.4 + 0.04 * gauss(r);
      // grid import carbon intensity (tCO2/MWh) and price ($/MWh)
      ci[k] = 0.58 + 0.24 * bump(h, 19.5, 2.4) + 0.1 * (h < 6 ? 1 : 0) - 0.08 * bump(h, 13, 3);
      price[k] = 55 + 95 * bump(h, 19.5, 2.2) + 25 * bump(h, 8.5, 1.5) - 20 * bump(h, 13, 3) + (hot ? 40 * bump(h, 17, 3) : 0);
    }
    return { scenario, seed, solar, wind, load, ci, price, ambient: hot ? 34 : 26,
      ampFactor: [null, 1, 1, 1, 1].map(x => x === null ? null : new Float64Array(STEPS).fill(1)) };
  }

  // ---------- forecasts ----------
  // Re-issued every hour from "now": AR(1) errors that widen with lead time.
  function makeForecast(day, from, seed) {
    const r = rng(seed);
    const fc = { from, solar: new Float64Array(STEPS), wind: new Float64Array(STEPS),
      load: [0, 1, 2, 3, 4].map(() => new Float64Array(STEPS)), lo: new Float64Array(STEPS), hi: new Float64Array(STEPS) };
    let es = 0, ew = 0, el = 0;
    for (let k = 0; k < STEPS; k++) {
      if (k <= from) {
        fc.solar[k] = day.solar[k]; fc.wind[k] = day.wind[k];
        for (let b = 1; b <= 4; b++) fc.load[b][k] = day.load[b][k];
        fc.lo[k] = fc.hi[k] = day.solar[k] + day.wind[k];
        continue;
      }
      const lead = k - from;
      const sig = 0.03 + 0.13 * Math.min(1, lead / 96);
      es = 0.93 * es + 0.37 * gauss(r); ew = 0.95 * ew + 0.31 * gauss(r); el = 0.9 * el + 0.44 * gauss(r);
      // the forecaster cannot see injected cloud events: use the un-dimmed solar if present
      const sTrue = day.solarClear ? day.solarClear[k] : day.solar[k];
      fc.solar[k] = Math.max(0, sTrue * (1 + sig * es));
      fc.wind[k] = clamp(day.wind[k] * (1 + 1.3 * sig * ew), 0, GENS[1].cap);
      for (let b = 1; b <= 4; b++) fc.load[b][k] = day.load[b][k] * (1 + 0.4 * sig * el);
      const tot = fc.solar[k] + fc.wind[k], band = 1.28 * sig * Math.sqrt(fc.solar[k] ** 2 + 1.7 * fc.wind[k] ** 2);
      fc.lo[k] = Math.max(0, tot - band); fc.hi[k] = tot + band;
    }
    return fc;
  }

  // ---------- network evaluation ----------
  function evalNet(net, k, day) {
    const F = [0, 0, 0, 0, 0], V = [NET.v0, 0, 0, 0, 0];
    let acc = 0;
    for (let b = 4; b >= 1; b--) { acc += net[b]; F[b] = acc; }
    for (let i = 1; i <= 4; i++) V[i] = V[i - 1] - NET.lines[i].s * F[i];
    const amp = [0, 1, 2, 3, 4].map(i => i === 0 ? 0 : NET.lines[i].amp * (day ? day.ampFactor[i][k] : 1));
    return { F, V, amp, trafo: F[1] };
  }
  // dV_i / d(consumption at bus b)
  function vSens(i, b) { let s = 0; for (let j = 1; j <= Math.min(i, b); j++) s += NET.lines[j].s; return s; }

  // Find the worst violation. dir +1 = need more consumption downstream (export problem),
  // dir -1 = need less consumption (import problem). need = amount in constraint units.
  function worstViolation(r, hard) {
    const vmax = hard ? NET.vmax : NET.vmax - MARGIN_V, vmin = hard ? NET.vmin : NET.vmin + MARGIN_V;
    const mi = hard ? 1 : MARGIN_I;
    let worst = null, score = 1e-6;
    const consider = (v) => { if (v.mw > score) { score = v.mw; worst = v; } };
    for (let i = 1; i <= 4; i++) {
      if (r.V[i] > vmax) consider({ kind: 'V', idx: i, dir: +1, need: r.V[i] - vmax, mw: (r.V[i] - vmax) / vSens(i, 4), sens: b => vSens(i, b), loc: i });
      if (r.V[i] < vmin) consider({ kind: 'V', idx: i, dir: -1, need: vmin - r.V[i], mw: (vmin - r.V[i]) / vSens(i, 4), sens: b => vSens(i, b), loc: i });
      const lim = r.amp[i] * mi;
      if (-r.F[i] > lim) consider({ kind: 'L', idx: i, dir: +1, need: -r.F[i] - lim, mw: -r.F[i] - lim, sens: b => (b >= i ? 1 : 0), loc: i });
      if (r.F[i] > lim) consider({ kind: 'L', idx: i, dir: -1, need: r.F[i] - lim, mw: r.F[i] - lim, sens: b => (b >= i ? 1 : 0), loc: i });
    }
    const rev = NET.trafo.reverse * mi, fwd = NET.trafo.rating * mi;
    if (-r.F[1] > rev) consider({ kind: 'T', idx: 0, dir: +1, need: -r.F[1] - rev, mw: -r.F[1] - rev, sens: b => 1, loc: 1 });
    if (r.F[1] > fwd) consider({ kind: 'T', idx: 0, dir: -1, need: r.F[1] - fwd, mw: r.F[1] - fwd, sens: b => 1, loc: 1 });
    return worst;
  }
  function anyViolation(r) { return !!worstViolation(r, true); }
  function constraintLabel(v) {
    if (!v) return '';
    if (v.kind === 'V') return (v.dir > 0 ? 'Over-voltage' : 'Under-voltage') + ' at bus ' + v.idx;
    if (v.kind === 'L') return NET.lines[v.idx].id + (v.dir > 0 ? ' reverse-flow' : '') + ' thermal limit';
    return 'Transformer ' + (v.dir > 0 ? 'reverse-power' : 'loading') + ' limit';
  }

  // ---------- controller state ----------
  function newState(kind) {
    return {
      kind,
      soc: BESS.map(b => b.soc0 * b.cap * b.soh), temp: BESS.map(() => 27), thr: BESS.map(() => 0),
      flexDone: FLEX.map(() => 0),
      hist: [], // per-step records
      tot: { avail: 0, used: 0, curt: 0, imp: 0, exp: 0, co2: 0, viol: 0, peakT: 0, deg: 0, absorbedFlex: 0, absorbedBess: 0, cost: 0 },
      cachedPlan: null, lastReason: {}
    };
  }

  function bessLimits(st, i, day) {
    const b = BESS[i], capU = b.cap * b.soh;
    const derate = clamp(1 - (st.temp[i] - 38) / 12, 0.4, 1); // thermal derate above 38 C
    const p = b.pmax * derate;
    return {
      derate, capU, pmax: p,
      chg: Math.max(0, Math.min(p, (SOC_MAX * capU - st.soc[i]) / (DT * b.eta))),
      dis: Math.max(0, Math.min(p, (st.soc[i] - SOC_MIN * capU) * b.eta / DT))
    };
  }

  function flexRemaining(st, j) { return Math.max(0, FLEX[j].energy - st.flexDone[j]); }
  function inWindow(j, k) { const h = hourOf(k); return h >= FLEX[j].win[0] && h < FLEX[j].win[1]; }
  function stepsLeftInWindow(j, k) { let n = 0; for (let x = k; x < STEPS; x++) if (inWindow(j, x)) n++; return n; }
  // Minimum power now so the daily energy can still be met by the end of the window.
  function flexFloor(st, j, k) {
    const f = FLEX[j]; if (!inWindow(j, k)) return 0;
    const rem = flexRemaining(st, j); if (rem <= 0) return Math.min(f.pmin, 0);
    const left = stepsLeftInWindow(j, k);
    return clamp(rem / DT - (left - 1) * f.pmax, Math.min(f.pmin, rem / DT), f.pmax);
  }
  function flexCeil(st, j, k) { return inWindow(j, k) ? Math.min(FLEX[j].pmax, flexRemaining(st, j) / DT) : 0; }

  function netOf(day, k, solarAvail, windAvail, disp) {
    const n = [0, 0, 0, 0, 0];
    for (let b = 1; b <= 4; b++) n[b] = day.load[b][k];
    FLEX.forEach((f, j) => { n[f.bus] += disp.flex[j]; });
    BESS.forEach((b, i) => { n[b.bus] += disp.bess[i]; });
    n[GENS[0].bus] -= solarAvail - disp.curt[0];
    n[GENS[1].bus] -= windAvail - disp.curt[1];
    return n;
  }

  // ---------- forecast curtailment risk (MW that must be absorbed at the feeder end) ----------
  function riskProfile(day, fc, from) {
    const risk = new Float64Array(STEPS), exportMW = new Float64Array(STEPS), imp = new Float64Array(STEPS);
    for (let k = from; k < STEPS; k++) {
      const n = [0, 0, 0, 0, 0];
      for (let b = 1; b <= 4; b++) n[b] = fc.load[b][k];
      FLEX.forEach(f => { n[f.bus] += f.pmin; });
      n[3] -= fc.solar[k]; n[4] -= fc.wind[k];
      const r = evalNet(n, k, day);
      let m = 0;
      for (let i = 1; i <= 4; i++) {
        m = Math.max(m, (r.V[i] - (NET.vmax - MARGIN_V)) / vSens(i, 4));
        m = Math.max(m, -r.F[i] - r.amp[i] * MARGIN_I);
      }
      m = Math.max(m, -r.F[1] - NET.trafo.reverse * MARGIN_I);
      risk[k] = Math.max(0, m);
      exportMW[k] = Math.max(0, -r.F[1]);
      imp[k] = Math.max(0, r.F[1]);
    }
    return { risk, exportMW, imp };
  }

  // Water-fill an energy budget into the best-scoring steps.
  function waterFill(steps, score, pmax, energy) {
    const out = new Map(); const order = steps.slice().sort((a, b) => score[b] - score[a]);
    let e = energy;
    for (const k of order) { if (e <= 1e-9) break; const p = Math.min(pmax, e / DT); out.set(k, p); e -= p * DT; }
    return out;
  }

  // GridFlex look-ahead plan from step k: flexible load schedules + battery intents.
  function planAhead(st, day, fc, k) {
    const rp = riskProfile(day, fc, k);
    const horizon = Math.min(STEPS, k + 144); // 12 h
    // flex: put remaining energy where curtailment risk / export is highest, else where carbon is lowest
    const flexPlan = FLEX.map((f, j) => {
      const steps = []; for (let x = k; x < STEPS; x++) if (inWindow(j, x)) steps.push(x);
      const score = new Float64Array(STEPS);
      for (const x of steps) score[x] = 10 * rp.risk[x] + rp.exportMW[x] - 8 * day.ci[x] - 0.02 * (x - k) / 12;
      const extra = Math.max(0, flexRemaining(st, j) - f.pmin * steps.length * DT);
      const fill = waterFill(steps, score, f.pmax - f.pmin, extra);
      const p = new Float64Array(STEPS);
      for (const x of steps) p[x] = Math.min(f.pmax, f.pmin + (fill.get(x) || 0));
      return p;
    });
    // energy at risk of curtailment ahead, after the flex plan soaks up what it can at buses 3/4
    let riskE = 0, nextRisk = -1;
    for (let x = k; x < horizon; x++) {
      let flexEnd = 0; FLEX.forEach((f, j) => { if (f.bus >= 3) flexEnd += flexPlan[j][x] - f.pmin; });
      const res = Math.max(0, rp.risk[x] - flexEnd);
      if (res > 0.05 && nextRisk < 0) nextRisk = x;
      riskE += res * DT;
    }
    // discharge plan: highest-value import steps before the next surplus (or within 12 h)
    const discEnd = nextRisk >= 0 ? nextRisk : horizon;
    const value = new Float64Array(STEPS); const vsteps = [];
    for (let x = k; x < discEnd; x++) { value[x] = day.price[x] + 80 * day.ci[x]; if (rp.imp[x] > 0.5) vsteps.push(x); }
    const bessPlan = BESS.map((b, i) => {
      const lim = bessLimits(st, i, day);
      const avail = Math.max(0, st.soc[i] - SOC_MIN * lim.capU) * b.eta;
      const ok = vsteps.filter(x => value[x] > b.degCost * 2.2);
      return waterFill(ok, value, lim.pmax * 0.85, avail);
    });
    return { k, rp, flexPlan, bessPlan, riskE, nextRisk, horizon };
  }

  // ---------- real-time constraint solver ----------
  function resolve(day, k, sA, wA, disp, st, opts) {
    const reasons = [];
    const res = { solved: true, binding: null };
    for (let it = 0; it < 4000; it++) {
      const r = evalNet(netOf(day, k, sA, wA, disp), k, day);
      const v = worstViolation(r, false);
      if (!v) break;
      if (!res.binding) res.binding = v;
      let best = null;
      const offer = (type, idx, bus, room, cost) => {
        if (room <= 1e-6) return;
        const s = v.sens(bus); if (s <= 1e-9) return;
        if (opts.localOnly && bus !== v.loc && !(type === 'curt')) return;
        const c = cost / s;
        if (!best || c < best.c) best = { type, idx, bus, room, s, c };
      };
      if (v.dir > 0) {
        if (opts.useFlex) FLEX.forEach((f, j) => offer('flex', j, f.bus, flexCeil(st, j, k) - disp.flex[j], 0.2));
        if (opts.useBess) BESS.forEach((b, i) => { const l = bessLimits(st, i, day); offer('bess', i, b.bus, l.chg - disp.bess[i], 1 + b.degCost / 40); });
        offer('curt', 0, GENS[0].bus, sA - disp.curt[0], 12);
        offer('curt', 1, GENS[1].bus, wA - disp.curt[1], 12);
      } else {
        if (opts.useFlex) FLEX.forEach((f, j) => offer('flex', j, f.bus, disp.flex[j] - flexFloor(st, j, k), 0.4));
        if (opts.useBess) BESS.forEach((b, i) => { const l = bessLimits(st, i, day); offer('bess', i, b.bus, disp.bess[i] + l.dis, 1 + b.degCost / 40); });
        // restore curtailed generation before anything else when the problem is import-side
        offer('uncurt', 0, GENS[0].bus, disp.curt[0], 0.01);
        offer('uncurt', 1, GENS[1].bus, disp.curt[1], 0.01);
      }
      if (!best) { res.solved = false; break; }
      const delta = Math.min(best.room, v.need / best.s + 1e-4, 0.5);
      const sgn = v.dir > 0 ? 1 : -1;
      if (best.type === 'flex') disp.flex[best.idx] += sgn * delta;
      else if (best.type === 'bess') disp.bess[best.idx] += sgn * delta;
      else if (best.type === 'curt') disp.curt[best.idx] += delta;
      else if (best.type === 'uncurt') disp.curt[best.idx] -= delta;
      reasons.push({ type: best.type, idx: best.idx, v });
    }
    res.reasons = reasons;
    return res;
  }

  // ---------- one controller step ----------
  function step(st, day, fc, k, ctx) {
    const sA = day.solar[k], wA = day.wind[k], h = hourOf(k);
    const disp = { flex: FLEX.map(() => 0), bess: BESS.map(() => 0), curt: [0, 0] };
    const why = { flex: FLEX.map(() => ''), bess: BESS.map(() => ''), curt: ['', ''] };
    let plan = null, offline = false;

    if (st.kind === 'baseline') {
      FLEX.forEach((f, j) => { disp.flex[j] = clamp(f.base(h), 0, flexCeil(st, j, k)); why.flex[j] = disp.flex[j] > 0 ? 'fixed schedule' : 'off (schedule)'; });
      BESS.forEach((b, i) => {
        const l = bessLimits(st, i, day);
        if (h < 5) { disp.bess[i] = Math.min(0.5 * b.pmax, l.chg); why.bess[i] = disp.bess[i] > 0 ? 'off-peak tariff charge' : 'idle'; }
        else if (h >= 18 && h < 22) { disp.bess[i] = -Math.min(0.5 * b.pmax, l.dis); why.bess[i] = disp.bess[i] < 0 ? 'tariff discharge' : 'idle'; }
        else why.bess[i] = 'idle';
      });
      const r = resolve(day, k, sA, wA, disp, st, { useFlex: false, useBess: false });
      for (const x of r.reasons) if (x.type === 'curt') why.curt[x.idx] = 'volt-watt / export trip: ' + constraintLabel(x.v);
      st._binding = r.binding;
    } else {
      offline = !!ctx.offline;
      if (!offline) { plan = planAhead(st, day, fc, k); st.cachedPlan = plan; }
      else plan = st.cachedPlan;
      // 1) flexible loads follow the plan (cached plan when offline)
      FLEX.forEach((f, j) => {
        const want = plan ? plan.flexPlan[j][k] : f.base(h);
        disp.flex[j] = clamp(want, flexFloor(st, j, k), flexCeil(st, j, k));
        why.flex[j] = !inWindow(j, k) ? 'outside window' : disp.flex[j] > f.pmin + 0.05
          ? (plan && plan.rp.risk[k] > 0.05 ? 'soaking forecast surplus' : 'scheduled: low-carbon slot')
          : (offline ? 'holding cached schedule' : 'deferred to surplus window');
        if (offline && disp.flex[j] > f.pmin + 0.05) why.flex[j] = 'cached schedule (offline)';
      });
      // 2) batteries: discharge into high-value import steps, store exports beyond the headroom we need
      if (!offline && plan) {
        const rnow = evalNet(netOf(day, k, sA, wA, disp), k, day);
        const exporting = Math.max(0, -rnow.F[1]);
        let headroom = 0; BESS.forEach((b, i) => { headroom += Math.max(0, SOC_MAX * bessLimits(st, i, day).capU - st.soc[i]); });
        let spare = Math.max(0, headroom - plan.riskE * 1.1);
        let expLeft = exporting;
        BESS.forEach((b, i) => {
          const l = bessLimits(st, i, day);
          const d = plan.bessPlan[i].get(k);
          if (d && rnow.F[1] > 0.3) {
            disp.bess[i] = -Math.min(d, l.dis, rnow.F[1]);
            why.bess[i] = plan.nextRisk >= 0 ? 'discharging to make room for forecast surplus' : 'shaving fossil-heavy peak';
          } else if (expLeft > 0.05 && spare > 0.05 && plan.rp.risk[k] < 0.05) {
            const p = Math.min(l.chg, expLeft, spare / DT);
            disp.bess[i] = p; expLeft -= p; spare -= p * DT;
            why.bess[i] = 'storing surplus for evening';
          } else why.bess[i] = plan.riskE > 0.1 ? 'holding headroom for forecast surplus' : 'idle';
        });
      } else BESS.forEach((b, i) => { why.bess[i] = 'local voltage control (offline)'; });
      const r = resolve(day, k, sA, wA, disp, st, { useFlex: true, useBess: true, localOnly: offline });
      for (const x of r.reasons) {
        const lab = constraintLabel(x.v);
        if (x.type === 'flex') why.flex[x.idx] = 'absorbing surplus: ' + lab;
        if (x.type === 'bess') why.bess[x.idx] = (x.v.dir > 0 ? 'charging to relieve ' : 'discharging to relieve ') + lab;
        if (x.type === 'curt') why.curt[x.idx] = 'last resort: ' + lab;
      }
      st._binding = r.binding;
    }

    // ---- settle physics ----
    // hard safety net: any violation that survives is cleared with curtailment (export side)
    let r = evalNet(netOf(day, k, sA, wA, disp), k, day);
    for (let guard = 0; guard < 200 && anyViolation(r); guard++) {
      const v = worstViolation(r, true); if (v.dir < 0) break;
      const s0 = v.sens(GENS[0].bus), s1 = v.sens(GENS[1].bus);
      const g = (s1 >= s0 && disp.curt[1] < wA - 1e-6) || disp.curt[0] >= sA - 1e-6 ? 1 : 0;
      const room = (g ? wA : sA) - disp.curt[g]; if (room <= 1e-6) break;
      disp.curt[g] += Math.min(room, v.need / Math.max(1e-9, v.sens(GENS[g].bus)) + 1e-4);
      if (!why.curt[g]) why.curt[g] = 'safety curtailment';
      r = evalNet(netOf(day, k, sA, wA, disp), k, day);
    }
    const viol = anyViolation(r);
    // battery energy + temperature
    BESS.forEach((b, i) => {
      const p = disp.bess[i];
      st.soc[i] += p > 0 ? p * DT * b.eta : p * DT / b.eta;
      st.thr[i] += Math.abs(p) * DT;
      st.temp[i] += DT * (14 * Math.abs(p) / b.pmax - 0.9 * (st.temp[i] - day.ambient));
      st.tot.deg += Math.abs(p) * DT * b.degCost;
    });
    FLEX.forEach((f, j) => { st.flexDone[j] += disp.flex[j] * DT; });
    const avail = sA + wA, curt = disp.curt[0] + disp.curt[1];
    const imp = Math.max(0, r.F[1]), exp = Math.max(0, -r.F[1]);
    const T = st.tot;
    T.avail += avail * DT; T.curt += curt * DT; T.used += (avail - curt) * DT;
    T.imp += imp * DT; T.exp += exp * DT; T.co2 += imp * day.ci[k] * DT; T.cost += imp * day.price[k] * DT;
    if (viol) T.viol += 5;
    T.peakT = Math.max(T.peakT, Math.abs(r.F[1]) / NET.trafo.rating);
    const rec = {
      k, sA, wA, curt: disp.curt.slice(), flex: disp.flex.slice(), bess: disp.bess.slice(),
      soc: st.soc.map((s, i) => s / (BESS[i].cap * BESS[i].soh)), temp: st.temp.slice(),
      V: r.V.slice(), F: r.F.slice(), amp: r.amp.slice(), imp, exp, viol, offline,
      load: [0, 1, 2, 3, 4].map(b => b ? day.load[b][k] : 0), why,
      binding: st._binding ? constraintLabel(st._binding) : '', bindingObj: st._binding,
      riskE: plan ? plan.riskE : 0, nextRisk: plan ? plan.nextRisk : -1
    };
    st.hist[k] = rec;
    return rec;
  }

  // ---------- simulation wrapper ----------
  function Sim(scenario, seed) {
    this.scenario = scenario; this.seed = seed;
    this.day = makeDay(scenario, seed);
    this.base = newState('baseline'); this.flex = newState('gridflex');
    this.k = 0; this.fc = makeForecast(this.day, 0, seed * 7 + 1);
    this.offlineUntil = -1; this.offlineManual = false;
    this.events = [];
  }
  Sim.prototype.isOffline = function () { return this.offlineManual || this.k < this.offlineUntil; };
  Sim.prototype.tick = function () {
    if (this.k >= STEPS) return null;
    if (this.k % 12 === 0) this.fc = makeForecast(this.day, this.k, this.seed * 7 + this.k + 1);
    const ctx = { offline: this.isOffline() };
    const b = step(this.base, this.day, this.fc, this.k, ctx);
    const g = step(this.flex, this.day, this.fc, this.k, ctx);
    this.k++;
    return { b, g };
  };
  Sim.prototype.runTo = function (k) { while (this.k < k) this.tick(); };
  Sim.prototype.cloudEvent = function () {
    const d = this.day; if (!d.solarClear) d.solarClear = Float64Array.from(d.solar);
    for (let x = this.k; x < Math.min(STEPS, this.k + 9); x++) d.solar[x] *= 0.3;
    this.events.push({ k: this.k, type: 'cloud' });
  };
  Sim.prototype.derateEvent = function () {
    for (let x = this.k; x < Math.min(STEPS, this.k + 24); x++) this.day.ampFactor[3][x] = 0.62;
    this.fc = makeForecast(this.day, this.k, this.seed * 13 + this.k);
    this.events.push({ k: this.k, type: 'derate' });
  };
  Sim.prototype.commsLoss = function (on) {
    if (on === undefined) on = !this.isOffline();
    if (on) { this.offlineUntil = this.k + 18; this.events.push({ k: this.k, type: 'offline' }); }
    else { this.offlineUntil = -1; this.offlineManual = false; this.events.push({ k: this.k, type: 'online' }); }
  };

  const API = { STEPS, DT, NET, GENS, FLEX, BESS, SOC_MIN, SOC_MAX, SCENARIOS, Sim, makeDay, evalNet, hourOf, constraintLabel };
  if (typeof module !== 'undefined' && module.exports) module.exports = API; else root.GridFlex = API;
})(typeof window !== 'undefined' ? window : this);
