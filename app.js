"use strict";

(() => {
  function PhysicsCore() {
    const DT = 1 / 240;
    const MAX_TIME = 300;
    const COLORS = ["#a5edbd", "#ffd293", "#a8caff", "#e0b8f0"];
    const TITLES = {
      body: "물체",
      platform: "수평 발판",
      ramp: "경사면",
      fixed: "고정 도르래",
      moving: "이동 도르래",
      anchor: "고정점",
      rope: "줄"
    };

    const DYNAMIC = n => n.kind === "body" || n.kind === "moving";
    const SURFACE = n => n.kind === "platform" || n.kind === "ramp";
    const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
    const hypot = (x, y) => Math.hypot(x, y);
    const fmt = (x, n = 2) =>
      Number.isFinite(x)
        ? (Math.abs(x) < 0.5 * 10 ** -n ? 0 : x).toFixed(n)
        : "—";
    const copy = x => JSON.parse(JSON.stringify(x));
    const esc = x => String(x).replace(/[&<>"']/g, c =>
      ({
        "&": "&amp;",
        "<": "&lt;",
        ">": "&gt;",
        '"': "&quot;",
        "'": "&#39;"
      }[c])
    );

    let serial = 0;
    const uid = () => "n" + Date.now().toString(36) + (++serial).toString(36);

    function node(kind, x, y, extra = {}) {
      return {
        id: uid(),
        kind,
        name: TITLES[kind],
        x,
        y,
        m: kind === "moving" ? 1 : 2,
        radius: kind === "moving" ? 0.25 : 0.35,
        mu: 0.2,
        fx: 0,
        fy: 0,
        vx: 0,
        vy: 0,
        length: 6,
        angle: kind === "ramp" ? 25 : 0,
        ...extra
      };
    }

    function rope(nodes, path) {
      const r = {
        id: uid(),
        kind: "rope",
        name: "줄",
        path,
        length: 0
      };
      const map = new Map(nodes.map(n => [n.id, n]));
      for (let i = 1; i < path.length; i++) {
        const a = map.get(path[i - 1]);
        const b = map.get(path[i]);
        r.length += hypot(b.x - a.x, b.y - a.y);
      }
      return r;
    }

    function example(kind = "train") {
      const s = { version: 1, g: 9.8, nodes: [], ropes: [] };
      const add = (...args) => {
        const n = node(...args);
        s.nodes.push(n);
        return n;
      };
      const link = list =>
        s.ropes.push(rope(s.nodes, list.map(n => n.id)));

      if (kind === "train") {
        add("platform", 8, 1, { length: 14, mu: 0 });
        const a = add("body", 4, 1.35, { name: "A", m: 2, mu: 0 });
        const b = add("body", 7, 1.35, {
          name: "B", m: 3, mu: 0, fx: 12
        });
        link([a, b]);
      } else if (kind === "incline") {
        const angle = 25 * Math.PI / 180;
        add("ramp", 8, 3, {
          length: 10, angle: 25, mu: 0.2
        });
        add(
          "body",
          8 - Math.sin(angle) * 0.35,
          3 + Math.cos(angle) * 0.35,
          { name: "A", m: 2, mu: 0.2 }
        );
      } else if (kind === "fall") {
        add("body", 8, 8, { name: "A", m: 2 });
        add("platform", 8, 0.5, { length: 14, mu: 0 });
      } else if (kind === "projectile") {
        add("body", 3, 2, {
          name: "A", vx: 5, vy: 7, mu: 0
        });
        add("platform", 8, 0.5, { length: 15, mu: 0 });
      } else if (kind === "atwood") {
        const a = add("body", 5, 4, { name: "A", m: 2 });
        const p = add("fixed", 5, 8, { name: "P₁" });
        const q = add("fixed", 11, 8, { name: "P₂" });
        const b = add("body", 11, 4, { name: "B", m: 3 });
        link([a, p, q, b]);
      } else if (kind === "moving") {
        const anchor = add("anchor", 5, 9, { name: "고정점" });
        const moving = add("moving", 5, 4, { name: "P", m: 4 });
        const p = add("fixed", 5, 8, { name: "P₁" });
        const q = add("fixed", 11, 8, { name: "P₂" });
        const b = add("body", 11, 3, { name: "A", m: 1 });
        link([anchor, moving, p, q, b]);
      }
      return s;
    }

    function addTerm(terms, id, x, y) {
      const old = terms.get(id) || { x: 0, y: 0 };
      terms.set(id, { x: old.x + x, y: old.y + y });
    }

    function geometry(r, map) {
      let length = 0;
      const terms = new Map();

      for (let i = 1; i < r.path.length; i++) {
        const a = map.get(r.path[i - 1]);
        const b = map.get(r.path[i]);
        const dx = b.x - a.x;
        const dy = b.y - a.y;
        const d = hypot(dx, dy);

        if (d < 1e-7) continue;
        length += d;

        const ux = dx / d;
        const uy = dy / d;

        // J = -∇L. 양의 장력은 줄 경로를 짧게 만드는 방향이다.
        if (DYNAMIC(a)) addTerm(terms, a.id, ux, uy);
        if (DYNAMIC(b)) addTerm(terms, b.id, -ux, -uy);
      }

      return { length, terms };
    }

    function matrixEntry(a, b, map) {
      let sum = 0;
      a.terms.forEach((u, id) => {
        const v = b.terms.get(id);
        const p = map.get(id);
        if (v && DYNAMIC(p)) {
          sum += (u.x * v.x + u.y * v.y) / p.m;
        }
      });
      return sum;
    }

    function rank(matrix, tolerance = 1e-8) {
      if (!matrix.length) return 0;
      const a = matrix.map(r => r.slice());
      let row = 0;

      for (let col = 0; col < a[0].length && row < a.length; col++) {
        let pivot = row;
        for (let r = row + 1; r < a.length; r++) {
          if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
        }

        if (Math.abs(a[pivot][col]) < tolerance) continue;
        [a[row], a[pivot]] = [a[pivot], a[row]];

        const div = a[row][col];
        for (let c = col; c < a[0].length; c++) a[row][c] /= div;

        for (let r = 0; r < a.length; r++) {
          if (r === row) continue;
          const factor = a[r][col];
          for (let c = col; c < a[0].length; c++) {
            a[r][c] -= factor * a[row][c];
          }
        }
        row++;
      }
      return row;
    }

    function validate(s) {
      const errors = [];

      if (!s || s.version !== 1 ||
          !Array.isArray(s.nodes) || !Array.isArray(s.ropes)) {
        return ["지원하는 실험 파일 형식이 아닙니다."];
      }

      if (!Number.isFinite(s.g) || s.g < 0 || s.g > 30) {
        errors.push("중력가속도는 0~30 m/s²여야 합니다.");
      }

      if (s.nodes.length > 40 || s.ropes.length > 12) {
        errors.push("항목 40개와 줄 12개까지 지원합니다.");
      }

      const ids = new Set();
      const allowed = [
        "body", "moving", "platform", "ramp", "fixed", "anchor"
      ];

      for (const n of s.nodes) {
        if (!n || typeof n.id !== "string" || ids.has(n.id)) {
          errors.push("항목 ID가 없거나 중복됩니다.");
          continue;
        }

        ids.add(n.id);

        if (!allowed.includes(n.kind)) {
          errors.push("지원하지 않는 항목 종류입니다.");
        }

        if (typeof n.name !== "string" || n.name.length > 60) {
          errors.push("항목 이름은 60자 이내여야 합니다.");
        }

        if (![n.x, n.y].every(v =>
          Number.isFinite(v) && Math.abs(v) <= 1000)) {
          errors.push("위치는 −1000~1000 m 범위여야 합니다.");
        }

        if (DYNAMIC(n)) {
          if (!Number.isFinite(n.m) || n.m < 0.05 || n.m > 100) {
            errors.push("질량은 0.05~100 kg이어야 합니다.");
          }

          if (!Number.isFinite(n.radius) ||
              n.radius < 0.1 || n.radius > 1) {
            errors.push("접촉 반지름은 0.1~1 m여야 합니다.");
          }

          if (![n.fx, n.fy].every(v =>
            Number.isFinite(v) && Math.abs(v) <= 200)) {
            errors.push("외력은 각 방향 −200~200 N이어야 합니다.");
          }

          if (![n.vx, n.vy].every(v =>
            Number.isFinite(v) && Math.abs(v) <= 30)) {
            errors.push("초기 속도는 각 방향 −30~30 m/s여야 합니다.");
          }
        }

        if (DYNAMIC(n) || SURFACE(n)) {
          if (!Number.isFinite(n.mu) || n.mu < 0 || n.mu > 2) {
            errors.push("마찰계수는 0~2여야 합니다.");
          }
        }

        if (SURFACE(n)) {
          if (!Number.isFinite(n.length) ||
              n.length < 0.5 || n.length > 40) {
            errors.push("발판 길이는 0.5~40 m여야 합니다.");
          }

          if (!Number.isFinite(n.angle) ||
              Math.abs(n.angle) > 80 ||
              (n.kind === "platform" && n.angle !== 0)) {
            errors.push(
              "발판 각도를 확인하세요. 수평 발판은 0°, 경사면은 ±80°입니다."
            );
          }
        }
      }

      if (errors.length) return [...new Set(errors)];

      const map = new Map(s.nodes.map(n => [n.id, n]));
      const dynamics = s.nodes.filter(DYNAMIC);

      if (!dynamics.length) {
        errors.push("물체 또는 이동 도르래를 한 개 이상 추가하세요.");
      }
      if (dynamics.length > 12) {
        errors.push("움직이는 항목은 12개까지 지원합니다.");
      }

      const pulleyUse = new Map();
      const tautRows = [];
      const signatures = new Set();

      for (const r of s.ropes) {
        if (!r || typeof r.id !== "string" || ids.has(r.id)) {
          errors.push("줄 ID가 없거나 중복됩니다.");
          continue;
        }

        ids.add(r.id);

        if (typeof r.name !== "string" || r.name.length > 60) {
          errors.push("줄 이름을 확인하세요.");
        }

        if (!Array.isArray(r.path) ||
            r.path.length < 2 || r.path.length > 12 ||
            r.path.some(id => !map.has(id))) {
          errors.push(
            "줄의 연결점이 없거나 경로 길이가 지원 범위를 벗어났습니다."
          );
          continue;
        }

        if (new Set(r.path).size !== r.path.length) {
          errors.push(
            "한 줄이 같은 연결점을 반복해서 지나는 구조는 지원하지 않습니다."
          );
        }

        for (let i = 0; i < r.path.length; i++) {
          const n = map.get(r.path[i]);

          if (SURFACE(n)) {
            errors.push(
              "줄은 발판에 직접 연결할 수 없습니다. 고정점을 사용하세요."
            );
          }

          if (i > 0 && i < r.path.length - 1) {
            if (!["fixed", "moving"].includes(n.kind)) {
              errors.push(
                "줄 경로 중간에는 고정·이동 도르래만 올 수 있습니다."
              );
            }
            pulleyUse.set(n.id, (pulleyUse.get(n.id) || 0) + 1);
          }

          if (i) {
            const a = map.get(r.path[i - 1]);
            if (hypot(n.x - a.x, n.y - a.y) < 0.05) {
              errors.push(
                "인접한 줄 연결점은 0.05 m 이상 떨어져야 합니다."
              );
            }
          }
        }

        const key = [
          r.path.join("|"),
          r.path.slice().reverse().join("|")
        ].sort()[0];

        if (signatures.has(key)) {
          errors.push("동일한 경로의 중복 줄은 지원하지 않습니다.");
        }
        signatures.add(key);

        if (!Number.isFinite(r.length) ||
            r.length < 0.1 || r.length > 200) {
          errors.push("줄 길이는 0.1~200 m여야 합니다.");
        }

        const geo = geometry(r, map);

        if (geo.length > r.length + 0.015) {
          errors.push(
            `${r.name}: 줄이 현재 경로보다 짧습니다. 자동 길이를 누르세요.`
          );
        }

        if (geo.terms.size === 0) {
          errors.push(`${r.name}: 움직이는 물체가 연결되지 않았습니다.`);
        }

        if (Math.abs(geo.length - r.length) <= 0.02) {
          tautRows.push({ terms: geo.terms });
        }
      }

      if ([...pulleyUse.values()].some(v => v > 1)) {
        errors.push(
          "도르래 하나를 여러 줄이 통과하는 다중 홈 구조는 지원하지 않습니다."
        );
      }

      if (tautRows.length) {
        const a = tautRows.map(r =>
          tautRows.map(q => matrixEntry(r, q, map))
        );
        if (rank(a) < tautRows.length) {
          errors.push(
            "줄 구속이 중복되거나 특이합니다. 장력을 유일하게 구할 수 없어 실행하지 않습니다."
          );
        }
      }

      for (let i = 0; i < dynamics.length; i++) {
        const b = dynamics[i];

        for (let j = i + 1; j < dynamics.length; j++) {
          const c = dynamics[j];
          if (hypot(b.x - c.x, b.y - c.y) <
              b.radius + c.radius - 0.025) {
            errors.push(
              `${b.name}와 ${c.name}가 겹칩니다. 떨어뜨려 배치하세요.`
            );
          }
        }

        for (const p of s.nodes.filter(SURFACE)) {
          const a = p.angle * Math.PI / 180;
          const tx = Math.cos(a), ty = Math.sin(a);
          const nx = -ty, ny = tx;
          const dx = b.x - p.x, dy = b.y - p.y;
          const along = dx * tx + dy * ty;
          const distance = dx * nx + dy * ny;

          if (Math.abs(along) <= p.length / 2 &&
              distance >= 0 && distance < b.radius - 0.025) {
            errors.push(
              `${b.name}가 발판 윗면을 파고듭니다. 위로 옮기세요.`
            );
          }
        }
      }

      return [...new Set(errors)];
    }

    function record(e) {
      const bodies = {};
      e.dynamic.forEach(n => {
        bodies[n.id] = {
          x: n.x,
          y: n.y,
          vx: n.vx,
          vy: n.vy,
          ax: n.ax,
          ay: n.ay,
          speed: hypot(n.vx, n.vy),
          energy: 0.5 * n.m * (n.vx ** 2 + n.vy ** 2) +
            n.m * e.model.g * n.y
        };
      });
      e.history.push({ t: e.time, bodies });
    }

    function dotVelocity(row, map) {
      let v = 0;
      row.terms.forEach((g, id) => {
        const n = map.get(id);
        v += g.x * n.vx + g.y * n.vy;
      });
      return v;
    }

    function applyImpulse(row, amount, map) {
      row.terms.forEach((g, id) => {
        const n = map.get(id);
        n.vx += amount * g.x / n.m;
        n.vy += amount * g.y / n.m;
      });
    }

    function makeRows(e, h) {
      const rows = [];

      function contact(a, b, nx, ny, gap, mu, label, surfaceId = null) {
        const normal = {
          kind: "normal",
          label,
          gap,
          impulse: 0,
          pair: !!b,
          surfaceId,
          bodyId: a.id,
          terms: new Map([[a.id, { x: nx, y: ny }]])
        };

        const tangent = {
          kind: "friction",
          label: "f " + label,
          gap: 0,
          impulse: 0,
          parent: normal,
          mu,
          terms: new Map([[a.id, { x: -ny, y: nx }]])
        };

        if (b) {
          normal.terms.set(b.id, { x: -nx, y: -ny });
          tangent.terms.set(b.id, { x: ny, y: -nx });
        }

        rows.push(normal, tangent);
      }

      for (const b of e.dynamic) {
        for (const p of e.nodes.filter(SURFACE)) {
          const a = p.angle * Math.PI / 180;
          const tx = Math.cos(a), ty = Math.sin(a);
          const nx = -ty, ny = tx;
          const dx = b.x - p.x, dy = b.y - p.y;
          const along = dx * tx + dy * ty;
          const distance = dx * nx + dy * ny;
          const gap = distance - b.radius;
          const reach = 0.06 + hypot(b.vx, b.vy) * h;

          if (Math.abs(along) <= p.length / 2 &&
              distance > -0.02 && gap < reach) {
            contact(
              b, null, nx, ny, gap,
              Math.min(b.mu, p.mu),
              `${p.name}→${b.name}`,
              p.id
            );
          }
        }
      }

      for (let i = 0; i < e.dynamic.length; i++) {
        for (let j = i + 1; j < e.dynamic.length; j++) {
          const a = e.dynamic[i], b = e.dynamic[j];
          const dx = a.x - b.x, dy = a.y - b.y;
          const d = hypot(dx, dy);
          const gap = d - a.radius - b.radius;
          const reach = 0.05 +
            (hypot(a.vx, a.vy) + hypot(b.vx, b.vy)) * h;

          if (d > 1e-6 && gap < reach) {
            contact(
              a, b, dx / d, dy / d, gap,
              Math.min(a.mu, b.mu),
              `${b.name}↔${a.name}`
            );
          }
        }
      }

      for (const r of e.model.ropes) {
        const geo = geometry(r, e.map);
        rows.push({
          kind: "rope",
          label: r.name,
          ropeId: r.id,
          terms: geo.terms,
          gap: r.length - geo.length,
          impulse: 0
        });
      }

      rows.forEach(r => {
        r.diag = matrixEntry(r, r, e.map);
      });

      return rows;
    }

    function gaussian(A, b) {
      const n = b.length;
      const aug = A.map((r, i) => r.concat(b[i]));
      const ops = [];

      for (let k = 0; k < n; k++) {
        let p = k;
        for (let i = k + 1; i < n; i++) {
          if (Math.abs(aug[i][k]) > Math.abs(aug[p][k])) p = i;
        }

        if (Math.abs(aug[p][k]) < 1e-9) {
          return { ok: false, answer: [], reduced: aug, ops };
        }

        if (p !== k) {
          [aug[p], aug[k]] = [aug[k], aug[p]];
          ops.push(`R${k + 1} ↔ R${p + 1}`);
        }

        const pivot = aug[k][k];
        if (Math.abs(pivot - 1) > 1e-9) {
          ops.push(`R${k + 1} ÷ ${fmt(pivot, 5)}`);
        }

        for (let j = k; j <= n; j++) aug[k][j] /= pivot;

        for (let i = 0; i < n; i++) {
          if (i === k) continue;
          const factor = aug[i][k];
          if (Math.abs(factor) < 1e-11) continue;

          for (let j = k; j <= n; j++) {
            aug[i][j] -= factor * aug[k][j];
          }
          ops.push(
            `R${i + 1} ← R${i + 1} − (${fmt(factor, 5)})R${k + 1}`
          );
        }
      }

      return {
        ok: true,
        answer: aug.map(r => r[n]),
        reduced: aug,
        ops
      };
    }

    function ropeCurvature(r, map) {
      let sum = 0;

      for (let i = 1; i < r.path.length; i++) {
        const a = map.get(r.path[i - 1]);
        const b = map.get(r.path[i]);
        const dx = b.x - a.x, dy = b.y - a.y;
        const l = hypot(dx, dy);

        if (l < 1e-8) continue;

        const vx = (DYNAMIC(b) ? b.vx : 0) -
          (DYNAMIC(a) ? a.vx : 0);
        const vy = (DYNAMIC(b) ? b.vy : 0) -
          (DYNAMIC(a) ? a.vy : 0);
        const radial = (vx * dx + vy * dy) / l;

        sum += Math.max(
          0, vx * vx + vy * vy - radial * radial
        ) / l;
      }

      return sum;
    }

    function solveProblem(e) {
      const accelerations = new Map(e.dynamic.map(n => [
        n.id,
        { x: n.fx / n.m, y: n.fy / n.m - e.model.g }
      ]));
      const free = new Map([...accelerations].map(([id, v]) =>
        [id, { ...v }]
      ));

      const rows = makeRows(e, DT).filter(r =>
        r.kind === "friction"
          ? r.parent.gap <= 1e-5
          : r.gap <= 1e-5
      );

      const reasons = [];
      const impact = rows.some(r =>
        r.kind !== "friction" &&
        dotVelocity(r, e.map) < -1e-5
      );

      function dotA(r) {
        let value = 0;
        r.terms.forEach((g, id) => {
          const a = accelerations.get(id);
          value += g.x * a.x + g.y * a.y;
        });
        return value;
      }

      function push(r, force) {
        r.terms.forEach((g, id) => {
          const a = accelerations.get(id);
          const m = e.map.get(id).m;
          a.x += g.x * force / m;
          a.y += g.y * force / m;
        });
      }

      rows.forEach(r => {
        r.force = 0;
        r.target = r.kind === "rope"
          ? ropeCurvature(
              e.model.ropes.find(q => q.id === r.ropeId),
              e.map
            )
          : 0;
      });

      // 장력·수직항력의 비음수 조건과 마찰 한계로 활성 집합을 찾는다.
      for (let pass = 0; pass < 1600; pass++) {
        let largest = 0;

        for (const r of rows) {
          if (r.diag < 1e-12) continue;
          const old = r.force;

          if (r.kind === "friction") {
            const cap = r.mu * r.parent.force;
            const vt = dotVelocity(r, e.map);
            r.force = Math.abs(vt) > 1e-6
              ? -Math.sign(vt) * cap
              : clamp(old - dotA(r) / r.diag, -cap, cap);
          } else {
            r.force = Math.max(
              0, old + (r.target - dotA(r)) / r.diag
            );
          }

          push(r, r.force - old);
          largest = Math.max(largest, Math.abs(r.force - old));
        }

        if (largest < 1e-11) break;
      }

      const active = rows.filter(r =>
        r.kind === "friction"
          ? r.parent.force > 1e-7 && r.mu > 1e-9
          : r.force > 1e-7
      );

      active.forEach(r => {
        if (r.kind !== "friction") return;
        const vt = dotVelocity(r, e.map);
        const at = dotA(r);

        r.slide = Math.abs(vt) > 1e-6
          ? Math.sign(vt)
          : Math.abs(at) > 1e-6 &&
            Math.abs(r.force) >= r.mu * r.parent.force - 1e-6
            ? Math.sign(at)
            : 0;
      });

      const A = active.map(r => {
        if (r.kind === "friction" && r.slide) {
          return active.map(q =>
            q === r ? 1 : q === r.parent ? r.slide * r.mu : 0
          );
        }
        return active.map(q => matrixEntry(r, q, e.map));
      });

      const b = active.map(r => {
        if (r.kind === "friction" && r.slide) return 0;
        let dot = 0;

        r.terms.forEach((g, id) => {
          const a = free.get(id);
          dot += g.x * a.x + g.y * a.y;
        });

        return r.target - dot;
      });

      // 교육용 풀이에는 실제 활성 연립계의 가우스 소거 결과를 사용한다.
      const linear = gaussian(A, b);

      if (linear.ok) {
        const original = rows.map(r => r.force);
        active.forEach((r, i) => {
          r.force = linear.answer[i];
        });

        const valid = rows.every(r =>
          r.kind === "friction"
            ? Math.abs(r.force) <= r.mu * r.parent.force + 1e-5
            : r.force >= -1e-6
        );

        if (!valid) {
          rows.forEach((r, i) => {
            r.force = original[i];
          });
        } else {
          accelerations.forEach((a, id) => {
            Object.assign(a, free.get(id));
          });
          rows.forEach(r => push(r, r.force));
        }
      }

      let residual = 0;
      for (const r of rows.filter(r => r.kind !== "friction")) {
        const value = dotA(r) - r.target;
        residual = Math.max(
          residual,
          r.force > 1e-7 ? Math.abs(value) : Math.max(0, -value)
        );
      }

      if (!linear.ok) {
        reasons.push("독립적인 연립계를 만들 수 없습니다.");
      }
      if (impact) {
        reasons.push(
          "초기 속도가 접촉 또는 팽팽한 줄의 구속과 맞지 않습니다. 먼저 충격이 발생하므로 순간 힘을 일정값으로 정의할 수 없습니다."
        );
      }
      if (residual > 1e-5) {
        reasons.push("힘 연립계가 충분히 수렴하지 않았습니다.");
      }

      // 각 구간의 상대속도와 상대가속도가 줄과 평행할 때만
      // 현재 운동 구간의 방향이 일정하다고 판정한다.
      for (const r of e.model.ropes) {
        const geo = geometry(r, e.map);

        if (r.length - geo.length > 1e-5) {
          reasons.push(
            `${r.name}: 느슨한 줄이므로 팽팽해지는 시점에 가속도가 달라집니다.`
          );
        }

        for (let i = 1; i < r.path.length; i++) {
          const a = e.map.get(r.path[i - 1]);
          const b = e.map.get(r.path[i]);
          const dx = b.x - a.x, dy = b.y - a.y;
          const l = hypot(dx, dy);

          const va = DYNAMIC(a)
            ? { x: a.vx, y: a.vy } : { x: 0, y: 0 };
          const vb = DYNAMIC(b)
            ? { x: b.vx, y: b.vy } : { x: 0, y: 0 };
          const aa = accelerations.get(a.id) || { x: 0, y: 0 };
          const ab = accelerations.get(b.id) || { x: 0, y: 0 };

          if (l > 1e-6 && (
            Math.abs(dx * (vb.y - va.y) - dy * (vb.x - va.x)) / l > 1e-6 ||
            Math.abs(dx * (ab.y - aa.y) - dy * (ab.x - aa.x)) / l > 1e-5
          )) {
            reasons.push(
              `${r.name}: 줄 구간의 방향이 변하므로 장력과 가속도도 변할 수 있습니다.`
            );
          }
        }
      }

      for (const r of rows) {
        if (r.pair) {
          reasons.push(
            "물체 간 접촉·충돌이 포함되어 있어 접촉 방향과 힘이 바뀔 수 있습니다."
          );
        }

        if (r.kind === "normal" && Math.abs(r.gap) > 1e-5) {
          reasons.push(
            "초기 접촉 위치에 침투 오차가 있습니다. 배치를 정확히 맞추세요."
          );
        }

        if (r.kind !== "friction" &&
            dotVelocity(r, e.map) > 1e-5) {
          reasons.push(
            "줄이 느슨해지거나 접촉이 떨어지는 초기 운동입니다."
          );
        }

        if (r.kind !== "friction" &&
            r.force < 1e-7 &&
            dotA(r) - r.target > 1e-5) {
          reasons.push(
            "장력 또는 수직항력이 0이 되어 구속이 해제됩니다."
          );
        }
      }

      const values = new Map();

      for (const n of e.dynamic) {
        const a = accelerations.get(n.id);
        const forces = [
          { kind: "gravity", name: "mg", x: 0, y: -n.m * e.model.g },
          { kind: "external", name: "F", x: n.fx, y: n.fy }
        ];

        rows.forEach(r => {
          const g = r.terms.get(n.id);
          if (!g || Math.abs(r.force) <= 1e-8) return;
          forces.push({
            kind: r.kind,
            name: r.kind === "rope"
              ? "T " + r.label
              : r.kind === "normal" ? "N" : "f",
            x: g.x * r.force,
            y: g.y * r.force,
            ropeId: r.ropeId
          });
        });

        values.set(n.id, {
          ax: a.x,
          ay: a.y,
          fx: n.m * a.x,
          fy: n.m * a.y,
          forces,
          normal: forces.filter(f => f.kind === "normal")
            .reduce((v, f) => v + hypot(f.x, f.y), 0),
          friction: forces.filter(f => f.kind === "friction")
            .reduce((v, f) => v + hypot(f.x, f.y), 0)
        });
      }

      return {
        constant: reasons.length === 0,
        reliable: !impact && residual < 1e-5,
        reasons: [...new Set(reasons)],
        rows,
        values,
        system: { A, b, active, ...linear },
        residual,
        contacts: rows.filter(r =>
          r.kind === "normal" && r.surfaceId && r.force > 1e-7
        )
      };
    }

    function seedSolution(e) {
      const solution = e.solution;

      for (const n of e.dynamic) {
        const v = solution.values.get(n.id);
        if (solution.reliable) {
          n.ax = v.ax;
          n.ay = v.ay;
          n.forces = copy(v.forces);
        }
      }

      e.rows = solution.rows.map(r => ({
        ...r, impulse: r.force * DT
      }));
      e.matrix = e.rows.map(a =>
        e.rows.map(b => matrixEntry(a, b, e.map))
      );
      e.rhs = e.rows.map(() => 0);
    }

    function boundaryReason(e, h) {
      const solution = e.solution;
      const future = new Map(e.nodes.map(n => [n.id, { ...n }]));

      for (const n of e.dynamic) {
        const a = solution.values.get(n.id);
        const p = future.get(n.id);
        p.x = n.x + n.vx * h + 0.5 * a.ax * h * h;
        p.y = n.y + n.vy * h + 0.5 * a.ay * h * h;
        p.vx = n.vx + a.ax * h;
        p.vy = n.vy + a.ay * h;
      }

      for (const n of e.dynamic) {
        const q = future.get(n.id);

        for (const p of e.nodes.filter(SURFACE)) {
          const angle = p.angle * Math.PI / 180;
          const t = { x: Math.cos(angle), y: Math.sin(angle) };
          const normal = { x: -t.y, y: t.x };
          const along = (q.x - p.x) * t.x + (q.y - p.y) * t.y;
          const gap = (q.x - p.x) * normal.x +
            (q.y - p.y) * normal.y - q.radius;

          const supported = solution.contacts.find(r =>
            r.surfaceId === p.id && r.bodyId === n.id
          );

          if (supported) {
            if (Math.abs(along) > p.length / 2 - 1e-7) {
              return `${n.name}가 ${p.name}의 끝에 도달합니다. 이후 수직항력이 바뀝니다.`;
            }

            const mu = Math.min(n.mu, p.mu);
            const v = n.vx * t.x + n.vy * t.y;
            const nv = q.vx * t.x + q.vy * t.y;

            if (mu > 0 && Math.abs(v) > 1e-7 && v * nv <= 0) {
              return `${n.name}가 마찰로 멈춥니다. 이후 정지 마찰 조건을 다시 풀어야 합니다.`;
            }
          } else if (
            Math.abs(along) <= p.length / 2 && gap < 1e-6
          ) {
            const oldGap = (n.x - p.x) * normal.x +
              (n.y - p.y) * normal.y - n.radius;
            const vn = q.vx * normal.x + q.vy * normal.y;

            if (oldGap >= -1e-6 && vn < -1e-6) {
              return `${n.name}가 ${p.name}와 충돌합니다. 이후 운동 구간의 힘이 달라집니다.`;
            }
          }
        }
      }

      for (let i = 0; i < e.dynamic.length; i++) {
        for (let j = i + 1; j < e.dynamic.length; j++) {
          const a = future.get(e.dynamic[i].id);
          const b = future.get(e.dynamic[j].id);

          if (hypot(a.x - b.x, a.y - b.y) <
              a.radius + b.radius - 1e-7) {
            return "물체 사이 충돌이 시작됩니다.";
          }
        }
      }

      const original = new Map(e.model.nodes.map(n => [n.id, n]));

      for (const r of e.model.ropes) {
        for (let i = 1; i < r.path.length; i++) {
          const a = future.get(r.path[i - 1]);
          const b = future.get(r.path[i]);
          const oa = original.get(a.id);
          const ob = original.get(b.id);

          if ((b.x - a.x) * (ob.x - oa.x) +
              (b.y - a.y) * (ob.y - oa.y) <= 0.01) {
            return `${r.name}의 연결점이 만나 줄 경로가 바뀝니다.`;
          }
        }

        if (Math.abs(geometry(r, future).length - r.length) > 2e-5) {
          return `${r.name}의 팽팽한 줄 조건이 달라집니다.`;
        }
      }

      return "";
    }

    function createEngine(s, textbook = true) {
      const e = {
        model: copy(s),
        nodes: copy(s.nodes),
        time: 0,
        rows: [],
        matrix: [],
        rhs: [],
        residual: 0,
        lastH: DT,
        history: [],
        nextSample: 0.05,
        useTextbook: textbook,
        phaseEnded: false,
        event: ""
      };

      e.map = new Map(e.nodes.map(n => [n.id, n]));
      e.dynamic = e.nodes.filter(DYNAMIC);

      e.dynamic.forEach(n => {
        n.ax = n.fx / n.m;
        n.ay = n.fy / n.m - s.g;
        n.forces = [
          { kind: "gravity", name: "mg", x: 0, y: -n.m * s.g },
          { kind: "external", name: "F", x: n.fx, y: n.fy }
        ];
      });

      e.initialU = e.dynamic.reduce(
        (v, n) => v + n.m * s.g * n.y, 0
      );

      e.solution = solveProblem(e);
      seedSolution(e);
      record(e);
      return e;
    }

    function numericalStep(e, h) {
      const old = new Map(e.dynamic.map(n =>
        [n.id, { vx: n.vx, vy: n.vy }]
      ));

      for (const n of e.dynamic) {
        n.vx += n.fx / n.m * h;
        n.vy += (n.fy / n.m - e.model.g) * h;
      }

      const rows = makeRows(e, h);

      // 침투 위치 오차를 반발 속도로 바꾸지 않는다.
      const rhs = rows.map(r =>
        -dotVelocity(r, e.map) - Math.max(0, r.gap) / h
      );

      for (let pass = 0; pass < 240; pass++) {
        let largest = 0;

        for (const r of rows) {
          if (r.diag < 1e-12) continue;

          const oldJ = r.impulse;
          const delta = (
            -dotVelocity(r, e.map) - Math.max(0, r.gap) / h
          ) / r.diag;

          r.impulse = r.kind === "friction"
            ? clamp(
                oldJ + delta,
                -r.mu * r.parent.impulse,
                r.mu * r.parent.impulse
              )
            : Math.max(0, oldJ + delta);

          applyImpulse(r, r.impulse - oldJ, e.map);
          largest = Math.max(largest, Math.abs(r.impulse - oldJ));
        }

        if (largest < 1e-12) break;
      }

      e.rows = rows;
      e.rhs = rhs;
      e.matrix = rows.map(a =>
        rows.map(b => matrixEntry(a, b, e.map))
      );
      e.residual = 0;

      for (const r of rows.filter(r => r.kind !== "friction")) {
        const value = dotVelocity(r, e.map) +
          Math.max(0, r.gap) / h;
        e.residual = Math.max(
          e.residual,
          r.impulse > 1e-8
            ? Math.abs(value)
            : Math.max(0, -value)
        );
      }

      for (const n of e.dynamic) {
        const previous = old.get(n.id);
        n.ax = (n.vx - previous.vx) / h;
        n.ay = (n.vy - previous.vy) / h;
        n.forces = [
          { kind: "gravity", name: "mg", x: 0, y: -n.m * e.model.g },
          { kind: "external", name: "F", x: n.fx, y: n.fy }
        ];
      }

      rows.forEach(r => {
        r.terms.forEach((g, id) => {
          if (Math.abs(r.impulse) <= 1e-10) return;
          e.map.get(id).forces.push({
            kind: r.kind,
            name: r.kind === "rope"
              ? "T " + r.label
              : r.kind === "normal" ? "N" : "f",
            ropeId: r.ropeId,
            x: g.x * r.impulse / h,
            y: g.y * r.impulse / h
          });
        });
      });

      e.dynamic.forEach(n => {
        n.x += n.vx * h;
        n.y += n.vy * h;
      });

      // 길이·침투 오차는 위치에서만 보정한다.
      // 이 보정을 탄성 에너지나 반발 속도로 환산하지 않는다.
      for (let pass = 0; pass < 32; pass++) {
        let error = 0;
        const constraints = makeRows(e, h).filter(r =>
          r.kind !== "friction" && r.gap < -1e-9
        );

        for (const r of constraints) {
          if (r.diag < 1e-12) continue;
          const correction = -r.gap / r.diag;

          r.terms.forEach((g, id) => {
            const n = e.map.get(id);
            n.x += g.x * correction / n.m;
            n.y += g.y * correction / n.m;
          });

          error = Math.max(error, -r.gap);
        }

        if (error < 1e-8) break;
      }
    }

    function integrate(e, h = DT) {
      if (e.event) return;

      if (e.solution.constant && !e.phaseEnded) {
        const reason = boundaryReason(e, h);
        if (reason) {
          e.phaseEnded = true;
          e.phaseReason = reason;
          if (e.useTextbook) {
            e.event = reason;
            return;
          }
        }
      }

      e.lastH = h;

      if (e.useTextbook &&
          e.solution.constant && !e.phaseEnded) {
        e.dynamic.forEach(n => {
          const value = e.solution.values.get(n.id);

          n.x += n.vx * h + 0.5 * value.ax * h * h;
          n.y += n.vy * h + 0.5 * value.ay * h * h;
          n.vx += value.ax * h;
          n.vy += value.ay * h;
          n.ax = value.ax;
          n.ay = value.ay;
          n.forces = copy(value.forces);
        });

        e.residual = 0;
        e.rows = e.solution.rows.map(r => ({
          ...r, impulse: r.force * h
        }));
      } else {
        numericalStep(e, h);
      }

      e.time += h;

      if (e.time + 1e-9 >= e.nextSample) {
        record(e);
        e.nextSample += 0.05;
      }
    }

    return {
      DT, MAX_TIME, COLORS, TITLES, DYNAMIC, SURFACE,
      clamp, hypot, fmt, copy, esc, uid, node, rope, example,
      geometry, matrixEntry, validate, createEngine, integrate,
      gaussian, solveProblem
    };
  }

  const {
    DT, MAX_TIME, COLORS, TITLES, DYNAMIC, SURFACE,
    clamp, hypot, fmt, copy, esc, uid, node, rope, example,
    geometry, matrixEntry, validate, createEngine, integrate
  } = PhysicsCore();

  const $ = id => document.getElementById(id);

  let scene = example();
  let engine;
  let selected = scene.nodes.find(DYNAMIC).id;
  let tool = "select";
  let route = [];
  let running = false;
  let finished = false;
  let lastTime = 0;
  let accumulated = 0;
  let activeTab = "analysis";
  let drag = null;
  let view = { x: 0, y: 0, w: 16, h: 10 };
  let renderAt = 0;
  let learningAt = 0;
  let uiDirty = true;
  let study = { revealed: false, predictions: {} };
  let textbookEnabled = true;
  let equationSolution = null;

  function installStudyUI() {
    const style = document.createElement("style");
    style.textContent = `
      .study-box{padding:18px}
      .study-box h2{margin:0}
      .study-head{display:flex;justify-content:space-between;align-items:center;gap:10px;flex-wrap:wrap}
      .study-tag{display:inline-block;background:#e7f3e5;color:#24633c;border-radius:20px;padding:4px 10px;font-size:11px}
      .study-scroll{overflow:auto}
      .study-table{font:12px/1.6 system-ui,sans-serif;width:100%;margin:10px 0}
      .study-table td,.study-table th{padding:8px;text-align:right}
      .study-table th:first-child,.study-table td:first-child{text-align:left}
      .study-pred{width:85px!important;min-height:40px}
      .study-live{background:#f4f7f0}
      .study-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}
      .study-fbd{width:100%;background:#f5f7f1;border-radius:10px}
      .study-result{margin:12px 0;color:#245d35;font-size:12px}
      .study-box .compact{margin:12px 0}
      .study-hint{font-size:12px;color:#637b60}
      @media(max-width:600px){
        .study-box{padding:14px}
        .study-table{font-size:11px}
      }
    `;
    document.head.append(style);

    const panel = document.createElement("section");
    panel.id = "studyPanel";
    panel.className = "card study-box";
    panel.innerHTML = `
      <div class="study-head">
        <h2>문제 풀이 모드</h2>
        <span id="studyTag" class="study-tag"></span>
      </div>
      <p class="small">
        배치의 초기 조건으로 먼저 풉니다.
        오른쪽 +x, 위쪽 +y · 위치와 속도는 별도 표시합니다.
      </p>
      <label class="check">
        <input id="textbookPlayback" type="checkbox" checked>
        조건이 성립하면 일정 가속도로 재생
      </label>
      <p class="small">
        충돌·발판 끝·마찰로 정지하는 경계에서는 자동 일시정지합니다.
        체크를 끄면 초기 상태부터 일반 수치 시뮬레이션으로 실험할 수 있습니다.
      </p>
      <div id="studyNotice" class="notice"></div>
      <div id="studyPredictions"></div>
      <div class="compact">
        <button id="revealSolution" class="primary">계산 결과와 풀이 확인</button>
        <button id="hideSolution">예측 다시 하기</button>
      </div>
      <div id="studyAnswers" hidden>
        <h3 id="studyResultTitle"></h3>
        <div id="studyFixed"></div>
        <div id="studyGrade" class="study-result"></div>
        <h3>시간에 따라 변하는 측정값</h3>
        <div id="studyLive" class="study-scroll"></div>
        <details>
          <summary>모든 물체의 초기 자유물체도</summary>
          <div id="studyDiagrams" class="study-grid"></div>
        </details>
      </div>
    `;

    const column = document.querySelector(".work-column");
    column.insertBefore(panel, column.firstChild);

    $("speed").innerHTML = `
      <option value=".05">0.05×</option>
      <option value=".1">0.1×</option>
      <option value=".25" selected>0.25×</option>
      <option value=".5">0.5×</option>
      <option value="1">1×</option>
      <option value="2">2×</option>
    `;
    $("step").textContent = "0.01초 진행";

    $("textbookPlayback").onchange = () => {
      textbookEnabled = $("textbookPlayback").checked;
      reset();
      message("재생 방식을 변경했습니다. 초기 상태에서 다시 계산합니다.");
    };

    $("revealSolution").onclick = () => {
      const errors = validate(scene);
      if (errors.length) {
        message(errors.join("\n"), true);
        return;
      }
      study.revealed = true;
      equationSolution = null;
      renderStudy(true);
      render();
    };

    $("hideSolution").onclick = () => {
      study.revealed = false;
      renderStudy(true);
      render();
    };

    $("studyPredictions").addEventListener("input", e => {
      if (e.target.dataset.pred) {
        study.predictions[e.target.dataset.pred] = e.target.value;
      }
    });

    const note = document.createElement("p");
    note.className = "notice";
    note.textContent =
      "개선된 재생: 교과서 조건이 성립하는 현재 운동 구간에서는 연립계의 일정한 가속도를 그대로 사용합니다. " +
      "각도가 변하는 진자 운동은 실제로 장력이 변하므로 고정값으로 처리하지 않습니다. " +
      "일반 모드에는 탄성 스프링을 넣지 않았으며, 줄의 미세한 길이 오차를 위치 투영으로 보정합니다.";
    $("notes").append(note);
  }

  function signed(value) {
    return (Math.abs(value) < 0.0005 ? "" : value > 0 ? "+" : "") +
      fmt(value, 3);
  }

  function maximumT(solution, id) {
    return Math.max(
      0,
      ...solution.rows
        .filter(r => r.kind === "rope" && r.terms.has(id))
        .map(r => Math.max(0, r.force))
    );
  }

  function currentT(id) {
    return Math.max(
      0,
      ...engine.rows
        .filter(r => r.kind === "rope" && r.terms.has(id))
        .map(r => (r.impulse || 0) / (engine.lastH || DT))
    );
  }

  function initialFBD(n, value) {
    let s = `
      <path d="M15 165H405M210 15V315"
        stroke="#dbe5d4" stroke-dasharray="4 6"/>
      <circle cx="210" cy="165" r="20" fill="#b3dfa8"/>
      <text x="210" y="170" text-anchor="middle"
        font-size="12" fill="#234c2b">${esc(n.name)}</text>
    `;

    value.forces.forEach((f, i) => {
      s += arrow(
        210 + (i - 1) * 7,
        165,
        f.x,
        f.y,
        forceColor(f.kind),
        `${f.name} ${fmt(hypot(f.x, f.y))} N`,
        true
      );
    });
    return s;
  }

  function renderStudy(rebuild = false) {
    if (!$("studyPanel")) return;

    const s = engine.solution;
    const errors = validate(scene);

    $("studyTag").textContent = errors.length
      ? "배치 확인 필요"
      : s.constant
        ? "현재 운동 구간: 일정 가속도"
        : "시간 변화 가능";

    $("studyNotice").textContent = errors.length
      ? errors.join("\n")
      : !s.reliable
        ? "초기 충격 또는 수치 수렴 문제 때문에 초기 힘을 하나의 일정값으로 표시하지 않습니다.\n" +
          s.reasons.join("\n")
        : s.constant
          ? engine.phaseEnded
            ? "초기 운동 구간이 끝났습니다. 아래 고정표는 초기 구간의 참고값입니다.\n" +
              (engine.phaseReason || "")
            : "현재 줄 방향과 접촉 상태가 유지되는 구간에서 가속도·장력·알짜힘이 일정합니다. " +
              "충돌, 줄 경로 변화, 접촉 변화, 마찰로 정지하면 다시 풀어야 합니다."
          : "아래 사전 계산은 t=0의 순간값입니다. 운동 중 고정값으로 사용하지 않습니다.\n" +
            s.reasons.join("\n");

    $("studyNotice").classList.toggle(
      "error", errors.length > 0 || !s.reliable
    );
    $("studyAnswers").hidden = !study.revealed;

    $("studyResultTitle").textContent =
      s.constant && !engine.phaseEnded
        ? "구간 내 일정값 · 위치·속도와 분리"
        : "초기 조건의 계산값 · 현재값과 구분";

    if (rebuild) {
      $("studyPredictions").innerHTML = `
        <h3>먼저 예측해 보세요
          <span class="small">· 입력은 선택 사항</span>
        </h3>
        <div class="study-scroll">
          <table class="study-table">
            <thead><tr>
              <th>물체</th>
              <th>aₓ (m/s²)</th>
              <th>aᵧ (m/s²)</th>
              <th>연결된 줄의 최대 T (N)</th>
            </tr></thead>
            <tbody>
              ${engine.dynamic.map(n => `
                <tr><td>${esc(n.name)}</td>
                ${["ax", "ay", "T"].map(key => {
                  const id = n.id + ":" + key;
                  return `
                    <td>
                      <input class="study-pred"
                        type="number" step="any" inputmode="decimal"
                        data-pred="${id}"
                        aria-label="${esc(n.name)} ${key} 예측"
                        value="${esc(study.predictions[id] || "")}">
                    </td>
                  `;
                }).join("")}
                </tr>
              `).join("")}
            </tbody>
          </table>
        </div>
      `;

      if (study.revealed) {
        $("studyFixed").innerHTML = !s.reliable
          ? `<p class="notice">
               초기 충격의 순간 힘은 유한한 고정값으로 정의하지 않습니다.
               운동을 진행하면 현재 시간 단계의 평균 힘을 확인할 수 있습니다.
             </p>`
          : `
            <div class="study-scroll">
              <table class="study-table">
                <thead><tr>
                  <th>물체</th>
                  <th>aₓ (m/s²)</th>
                  <th>aᵧ (m/s²)</th>
                  <th>ΣFₓ (N)</th>
                  <th>ΣFᵧ (N)</th>
                  <th>Σ|N| (N)</th>
                  <th>Σ|f| (N)</th>
                  <th>최대 T (N)</th>
                </tr></thead>
                <tbody>
                ${engine.dynamic.map(n => {
                  const v = s.values.get(n.id);
                  return `
                    <tr>
                      <td>${esc(n.name)}</td>
                      <td>${signed(v.ax)}</td>
                      <td>${signed(v.ay)}</td>
                      <td>${signed(v.fx)}</td>
                      <td>${signed(v.fy)}</td>
                      <td>${fmt(v.normal, 3)}</td>
                      <td>${fmt(v.friction, 3)}</td>
                      <td>${fmt(maximumT(s, n.id), 3)}</td>
                    </tr>
                  `;
                }).join("")}
                </tbody>
              </table>
            </div>
            <p class="small">
              N·f 열은 접촉별 힘의 크기 합입니다.
              최대 T는 한 줄의 장력이며, 이동 도르래를 당기는 양쪽 장력의 합력과 다릅니다.
            </p>
            <div class="study-scroll">
              <table class="study-table">
                <thead><tr>
                  <th>줄</th>
                  <th>초기 T (N)</th>
                  <th>초기 경로 / 설정 길이 (m)</th>
                </tr></thead>
                <tbody>
                ${scene.ropes.map(r => `
                  <tr>
                    <td>${esc(r.name)}</td>
                    <td>${fmt(
                      s.rows.find(q => q.ropeId === r.id)?.force || 0,
                      3
                    )}</td>
                    <td>${fmt(geometry(
                      r,
                      new Map(scene.nodes.map(n => [n.id, n]))
                    ).length, 3)} / ${fmt(r.length, 3)}</td>
                  </tr>
                `).join("")}
                </tbody>
              </table>
            </div>
          `;

        const grades = [];

        if (s.reliable) {
          for (const n of engine.dynamic) {
            const value = s.values.get(n.id);

            for (const [key, answer] of [
              ["ax", value.ax],
              ["ay", value.ay],
              ["T", maximumT(s, n.id)]
            ]) {
              const text = study.predictions[n.id + ":" + key];
              if (text === undefined ||
                  text.trim() === "" ||
                  !Number.isFinite(Number(text))) continue;

              const ok = Math.abs(Number(text) - answer) <=
                Math.max(0.02, Math.abs(answer) * 0.01);

              grades.push(
                `${esc(n.name)} ${key}: ` +
                `${ok ? "✓ 허용 오차 내 정답" : "다시 확인"} ` +
                `(계산값 ${signed(answer)} ${key === "T" ? "N" : "m/s²"})`
              );
            }
          }
        }

        $("studyGrade").innerHTML = grades.join("<br>") ||
          "예측값을 입력하면 계산 결과를 확인할 때 비교합니다.";

        $("studyDiagrams").innerHTML = s.reliable
          ? engine.dynamic.map(n => `
              <div>
                <h3>${esc(n.name)} · 초기 상태</h3>
                <svg class="study-fbd" viewBox="0 0 420 330"
                  role="img"
                  aria-label="${esc(n.name)} 초기 자유물체도">
                  ${initialFBD(n, s.values.get(n.id))}
                </svg>
              </div>
            `).join("")
          : "초기 충격 상태에서는 중력과 외력 외에 유한한 순간 접촉력을 지정하지 않습니다.";
      }
    }

    if (study.revealed) {
      $("studyLive").innerHTML = `
        <table class="study-table study-live">
          <thead><tr>
            <th>물체</th><th>x (m)</th><th>y (m)</th>
            <th>vₓ (m/s)</th><th>vᵧ (m/s)</th>
            <th>현재 aₓ (m/s²)</th><th>현재 aᵧ (m/s²)</th>
            <th>현재 최대 T (N)</th>
          </tr></thead>
          <tbody>
          ${engine.dynamic.map(n => `
            <tr>
              <td>${esc(n.name)}</td>
              <td>${signed(n.x)}</td><td>${signed(n.y)}</td>
              <td>${signed(n.vx)}</td><td>${signed(n.vy)}</td>
              <td>${!s.reliable && engine.time === 0 ? "—" : signed(n.ax)}</td>
              <td>${!s.reliable && engine.time === 0 ? "—" : signed(n.ay)}</td>
              <td>${!s.reliable && engine.time === 0 ? "—" : fmt(currentT(n.id), 3)}</td>
            </tr>
          `).join("")}
          </tbody>
        </table>
      `;
    }
  }

  function renderForceSystem() {
    const s = engine.solution;
    const system = s.system;

    if (!s.reliable) {
      $("systemEquations").innerHTML = `
        <p class="notice">
          초기 충격 또는 수렴 문제 때문에 일정 가속도 연립 풀이를 제시하지 않습니다.
          일반 시뮬레이션에서는 시간 간격 동안의 평균 힘을 표시합니다.
        </p>
      `;
      return;
    }

    const symbols = system.active.map((r, i) =>
      (r.kind === "rope" ? "T" : r.kind === "normal" ? "N" : "f") +
      (i + 1)
    );
    const term = (coefficient, label) =>
      `(${signed(coefficient)})${label}`;

    const bodyLines = engine.dynamic.map(n => {
      const equation = (axis, force) =>
        `${fmt(n.m)} a${esc(n.name)},${axis} = ${signed(force)}` +
        system.active.map((r, i) => {
          const g = r.terms.get(n.id);
          return g && Math.abs(g[axis]) > 1e-8
            ? " + " + term(g[axis], symbols[i])
            : "";
        }).join("");

      return equation("x", n.fx) + "<br>" +
        equation("y", n.fy - n.m * scene.g);
    }).join("<br>");

    const constraints = s.rows
      .filter(r => r.kind !== "friction" && r.force > 1e-7)
      .map(r => {
        const terms = [...r.terms].flatMap(([id, g]) =>
          ["x", "y"]
            .filter(axis => Math.abs(g[axis]) > 1e-8)
            .map(axis => term(
              g[axis],
              "a" + esc(engine.map.get(id).name) + "," + axis
            ))
        );
        return `${esc(r.label)}: ${terms.join(" + ")} = ${fmt(r.target, 5)}`;
      }).join("<br>");

    function table(A, b, head) {
      if (!A.length) {
        return "<p class='small'>구속력 미지수 없음: 외력과 중력으로 직접 계산합니다.</p>";
      }

      return `
        <div class="table-wrap">
          <table>
            <thead><tr>
              <th>${head}</th>
              ${symbols.map(v => `<th>${v}</th>`).join("")}
              <th>우변</th>
            </tr></thead>
            <tbody>
            ${A.map((r, i) => `
              <tr>
                <th>${i + 1}</th>
                ${r.map(v => `<td>${fmt(v, 5)}</td>`).join("")}
                <td>${fmt(b[i], 5)}</td>
              </tr>
            `).join("")}
            </tbody>
          </table>
        </div>
      `;
    }

    const labelList = system.active.map((r, i) =>
      `${symbols[i]}: ${esc(r.label)}` +
      (r.kind === "friction"
        ? r.slide
          ? ` · 미끄러짐: f = ${fmt(-r.slide * r.mu, 3)}N`
          : " · 정지 마찰: 접선 가속도 0"
        : "")
    ).join("<br>");

    const reducedA = system.reduced.map(r => r.slice(0, -1));
    const reducedB = system.reduced.map(r => r[r.length - 1]);

    $("systemEquations").innerHTML = `
      <h3>1. 좌표와 힘을 정합니다</h3>
      <p class="small">
        오른쪽 +x, 위쪽 +y.
        중력은 (0, −mg), 줄 장력과 수직항력은 음수가 될 수 없습니다.
      </p>

      <h3>2. 물체마다 운동 방정식을 세웁니다</h3>
      <div class="formula">${bodyLines}</div>

      <h3>3. 줄 길이·접촉 구속을 추가합니다</h3>
      <div class="formula">${constraints || "현재 활성 구속 없음"}</div>
      <p class="small">
        줄에서는 ΣJᵢ·aᵢ = Σ|상대속도 수직성분|²/구간길이입니다.
        교과서처럼 구간 방향이 일정하면 오른쪽이 0입니다.
        이동 도르래의 양쪽 줄 성분은 같은 물체에 합산됩니다.
      </p>
      <p class="small">${labelList || "구속력 없음"}</p>

      <h3>4. 가속도를 소거해 구속력 연립계를 만듭니다</h3>
      <div class="formula">
        a = M⁻¹(F외부 + Jᵀλ)<br>
        (J M⁻¹ Jᵀ)λ = 목표값 − J M⁻¹F외부
      </div>
      <p class="small">
        여기서 λ는 힘(N)입니다.
        미끄럼 마찰 행은 f ± μN = 0으로 대체합니다.
        활성 장력·수직항력을 먼저 판별하고,
        정지 마찰 한계와 음의 힘 여부를 검사합니다.
      </p>
      ${table(system.A, system.b, "원래 연립계")}

      <h3>5. 가우스 소거법으로 풉니다</h3>
      <details>
        <summary>실제 행 연산 ${system.ops.length}개 보기</summary>
        <pre>${esc(system.ops.join("\n") || "행 연산 불필요")}</pre>
      </details>
      ${system.ok
        ? table(reducedA, reducedB, "소거 결과")
        : "<p>특이 연립계: 유일한 해를 확인하지 못했습니다.</p>"}
      <div class="formula">
        ${system.active.map((r, i) =>
          `${symbols[i]} = ${signed(r.force)} N`
        ).join("<br>") || "구속력 미지수가 없습니다."}
      </div>

      <h3>6. 뉴턴 방정식에 대입합니다</h3>
      <div class="formula">
        ${engine.dynamic.map(n => {
          const v = s.values.get(n.id);
          return `${esc(n.name)}: aₓ = ${signed(v.fx)} / ${fmt(n.m)} = ${signed(v.ax)} m/s²<br>` +
            `aᵧ = ${signed(v.fy)} / ${fmt(n.m)} = ${signed(v.ay)} m/s²`;
        }).join("<br>")}
      </div>
      <p class="notice">
        ${s.constant
          ? "이 결과는 현재 운동 구간에서 일정합니다. 충돌이나 접촉 상태가 바뀌기 전까지만 적용합니다."
          : "이것은 초기 순간의 연립 풀이입니다. 운동 중 줄 방향이나 접촉이 바뀌면 다시 계산해야 하므로 고정 답으로 사용하지 않습니다."}
      </p>
    `;
  }

  function analysis() {
    const n = engine.map.get(selected);

    if (!study.revealed) {
      $("fbd").innerHTML = `
        <text x="210" y="165" text-anchor="middle"
          fill="#67805d" font-size="13">
          위의 ‘계산 결과와 풀이 확인’을 눌러 주세요.
        </text>
      `;
      $("bodyEquations").innerHTML = `
        <p class="small">
          가속도와 장력을 먼저 예측해 보세요.
          예측 입력 없이 풀이를 확인할 수도 있습니다.
        </p>
      `;
      $("systemEquations").innerHTML = `
        <p class="small">
          문제 풀이 모드에서 결과를 확인하면
          물체별 방정식과 실제 연립계 소거 과정이 표시됩니다.
        </p>
      `;
      equationSolution = null;
      return;
    }

    if (n && DYNAMIC(n) &&
        !engine.solution.reliable && engine.time === 0) {
      $("fbd").innerHTML = initialFBD(n, { forces: n.forces });
      $("bodyEquations").innerHTML = `
        <p class="notice">
          초기 속도와 구속이 맞지 않아 충격이 먼저 발생합니다.
          중력·외력만 그림에 표시했습니다.
          유한한 순간 장력·알짜힘·가속도를 지정하지 않습니다.
          한 단계를 진행하면 그 시간 간격의 평균값을 볼 수 있습니다.
        </p>
      `;
    } else if (n && DYNAMIC(n)) {
      $("fbd").innerHTML = initialFBD(n, { forces: n.forces });

      const names = {
        gravity: "중력",
        normal: "수직항력",
        friction: "마찰력",
        rope: "장력",
        external: "외력"
      };

      $("bodyEquations").innerHTML = `
        <p class="small">
          ${esc(n.name)} ·
          ${engine.time > 0 ? "현재 시각" : "초기 조건"}
          ${fmt(engine.time)} s
        </p>
        <div class="equation-step">
          <b>1</b>
          <p>
            성분별 힘(N)<br>
            ${n.forces.map(f =>
              `${names[f.kind]} ${esc(f.name)}: (${signed(f.x)}, ${signed(f.y)})`
            ).join("<br>")}
          </p>
        </div>
        <div class="equation-step">
          <b>2</b>
          <p>
            ΣFₓ = ${signed(n.m * n.ax)} N<br>
            ΣFᵧ = ${signed(n.m * n.ay)} N
          </p>
        </div>
        <div class="formula">
          aₓ = ${signed(n.m * n.ax)} / ${fmt(n.m)} = ${signed(n.ax)} m/s²<br>
          aᵧ = ${signed(n.m * n.ay)} / ${fmt(n.m)} = ${signed(n.ay)} m/s²
        </div>
        <p class="small">
          ${engine.solution.reliable
            ? "초기 연립 풀이는 아래에서 확인하세요."
            : "초기 충격에서는 유한한 순간 힘을 정하지 않습니다. 실행 후 표시되는 힘은 시간 간격의 평균값입니다."}
        </p>
      `;
    } else {
      $("fbd").innerHTML = `
        <text x="210" y="165" text-anchor="middle"
          fill="#67805d" font-size="14">
          물체 또는 이동 도르래를 선택하세요.
        </text>
      `;
      $("bodyEquations").innerHTML = `
        <p class="small">
          선택한 항목의 위치와 조건은 속성에서 변경할 수 있습니다.
        </p>
      `;
    }

    if (equationSolution !== engine.solution) {
      renderForceSystem();
      equationSolution = engine.solution;
    }
  }

  function message(text, error = false) {
    $("notice").textContent = text;
    $("notice").classList.toggle("error", error);
  }

  function pause(label = "일시정지") {
    running = false;
    lastTime = 0;
    accumulated = 0;
    $("play").textContent = finished ? "↺ 다시 시작" : "▶ 시작";
    $("status").textContent = label;
    uiDirty = true;
  }

  function reset(announce = false) {
    pause("실험 준비");
    finished = false;
    $("play").textContent = "▶ 시작";
    study.revealed = false;
    study.predictions = {};
    equationSolution = null;
    engine = createEngine(scene, textbookEnabled);
    uiDirty = true;
    renderStudy(true);
    render(true);

    if (announce) {
      message(
        "초기 배치와 초기 속도로 돌아왔습니다. 사전 풀이를 다시 확인할 수 있습니다."
      );
    }
  }

  function changed() {
    reset();
    const errors = validate(scene);
    if (errors.length) message(errors.join("\n"), true);
    else message("설정이 반영되었습니다. 시작을 눌러 실험하세요.");
  }

  function start() {
    if (running) {
      pause();
      render(true);
      return;
    }

    if (finished) reset();

    const errors = validate(scene);
    if (errors.length) {
      message(errors.join("\n"), true);
      return;
    }

    if (route.length) {
      message("연결 중인 줄을 먼저 완성하거나 취소하세요.", true);
      return;
    }

    setTool("select");
    running = true;
    lastTime = 0;
    accumulated = 0;
    $("play").textContent = "Ⅱ 일시정지";
    $("status").textContent = "실험 진행 중";

    message(
      engine.useTextbook && engine.solution.constant
        ? "교과서 재생: 가속도·장력은 현재 구간에서 일정하며, 조건이 바뀌는 경계에서 멈춥니다."
        : "일반 재생: 줄 방향·접촉 변화에 따라 힘이 변할 수 있습니다. 반발계수 0, 위치 보정은 속도와 분리합니다."
    );

    uiDirty = true;
    render(true);
  }

  function checkRuntime() {
    if (engine.event) {
      pause("운동 구간 경계");
      message(
        engine.event +
        " 교과서 재생은 여기서 멈춥니다. 체크를 끄면 초기 상태부터 일반 운동으로 실험할 수 있습니다."
      );
      renderStudy();
      render(true);
      return false;
    }

    const invalid = engine.dynamic.some(n =>
      ![n.x, n.y, n.vx, n.vy, n.ax, n.ay].every(Number.isFinite)
    );

    if (invalid || engine.residual > 0.03) {
      pause("계산 중단");
      message(
        "구속조건 계산이 수렴하지 않았습니다. 중복 접촉·줄 연결을 줄이거나 배치를 바꾸세요. 이 구조의 결과는 신뢰할 수 없습니다.",
        true
      );
      return false;
    }

    for (const r of scene.ropes) {
      for (let i = 1; i < r.path.length; i++) {
        const a = engine.map.get(r.path[i - 1]);
        const b = engine.map.get(r.path[i]);

        if (hypot(a.x - b.x, a.y - b.y) < 0.035) {
          pause("지원 범위 도달");
          message(
            "인접한 줄 연결점이 거의 겹쳐 경로 방향을 정의할 수 없습니다. 배치를 바꾸세요.",
            true
          );
          return false;
        }
      }
    }

    if (engine.time >= MAX_TIME ||
        engine.dynamic.some(n =>
          Math.abs(n.x) > 1000 || Math.abs(n.y) > 1000)) {
      finished = true;
      pause("기록 범위 도달");
      message(
        "최대 기록 시간 또는 좌표 범위에 도달했습니다. 초기화 후 다시 실험하세요."
      );
      return false;
    }

    return true;
  }

  function project(x, y) {
    return {
      x: (x - view.x) / view.w * 1000,
      y: 625 - (y - view.y) / view.h * 625
    };
  }

  function unproject(clientX, clientY) {
    const p = $("workspace").createSVGPoint();
    p.x = clientX;
    p.y = clientY;
    const q = p.matrixTransform(
      $("workspace").getScreenCTM().inverse()
    );
    return {
      x: view.x + q.x / 1000 * view.w,
      y: view.y + (625 - q.y) / 625 * view.h
    };
  }

  const px = m => m / view.w * 1000;

  function fitView() {
    const nodes = engine ? engine.nodes : scene.nodes;

    if (!nodes.length) {
      view = { x: 0, y: 0, w: 16, h: 10 };
      return;
    }

    const points = [];

    nodes.forEach(n => {
      if (SURFACE(n)) {
        const a = n.angle * Math.PI / 180;
        for (const k of [-1, 1]) {
          points.push({
            x: n.x + k * n.length / 2 * Math.cos(a),
            y: n.y + k * n.length / 2 * Math.sin(a)
          });
        }
      } else {
        points.push(n);
      }
    });

    const minX = Math.min(...points.map(p => p.x)) - 1.5;
    const maxX = Math.max(...points.map(p => p.x)) + 1.5;
    const minY = Math.min(...points.map(p => p.y)) - 1.5;
    const maxY = Math.max(...points.map(p => p.y)) + 1.5;
    const w = Math.max(10, maxX - minX, (maxY - minY) * 1.6);
    const h = w / 1.6;

    view = {
      x: (minX + maxX - w) / 2,
      y: (minY + maxY - h) / 2,
      w,
      h
    };
  }

  const forceColor = kind => ({
    gravity: "#ffbf7a",
    external: "#c5afff",
    normal: "#83c5ff",
    friction: "#ff9fbc",
    rope: "#a6efbd"
  }[kind] || "#ffffff");

  function arrow(x, y, dx, dy, color, label = "", light = false) {
    const magnitude = hypot(dx, dy);
    if (magnitude < 0.015) return "";

    const len = clamp(magnitude * 3, 24, 110);
    const ux = dx / magnitude;
    const uy = -dy / magnitude;
    const ex = x + ux * len;
    const ey = y + uy * len;
    const bx = ex - ux * 9;
    const by = ey - uy * 9;

    return `
      <g pointer-events="none">
        <path d="M${x} ${y}L${ex} ${ey}"
          stroke="${color}" stroke-width="3"/>
        <path d="M${ex} ${ey}
          L${bx - uy * 4} ${by + ux * 4}
          L${bx + uy * 4} ${by - ux * 4}Z"
          fill="${color}"/>
        ${label ? `
          <text x="${ex + ux * 10}" y="${ey + uy * 10 + 4}"
            text-anchor="${ux > 0.25 ? "start" : ux < -0.25 ? "end" : "middle"}"
            font-size="12"
            fill="${light ? "#284a34" : color}"
            stroke="${light ? "#f5f7f1" : "#142f29"}"
            stroke-width="3" paint-order="stroke">
            ${esc(label)}
          </text>
        ` : ""}
      </g>
    `;
  }

  function draw() {
    let s = `
      <defs>
        <pattern id="grid"
          width="${px(1)}" height="${px(1)}"
          patternUnits="userSpaceOnUse"
          x="${project(0, 0).x}" y="${project(0, 0).y}">
          <path d="M${px(1)} 0H0V${px(1)}"
            fill="none" stroke="#45634e"
            stroke-width=".6" opacity=".55"/>
        </pattern>
      </defs>
      <rect width="1000" height="625" fill="url(#grid)"/>
    `;

    const origin = project(0, 0);
    s += `
      <path d="M0 ${origin.y}H1000M${origin.x} 0V625"
        stroke="#63856b" stroke-width="1" opacity=".6"/>
      <text x="20" y="28" fill="#9fbea6" font-size="12">
        +y ↑　+x →　1格 = 1 m
      </text>
    `;

    if ($("showTrails").checked) {
      for (const n of engine.dynamic) {
        const path = engine.history.slice(-400).map((r, i) => {
          const b = r.bodies[n.id];
          const p = project(b.x, b.y);
          return (i ? "L" : "M") + p.x + " " + p.y;
        }).join(" ");

        s += `
          <path d="${path}" fill="none"
            stroke="#b7dcae" stroke-width="1.5" opacity=".35"/>
        `;
      }
    }

    for (const r of scene.ropes) {
      const geo = geometry(r, engine.map);
      const slack = r.length - geo.length > 0.015;
      const points = r.path.map(id => {
        const n = engine.map.get(id);
        return project(n.x, n.y);
      });

      let path = `M${points[0].x} ${points[0].y}`;

      for (let i = 1; i < points.length; i++) {
        const a = points[i - 1], b = points[i];
        path += slack
          ? `Q${(a.x + b.x) / 2} ${(a.y + b.y) / 2 + 22} ${b.x} ${b.y}`
          : `L${b.x} ${b.y}`;
      }

      const row = engine.rows.find(q => q.ropeId === r.id);

      s += `
        <g data-id="${r.id}" style="cursor:pointer">
          <path d="${path}" fill="none"
            stroke="transparent" stroke-width="18"/>
          <path d="${path}" fill="none"
            stroke="${r.id === selected ? "#ffee9c" : slack ? "#72937a" : "#dbebbc"}"
            stroke-width="${r.id === selected ? 4 : 2.5}"
            ${slack ? 'stroke-dasharray="7 5"' : ""}/>
        </g>
      `;

      const p = points[Math.floor(points.length / 2)];
      const label = slack
        ? "느슨함"
        : study.revealed
          ? "T " + fmt((row?.impulse || 0) /
              (engine.lastH || DT)) + " N"
          : "풀이에서 장력 확인";

      s += `
        <text x="${p.x + 12}" y="${p.y - 18}"
          fill="#c3d8ad" font-size="11" pointer-events="none">
          ${esc(r.name)} · ${label}
        </text>
      `;
    }

    let dynamicIndex = 0;

    for (const n of engine.nodes) {
      const p = project(n.x, n.y);
      const chosen = selected === n.id;
      s += `
        <g data-id="${n.id}"
          style="cursor:${tool === "rope" ? "crosshair" : "grab"}">
      `;

      if (SURFACE(n)) {
        const a = n.angle * Math.PI / 180;
        const dx = Math.cos(a) * n.length / 2;
        const dy = Math.sin(a) * n.length / 2;
        const l = project(n.x - dx, n.y - dy);
        const r = project(n.x + dx, n.y + dy);

        s += `
          <path d="M${l.x} ${l.y}L${r.x} ${r.y}"
            stroke="transparent" stroke-width="25"/>
          <path d="M${l.x} ${l.y + 6}L${r.x} ${r.y + 6}"
            stroke="#416c49" stroke-width="11"/>
          <path d="M${l.x} ${l.y}L${r.x} ${r.y}"
            stroke="${chosen ? "#ffee9c" : "#afd1a3"}" stroke-width="4"/>
          <text x="${p.x}" y="${p.y + 30}" text-anchor="middle"
            fill="#a2c29a" font-size="12">
            ${esc(n.name)} ${fmt(n.angle, 0)}°
          </text>
        `;
      } else if (n.kind === "anchor") {
        s += `
          <path d="M${p.x - 18} ${p.y - 16}H${p.x + 18}"
            stroke="#a8c6a2" stroke-width="6"/>
          <circle cx="${p.x}" cy="${p.y}" r="9"
            fill="#7c9e77"
            stroke="${chosen ? "#ffee9c" : "#bed9ae"}" stroke-width="3"/>
        `;
      } else if (n.kind === "fixed" || n.kind === "moving") {
        const r = n.kind === "fixed"
          ? 21 : clamp(px(n.radius), 13, 40);

        if (n.kind === "fixed") {
          s += `
            <path d="M${p.x} ${p.y - r}V${p.y - r - 16}
              M${p.x - 16} ${p.y - r - 16}H${p.x + 16}"
              stroke="#88ad83" stroke-width="5"/>
          `;
        }

        s += `
          <circle cx="${p.x}" cy="${p.y}"
            r="${r + (chosen ? 6 : 0)}"
            fill="#234b38"
            stroke="${chosen ? "#ffee9c" : "#a8d498"}" stroke-width="3"/>
          <circle cx="${p.x}" cy="${p.y}" r="5" fill="#c6e9ad"/>
          <path d="M${p.x - r * .65} ${p.y}H${p.x + r * .65}
            M${p.x} ${p.y - r * .65}V${p.y + r * .65}"
            stroke="#7ca372" stroke-width="2"/>
          <text x="${p.x + r + 9}" y="${p.y + 4}"
            fill="#d4e9c5" font-size="12">
            ${esc(n.name)}${n.kind === "moving" ? " · " + fmt(n.m, 1) + " kg" : ""}
          </text>
        `;
      } else {
        const color = COLORS[dynamicIndex++ % COLORS.length];
        const r = px(n.radius);

        s += `
          ${chosen ? `
            <circle cx="${p.x}" cy="${p.y}" r="${r + 6}"
              fill="none" stroke="#ddeb9f" stroke-width="2" opacity=".65"/>
          ` : ""}
          <circle cx="${p.x}" cy="${p.y}" r="${r}"
            fill="${color}" stroke="#dceccf" stroke-width="1.5"/>
          <text x="${p.x}" y="${p.y + 1}" text-anchor="middle"
            fill="#234731" font-size="14" font-weight="800">
            ${esc(n.name)}
          </text>
          <text x="${p.x}" y="${p.y + 16}" text-anchor="middle"
            fill="#315b3c" font-size="10">${fmt(n.m, 1)} kg</text>
        `;
      }

      s += "</g>";
    }

    const body = engine.map.get(selected);

    if (body && DYNAMIC(body)) {
      const p = project(body.x, body.y);

      if ($("showForces").checked) {
        body.forces.forEach((f, i) => {
          const shift = (i - 1) * 7;
          s += arrow(
            p.x + shift, p.y, f.x, f.y,
            forceColor(f.kind), f.name
          );
        });
      }

      if ($("showVelocity").checked) {
        s += arrow(
          p.x,
          p.y - px(body.radius) - 30,
          body.vx * 3,
          body.vy * 3,
          "#d8c6ff",
          "v"
        );
      }
    }

    if (route.length) {
      const points = route.map(id => {
        const n = engine.map.get(id);
        return project(n.x, n.y);
      });

      s += `
        <polyline
          points="${points.map(p => p.x + "," + p.y).join(" ")}"
          fill="none" stroke="#ffee9c" stroke-width="3"
          stroke-dasharray="6 5" pointer-events="none"/>
      `;

      points.forEach((p, i) => {
        s += `
          <circle cx="${p.x}" cy="${p.y}" r="9"
            fill="#ffee9c" pointer-events="none"/>
          <text x="${p.x}" y="${p.y + 4}" text-anchor="middle"
            font-size="11" fill="#294732" pointer-events="none">
            ${i + 1}
          </text>
        `;
      });
    }

    const visible = engine.dynamic.some(n => {
      const p = project(n.x, n.y);
      return p.x >= 0 && p.x <= 1000 && p.y >= 0 && p.y <= 625;
    });

    if (!visible) {
      s += `
        <rect x="250" y="260" width="500" height="70"
          rx="12" fill="#203d2f" opacity=".95"/>
        <text x="500" y="300" text-anchor="middle"
          fill="#cfe4be" font-size="16">
          물체가 화면 밖에 있습니다. ‘전체 보기’를 누르세요.
        </text>
      `;
    }

    $("workspace").innerHTML = s;
  }

  function inputField(key, label, value, min, max, step = "0.1") {
    return `
      <label class="field">
        ${label}
        <input data-prop="${key}" type="number"
          value="${value}" min="${min}" max="${max}" step="${step}"
          ${running ? "disabled" : ""}>
      </label>
    `;
  }

  function inspector() {
    const n = scene.nodes.find(n => n.id === selected);
    const r = scene.ropes.find(r => r.id === selected);
    $("deleteItem").disabled = running || (!n && !r);

    if (r) {
      const geo = geometry(r, engine.map);
      $("inspector").innerHTML = `
        <label class="field">
          이름
          <input data-prop="name" value="${esc(r.name)}"
            maxlength="60" ${running ? "disabled" : ""}>
        </label>
        ${inputField("length", "줄 전체 길이 (m)", r.length, 0.1, 200, "0.01")}
        <p class="small">
          현재 경로 ${fmt(geo.length, 3)} m<br>
          ${r.path.map(id =>
            esc(scene.nodes.find(n => n.id === id).name)
          ).join(" → ")}
        </p>
        <button id="autoLength" ${running ? "disabled" : ""}>
          현재 배치에 맞춰 자동 길이
        </button>
      `;

      $("autoLength").onclick = () => {
        r.length = geometry(
          r, new Map(scene.nodes.map(n => [n.id, n]))
        ).length;
        changed();
      };
      return;
    }

    if (!n) {
      $("inspector").innerHTML =
        '<p class="small">장면이나 목록에서 항목을 선택하세요.</p>';
      return;
    }

    let html = `
      <p class="small">${TITLES[n.kind]}</p>
      <label class="field">
        이름
        <input data-prop="name" maxlength="60"
          value="${esc(n.name)}" ${running ? "disabled" : ""}>
      </label>
      <div class="field-grid">
      ${inputField("x", "초기 x (m)", n.x, -1000, 1000, "0.01")}
      ${inputField("y", "초기 y (m)", n.y, -1000, 1000, "0.01")}
    `;

    if (DYNAMIC(n)) {
      html += inputField("m", "질량 (kg)", n.m, 0.05, 100, "0.05");
      html += inputField(
        "radius", "접촉 반지름 (m)", n.radius, 0.1, 1, "0.01"
      );
      html += inputField("mu", "마찰계수 μ", n.mu, 0, 2, "0.01");
      html += inputField("fx", "외력 x (N)", n.fx, -200, 200, "0.5");
      html += inputField("fy", "외력 y (N)", n.fy, -200, 200, "0.5");
      html += inputField(
        "vx", "초기 속도 x (m/s)", n.vx, -30, 30, "0.1"
      );
      html += inputField(
        "vy", "초기 속도 y (m/s)", n.vy, -30, 30, "0.1"
      );
    }

    if (SURFACE(n)) {
      html += inputField("length", "길이 (m)", n.length, 0.5, 40, "0.1");
      html += inputField("mu", "마찰계수 μ", n.mu, 0, 2, "0.01");
      if (n.kind === "ramp") {
        html += inputField("angle", "경사각 (°)", n.angle, -80, 80, "1");
      }
    }

    $("inspector").innerHTML = html + "</div>";
  }

  function sceneList() {
    $("sceneList").innerHTML =
      [...scene.nodes, ...scene.ropes].map(n => `
        <button data-list-id="${n.id}"
          class="${n.id === selected ? "chosen" : ""}">
          ${esc(TITLES[n.kind])} · ${esc(n.name)}
        </button>
      `).join("");
  }

  function metric() {
    const b = engine.map.get(selected);
    const dynamic = b && DYNAMIC(b);

    $("metricName").textContent = b?.name || "—";
    $("metricV").textContent = dynamic
      ? fmt(hypot(b.vx, b.vy)) + " m/s" : "—";
    $("metricA").textContent = dynamic
      ? fmt(hypot(b.ax, b.ay)) + " m/s²" : "—";
    $("metricF").textContent = dynamic
      ? fmt(b.m * hypot(b.ax, b.ay)) + " N" : "—";
    $("clock").textContent = fmt(engine.time) + " s";

    if (!study.revealed) {
      $("metricA").textContent = "풀이에서 확인";
      $("metricF").textContent = "풀이에서 확인";
    } else if (!engine.solution.reliable && engine.time === 0) {
      $("metricA").textContent = "초기 충격";
      $("metricF").textContent = "단일값 없음";
    }
  }

  function graph() {
    const n = engine.map.get(selected);

    if (!n || !DYNAMIC(n)) {
      $("chart").innerHTML = `
        <text x="400" y="150" text-anchor="middle" fill="#6c826a">
          움직이는 항목을 선택하세요.
        </text>
      `;
      $("energySummary").textContent = "";
      return;
    }

    const key = $("graphType").value;
    const start = Math.max(0, engine.time - 15);
    const end = Math.max(start + 3, engine.time);
    const rows = engine.history.filter(r => r.t >= start);

    rows.push({
      t: engine.time,
      bodies: {
        [n.id]: {
          x: n.x, y: n.y,
          vx: n.vx, vy: n.vy,
          ax: n.ax, ay: n.ay,
          speed: hypot(n.vx, n.vy),
          energy: 0.5 * n.m * (n.vx * n.vx + n.vy * n.vy) +
            n.m * scene.g * n.y
        }
      }
    });

    const values = rows.map(r => r.bodies[n.id]?.[key])
      .filter(Number.isFinite);

    let lo = Math.min(0, ...values);
    let hi = Math.max(0, ...values);
    const pad = Math.max(0.5, (hi - lo) * 0.12);
    lo -= pad;
    hi += pad;

    const X = t => 65 + (t - start) / (end - start) * 700;
    const Y = v => 255 - (v - lo) / (hi - lo) * 220;
    let s = "";

    for (let i = 0; i <= 4; i++) {
      const value = lo + (hi - lo) * i / 4;
      const y = Y(value);
      const t = start + (end - start) * i / 4;

      s += `
        <path d="M65 ${y}H765" stroke="#dce5d5"/>
        <text x="56" y="${y + 4}" text-anchor="end"
          font-size="11" fill="#72846a">${fmt(value, 1)}</text>
        <text x="${X(t)}" y="280" text-anchor="middle"
          font-size="11" fill="#72846a">${fmt(t, 1)}</text>
      `;
    }

    const path = rows.map((r, i) => {
      const value = r.bodies[n.id]?.[key];
      return Number.isFinite(value)
        ? (i ? "L" : "M") + X(r.t) + " " + Y(value)
        : "";
    }).join(" ");

    s += `
      <path d="${path}" fill="none" stroke="#208459" stroke-width="2.5"/>
      <text x="778" y="281" font-size="12" fill="#72846a">s</text>
    `;

    $("chart").innerHTML = s;

    const K = engine.dynamic.reduce((v, b) =>
      v + 0.5 * b.m * (b.vx * b.vx + b.vy * b.vy), 0
    );
    const U = engine.dynamic.reduce((v, b) =>
      v + b.m * scene.g * b.y, 0
    );

    $("energySummary").innerHTML = `
      <div class="formula">
        계 전체 K = ${fmt(K)} J<br>
        U = ${fmt(U)} J<br>
        K + U = ${fmt(K + U)} J
      </div>
      <p class="small">
        위치에너지 기준은 세계 좌표 y=0입니다.
        외력·마찰·비탄성 충돌·줄의 충격 때문에 역학적 에너지가 바뀔 수 있습니다.
        일반 모드에는 수치 적분 오차도 존재합니다.
      </p>
    `;
  }

  function render(full = false) {
    draw();
    metric();
    renderStudy();

    if (uiDirty || full) {
      inspector();
      sceneList();
      $("gravity").value = scene.g;
      $("gravity").disabled = running;
      $("loadExample").disabled = running;
      $("openJSON").disabled = running;
      $("loadLocal").disabled = running;
      uiDirty = false;
    }

    if (activeTab === "analysis") analysis();
    if (activeTab === "graphs") graph();
  }

  function setTool(value) {
    if (running && !["select", "pan"].includes(value)) {
      message(
        "배치와 연결은 일시정지 또는 초기화 후 변경할 수 있습니다.",
        true
      );
      return;
    }

    tool = value;
    if (value !== "rope") route = [];

    document.querySelectorAll("[data-tool]").forEach(b => {
      const on = b.dataset.tool === value;
      b.classList.toggle("active", on);
      b.setAttribute("aria-pressed", String(on));
    });

    $("ropeActions").hidden = value !== "rope";
    $("routeText").textContent = route.map(id =>
      scene.nodes.find(n => n.id === id)?.name
    ).join(" → ") || "연결점을 순서대로 선택하세요.";

    $("toolHint").textContent = {
      select: "항목을 눌러 선택하고 드래그하세요. 정확한 위치와 조건은 오른쪽 속성에서 입력합니다.",
      body: "작업 공간을 누르면 물체가 추가됩니다.",
      platform: "작업 공간을 누르면 고정된 수평 발판이 추가됩니다.",
      ramp: "작업 공간을 누르면 경사면이 추가됩니다. 각도는 속성에서 변경합니다.",
      fixed: "작업 공간을 누르면 고정 도르래가 추가됩니다.",
      moving: "작업 공간을 누르면 질량을 가진 이동 도르래가 추가됩니다.",
      anchor: "작업 공간을 누르면 줄을 묶는 고정점이 추가됩니다.",
      rope: "시작점 → 중간 도르래 → 끝점 순서로 누르세요. 마지막에 줄 완성을 누릅니다.",
      pan: "빈 공간을 드래그해 화면을 이동합니다."
    }[value];

    draw();
  }

  document.querySelectorAll("[data-tool]").forEach(b => {
    b.onclick = () => setTool(b.dataset.tool);
  });

  $("workspace").addEventListener("pointerdown", e => {
    if (e.button !== 0 && e.pointerType === "mouse") return;
    e.preventDefault();

    const p = unproject(e.clientX, e.clientY);
    const target = e.target.closest("[data-id]");
    const id = target?.dataset.id;

    if (tool === "pan") {
      drag = {
        kind: "pan",
        clientX: e.clientX,
        clientY: e.clientY,
        view: { ...view }
      };
      $("workspace").setPointerCapture(e.pointerId);
      return;
    }

    if (tool === "rope") {
      if (running) return;
      const n = scene.nodes.find(n => n.id === id);

      if (!n || SURFACE(n)) {
        message(
          "물체·도르래·고정점을 선택하세요. 발판에는 연결할 수 없습니다.",
          true
        );
        return;
      }

      if (route.includes(id)) {
        message("같은 연결점을 반복할 수 없습니다.", true);
        return;
      }

      route.push(id);
      setTool("rope");
      return;
    }

    if (!["select", "pan", "rope"].includes(tool)) {
      if (running) return;

      if (scene.nodes.length >= 40) {
        message("항목은 최대 40개입니다.", true);
        return;
      }

      const n = node(tool, p.x, p.y, {
        name: TITLES[tool] + " " +
          (scene.nodes.filter(n => n.kind === tool).length + 1)
      });

      scene.nodes.push(n);
      selected = n.id;
      changed();
      setTool("select");
      return;
    }

    if (id) {
      selected = id;
      uiDirty = true;
      const n = scene.nodes.find(n => n.id === id);

      if (n && !running) {
        const live = engine.map.get(id);
        drag = {
          kind: "node",
          id,
          start: p,
          x: live.x,
          y: live.y,
          moved: false
        };
        $("workspace").setPointerCapture(e.pointerId);
      }

      render(true);
    }
  });

  $("workspace").addEventListener("pointermove", e => {
    if (!drag) return;

    if (drag.kind === "pan") {
      const rect = $("workspace").getBoundingClientRect();
      view.x = drag.view.x -
        (e.clientX - drag.clientX) / rect.width * drag.view.w;
      view.y = drag.view.y +
        (e.clientY - drag.clientY) / rect.height * drag.view.h;
      draw();
      return;
    }

    const p = unproject(e.clientX, e.clientY);

    if (!drag.moved &&
        hypot(p.x - drag.start.x, p.y - drag.start.y) *
          1000 / view.w < 3) return;

    if (!drag.moved && engine.time > 0) reset();

    const n = scene.nodes.find(n => n.id === drag.id);
    n.x = Math.round(clamp(
      drag.x + p.x - drag.start.x, -1000, 1000
    ) * 100) / 100;
    n.y = Math.round(clamp(
      drag.y + p.y - drag.start.y, -1000, 1000
    ) * 100) / 100;

    const live = engine.map.get(n.id);
    live.x = n.x;
    live.y = n.y;
    drag.moved = true;
    draw();
  });

  function endDrag(e) {
    if (!drag) return;
    const moved = drag.kind === "node" && drag.moved;
    drag = null;

    if ($("workspace").hasPointerCapture(e.pointerId)) {
      $("workspace").releasePointerCapture(e.pointerId);
    }
    if (moved) changed();
  }

  $("workspace").addEventListener("pointerup", endDrag);
  $("workspace").addEventListener("pointercancel", endDrag);

  $("finishRope").onclick = () => {
    if (route.length < 2) {
      message("줄의 시작과 끝을 선택하세요.", true);
      return;
    }

    const r = rope(scene.nodes, route.slice());
    r.name = "줄 " + (scene.ropes.length + 1);
    const candidate = copy(scene);
    candidate.ropes.push(r);

    const errors = validate(candidate);
    if (errors.length) {
      message(errors.join("\n"), true);
      return;
    }

    scene.ropes.push(r);
    selected = r.id;
    route = [];
    setTool("select");
    changed();
  };

  $("cancelRope").onclick = () => {
    route = [];
    setTool("select");
  };

  $("inspector").addEventListener("change", e => {
    const key = e.target.dataset.prop;
    if (!key || running) return;

    const n = scene.nodes.find(n => n.id === selected) ||
      scene.ropes.find(r => r.id === selected);
    if (!n) return;

    const old = n[key];

    if (key === "name") {
      n.name = e.target.value.trim().slice(0, 60) || TITLES[n.kind];
    } else {
      if (!e.target.checkValidity() || e.target.value === "") {
        e.target.value = old;
        message("입력 범위를 확인하세요.", true);
        return;
      }
      n[key] = Number(e.target.value);
    }

    changed();
  });

  $("sceneList").onclick = e => {
    const b = e.target.closest("[data-list-id]");
    if (b) {
      selected = b.dataset.listId;
      uiDirty = true;
      render(true);
    }
  };

  function removeSelected() {
    if (running) return;
    const n = scene.nodes.find(n => n.id === selected);

    if (n) {
      scene.nodes = scene.nodes.filter(q => q.id !== selected);
      scene.ropes = scene.ropes.filter(r => !r.path.includes(selected));
    } else {
      scene.ropes = scene.ropes.filter(r => r.id !== selected);
    }

    selected = scene.nodes.find(DYNAMIC)?.id ||
      scene.nodes[0]?.id || null;
    route = [];
    changed();
  }

  $("deleteItem").onclick = removeSelected;

  document.addEventListener("keydown", e => {
    if (["INPUT", "SELECT", "TEXTAREA"].includes(e.target.tagName)) return;

    if (e.key === "Delete") {
      e.preventDefault();
      removeSelected();
    }

    if (e.key === "Escape") {
      drag = null;
      route = [];
      setTool("select");
    }
  });

  $("gravity").onchange = e => {
    if (!e.target.checkValidity() || e.target.value === "") {
      e.target.value = scene.g;
      return;
    }
    scene.g = Number(e.target.value);
    changed();
  };

  $("play").onclick = start;
  $("reset").onclick = () => reset(true);

  $("step").onclick = () => {
    pause();
    const errors = validate(scene);

    if (errors.length) {
      message(errors.join("\n"), true);
      return;
    }

    if (finished) reset();

    let remaining = 0.01;
    while (remaining > 1e-10) {
      const h = Math.min(DT, remaining);
      integrate(engine, h);
      remaining -= h;
      if (!checkRuntime()) break;
    }

    if (!engine.event) $("status").textContent = "0.01초 진행";
    render(true);
  };

  ["showForces", "showVelocity", "showTrails"].forEach(id => {
    $(id).onchange = () => render();
  });

  $("fit").onclick = () => {
    fitView();
    draw();
  };

  function zoom(factor) {
    const cx = view.x + view.w / 2;
    const cy = view.y + view.h / 2;
    const w = clamp(view.w * factor, 4, 3000);
    const h = w / 1.6;
    view = { x: cx - w / 2, y: cy - h / 2, w, h };
    draw();
  }

  $("zoomIn").onclick = () => zoom(0.8);
  $("zoomOut").onclick = () => zoom(1.25);

  $("workspace").addEventListener("wheel", e => {
    if (!e.ctrlKey) return;
    e.preventDefault();
    zoom(e.deltaY > 0 ? 1.08 : 0.92);
  }, { passive: false });

  function loadScene(s) {
    const errors = validate(s);
    if (errors.length) {
      message(errors.join("\n"), true);
      return false;
    }

    scene = copy(s);
    selected = scene.nodes.find(DYNAMIC)?.id || null;
    route = [];
    reset();
    fitView();
    setTool("select");
    render(true);
    message("실험을 불러왔습니다.");
    return true;
  }

  $("loadExample").onclick = () => {
    loadScene(example($("example").value));
  };

  function download(name, content, type) {
    const url = URL.createObjectURL(
      new Blob([content], { type })
    );
    const a = document.createElement("a");
    a.href = url;
    a.download = name;
    document.body.append(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1500);
  }

  $("saveJSON").onclick = () => {
    download(
      "physics-experiment.json",
      JSON.stringify(scene, null, 2),
      "application/json"
    );
  };

  $("openJSON").onclick = () => {
    $("fileInput").value = "";
    $("fileInput").click();
  };

  $("fileInput").onchange = async e => {
    const f = e.target.files[0];
    if (!f) return;

    if (f.size > 250000) {
      message("실험 파일은 250 KB 이하여야 합니다.", true);
      return;
    }

    try {
      loadScene(JSON.parse(await f.text()));
    } catch {
      message("JSON 파일을 읽을 수 없습니다.", true);
    }
  };

  $("saveLocal").onclick = () => {
    try {
      localStorage.setItem("physics-free-v1", JSON.stringify(scene));
      message("이 브라우저에 실험을 저장했습니다.");
    } catch {
      message(
        "브라우저가 저장소 사용을 허용하지 않습니다. JSON 저장을 이용하세요.",
        true
      );
    }
  };

  $("loadLocal").onclick = () => {
    try {
      const s = JSON.parse(localStorage.getItem("physics-free-v1"));
      if (!s) throw Error();
      loadScene(s);
    } catch {
      message("불러올 수 있는 브라우저 저장 실험이 없습니다.", true);
    }
  };

  $("csv").onclick = () => {
    const fields = ["x", "y", "vx", "vy", "ax", "ay", "energy"];
    const labels = [
      "x(m)", "y(m)", "vx(m/s)", "vy(m/s)",
      "ax(m/s²)", "ay(m/s²)", "E(J)"
    ];
    const quote = value =>
      '"' + String(value).replace(/"/g, '""') + '"';

    let csv = "\uFEFF" + [
      "시간(s)",
      ...engine.dynamic.flatMap(n =>
        labels.map(k => n.name + " " + k)
      )
    ].map(quote).join(",") + "\r\n";

    csv += engine.history.map(r => [
      r.t.toFixed(5),
      ...engine.dynamic.flatMap(n =>
        fields.map(k => r.bodies[n.id][k].toFixed(6))
      )
    ].join(",")).join("\r\n");

    download("physics-measurements.csv", csv, "text/csv;charset=utf-8");
  };

  function switchTab(id) {
    activeTab = id;

    document.querySelectorAll("[data-tab]").forEach(b => {
      const on = b.dataset.tab === id;
      b.classList.toggle("active", on);
      b.setAttribute("aria-selected", String(on));
      b.tabIndex = on ? 0 : -1;
    });

    document.querySelectorAll(".tab-content").forEach(p => {
      p.hidden = p.id !== id;
    });

    render();
  }

  document.querySelectorAll("[data-tab]").forEach(b => {
    b.onclick = () => switchTab(b.dataset.tab);
  });

  document.querySelector(".tabs").addEventListener("keydown", e => {
    if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(e.key)) return;
    e.preventDefault();

    const tabs = [...document.querySelectorAll("[data-tab]")];
    let i = tabs.findIndex(t => t.dataset.tab === activeTab);

    i = e.key === "Home" ? 0
      : e.key === "End" ? tabs.length - 1
      : (i + (e.key === "ArrowRight" ? 1 : -1) + tabs.length) %
        tabs.length;

    switchTab(tabs[i].dataset.tab);
    tabs[i].focus();
  });

  $("graphType").onchange = graph;
  $("helpButton").onclick = () => $("help").showModal();
  $("closeHelp").onclick = () => $("help").close();

  const questions = [
    {
      q: "알짜힘이 0인 물체의 운동으로 가능한 것은?",
      options: ["반드시 정지", "등속 직선 운동", "반드시 가속", "반드시 원운동"],
      answer: 1,
      why: "ΣF=ma에서 가속도는 0입니다. 정지 또는 등속 직선 운동이 가능합니다."
    },
    {
      q: "정지 마찰력에 관한 설명으로 옳은 것은?",
      options: ["항상 μN", "항상 0", "필요한 만큼 작용하되 최대 μN", "운동 방향으로만 작용"],
      answer: 2,
      why: "정지 마찰력은 접선 방향의 미끄러짐을 막는 데 필요한 만큼 작용합니다."
    },
    {
      q: "이상적인 이동 도르래를 위쪽으로 당기는 두 줄이 수직이고 각 장력이 T라면, 줄의 합력은?",
      options: ["T/2", "T", "2T", "0"],
      answer: 2,
      why: "두 장력이 같은 위쪽 방향이므로 벡터 합이 2T입니다. 줄이 기울면 방향 성분을 더해야 합니다."
    },
    {
      q: "줄이 느슨해졌을 때 줄이 전달하는 힘은?",
      options: ["미는 힘", "항상 mg", "항상 2mg", "0"],
      answer: 3,
      why: "이상적인 줄은 당기는 힘만 전달하며 느슨해지면 장력이 0입니다."
    },
    {
      q: "공기 저항 없는 포물선 운동에서 수평 외력이 없다면?",
      options: ["vx가 일정하다", "vy가 일정하다", "가속도가 0이다", "속력이 항상 일정하다"],
      answer: 0,
      why: "수평 알짜힘이 0이므로 ax=0, vx는 일정합니다. 수직으로는 ay=−g입니다."
    },
    {
      q: "질량 2 kg의 물체에 알짜힘 (6, 8) N이 작용할 때 가속도 크기는?",
      options: ["2 m/s²", "5 m/s²", "7 m/s²", "10 m/s²"],
      answer: 1,
      why: "알짜힘 크기는 √(6²+8²)=10 N입니다. 가속도 크기는 10/2=5 m/s²입니다."
    },
    {
      q: "이 앱에서 충돌 직후 역학적 에너지가 감소할 수 있는 이유는?",
      options: [
        "항상 에너지 보존 법칙이 틀려서",
        "반발계수 0의 비탄성 충돌을 사용해서",
        "중력이 없어서",
        "질량이 변해서"
      ],
      answer: 1,
      why: "비탄성 충돌에서는 일부 역학적 에너지가 내부 에너지 등으로 전환됩니다. 전체 에너지 보존과 모순되지 않습니다."
    }
  ];

  let qi = 0;
  let choice = -1;
  let submitted = false;
  const scores = new Set();

  function quiz() {
    const q = questions[qi];
    $("quizProgress").textContent =
      `QUESTION ${qi + 1}/${questions.length} · 맞힌 문제 ${scores.size}`;
    $("quizQuestion").textContent = q.q;

    $("quizChoices").innerHTML = q.options.map((o, i) => `
      <button data-choice="${i}"
        class="quiz-choice ${choice === i ? "selected" : ""}"
        aria-pressed="${choice === i}"
        ${submitted ? "disabled" : ""}>
        ${"①②③④"[i]} ${esc(o)}
      </button>
    `).join("");

    $("quizSubmit").disabled = submitted || choice < 0;
    $("quizFeedback").hidden = !submitted;

    if (submitted) {
      $("quizFeedback").textContent =
        (choice === q.answer
          ? "정답입니다. "
          : "정답은 " + "①②③④"[q.answer] + "입니다. ") +
        q.why;
    }
  }

  $("quizChoices").onclick = e => {
    const b = e.target.closest("[data-choice]");
    if (b && !submitted) {
      choice = +b.dataset.choice;
      quiz();
    }
  };

  $("quizSubmit").onclick = () => {
    if (choice < 0 || submitted) return;
    submitted = true;
    if (choice === questions[qi].answer) scores.add(qi);
    quiz();
  };

  $("quizNext").onclick = () => {
    qi = (qi + 1) % questions.length;
    choice = -1;
    submitted = false;
    quiz();
  };

  function runChecks() {
    const results = [];

    function near(a, b, tolerance = 1e-4) {
      if (!Number.isFinite(a) || Math.abs(a - b) > tolerance) {
        throw Error(fmt(a, 6) + " ≠ " + fmt(b, 6));
      }
    }

    function assert(condition, text) {
      if (!condition) throw Error(text);
    }

    function test(name, fn) {
      try {
        fn();
        results.push({ ok: true, text: "✓ " + name });
      } catch (e) {
        results.push({
          ok: false,
          text: "✕ " + name + " — " + e.message
        });
      }
    }

    const empty = () => ({
      version: 1, g: 9.8, nodes: [], ropes: []
    });

    test("초기 장면: 물체·발판·줄 자동 생성", () => {
      const s = example();
      assert(s.nodes.filter(DYNAMIC).length === 2, "초기 물체 수 오류");
      assert(s.nodes.some(SURFACE), "발판 없음");
      assert(s.ropes.length === 1, "줄 없음");
      assert(validate(s).length === 0, validate(s).join("; "));
    });

    test("초기 장면: 기본 화면 안에 물체가 있음", () => {
      const s = example();
      assert(
        s.nodes.filter(DYNAMIC).every(n =>
          n.x > 0 && n.x < 16 && n.y > 0 && n.y < 10
        ),
        "화면 밖 초기 물체"
      );
    });

    test("초기화: 위치·초기 속도·시간 복원", () => {
      const s = example("projectile");
      const a = createEngine(s);
      for (let i = 0; i < 20; i++) integrate(a);

      const b = createEngine(s);
      const p = b.dynamic[0];
      const original = s.nodes.find(DYNAMIC);
      near(b.time, 0);
      near(p.x, original.x);
      near(p.y, original.y);
      near(p.vx, original.vx);
      near(p.vy, original.vy);
    });

    test("자유낙하: ay=−g", () => {
      const s = empty();
      s.nodes = [node("body", 0, 8)];
      const e = createEngine(s);
      integrate(e);
      near(e.dynamic[0].ay, -9.8);
    });

    test("포물선: vx 일정, vy=vy₀−gt", () => {
      const s = empty();
      s.nodes = [node("body", 0, 10, { vx: 3, vy: 5 })];
      const e = createEngine(s);
      for (let i = 0; i < 120; i++) integrate(e);
      near(e.dynamic[0].vx, 3);
      near(e.dynamic[0].vy, 0.1, 1e-5);
    });

    test("수평면: 수직항력 mg", () => {
      const s = empty();
      s.nodes = [
        node("platform", 0, 0, { length: 10, mu: 0 }),
        node("body", 0, 0.35, { mu: 0 })
      ];
      const e = createEngine(s);
      integrate(e);
      const b = e.dynamic[0];

      near(b.ay, 0);
      near(
        b.forces.filter(f => f.kind === "normal")
          .reduce((a, f) => a + f.y, 0),
        19.6
      );
    });

    test("정지 마찰: 외력 3 N을 상쇄", () => {
      const s = empty();
      s.nodes = [
        node("platform", 0, 0, { length: 10, mu: 0.3 }),
        node("body", 0, 0.35, { mu: 0.3, fx: 3 })
      ];
      const e = createEngine(s);
      integrate(e);

      near(e.dynamic[0].vx, 0);
      near(
        e.dynamic[0].forces.filter(f => f.kind === "friction")
          .reduce((a, f) => a + f.x, 0),
        -3
      );
    });

    test("경사면: 마찰 없을 때 접선 가속도 −g sinθ", () => {
      const s = empty();
      const a = Math.PI / 6;

      s.nodes = [
        node("ramp", 0, 0, { length: 10, angle: 30, mu: 0 }),
        node(
          "body",
          -Math.sin(a) * 0.35,
          Math.cos(a) * 0.35,
          { mu: 0 }
        )
      ];

      const e = createEngine(s);
      integrate(e);
      const b = e.dynamic[0];
      near(b.ax * Math.cos(a) + b.ay * Math.sin(a), -4.9);
    });

    test("수평 연결: 전체 질량에 따른 가속도와 장력", () => {
      const e = createEngine(example("train"));
      integrate(e);
      near(e.dynamic[0].ax, 2.4);
      near(e.dynamic[1].ax, 2.4);
      near(e.rows.find(r => r.kind === "rope").impulse / DT, 4.8);
    });

    test("고정 도르래: Atwood 가속도·장력", () => {
      const e = createEngine(example("atwood"));
      integrate(e);
      near(e.dynamic[0].ay, 1.96);
      near(e.dynamic[1].ay, -1.96);
      near(e.rows.find(r => r.kind === "rope").impulse / DT, 23.52);
    });

    test("이동 도르래: 2:1 구속·질량 포함", () => {
      const e = createEngine(example("moving"));
      integrate(e);
      const p = e.dynamic.find(n => n.kind === "moving");
      const b = e.dynamic.find(n => n.kind === "body");
      near(p.ay, -2.45);
      near(b.ay, 4.9);
      near(2 * p.ay + b.ay, 0);
      near(e.rows.find(r => r.kind === "rope").impulse / DT, 14.7);
    });

    test("느슨한 줄: 장력 0", () => {
      const s = example("train");
      s.ropes[0].length += 2;
      const e = createEngine(s);
      integrate(e);
      near(e.rows.find(r => r.kind === "rope").impulse, 0);
    });

    test("줄은 압축력을 전달하지 않음", () => {
      const s = example("train");
      const bodies = s.nodes.filter(DYNAMIC);
      bodies[0].fx = 12;
      bodies[1].fx = 0;
      const e = createEngine(s);
      integrate(e);
      near(e.rows.find(r => r.kind === "rope").impulse, 0);
    });

    test("물체 충돌: 동일 질량·비탄성 속도", () => {
      const s = empty();
      s.g = 0;
      s.nodes = [
        node("body", 0, 2, { vx: 1, mu: 0 }),
        node("body", 0.7, 2, { vx: 0, mu: 0 })
      ];
      const e = createEngine(s);
      integrate(e);
      near(e.dynamic[0].vx, 0.5);
      near(e.dynamic[1].vx, 0.5);
    });

    test("잘못된 연결: 중간 물체 경로 차단", () => {
      const s = empty();
      s.nodes = [
        node("body", 0, 1),
        node("body", 2, 1),
        node("body", 4, 1)
      ];
      s.ropes = [rope(s.nodes, s.nodes.map(n => n.id))];
      assert(
        validate(s).some(t => t.includes("중간")),
        "지원하지 않는 경로를 허용함"
      );
    });

    test("중복 줄 구속 차단", () => {
      const s = example("train");
      const r = copy(s.ropes[0]);
      r.id = uid();
      s.ropes.push(r);
      assert(validate(s).length > 0, "중복 구속을 허용함");
    });

    test("JSON 저장·복원 구조", () => {
      const s = example("moving");
      const restored = JSON.parse(JSON.stringify(s));
      assert(validate(restored).length === 0, "복원 검증 실패");
      near(restored.ropes[0].length, s.ropes[0].length);
    });

    test("실행 화면: SVG와 초기 물체 요소 존재", () => {
      assert($("workspace") instanceof SVGElement, "SVG 없음");
      assert(
        $("workspace").querySelector("[data-id]"),
        "그려진 장면 항목 없음"
      );
      assert(
        $("workspace").getBoundingClientRect().width > 0,
        "작업 공간 너비 0"
      );
    });

    const passed = results.filter(r => r.ok).length;
    $("checkResults").textContent =
      `${passed}/${results.length} 검증 통과\n\n` +
      results.map(r => r.text).join("\n") +
      "\n\n이 검증은 실제 터치 기기 조작·모든 배치·장시간 수치 안정성의 전체 검증을 대신하지 않습니다.";
  }

  function runStudyChecks() {
    runChecks();
    const lines = [];

    const near = (a, b, tolerance = 1e-7) => {
      if (!Number.isFinite(a) || Math.abs(a - b) > tolerance) {
        throw Error(`${a} ≠ ${b}`);
      }
    };

    const test = (name, fn) => {
      try {
        fn();
        lines.push("✓ " + name);
      } catch (e) {
        lines.push("✕ " + name + " — " + e.message);
      }
    };

    test("사전 풀이: 수평 연결 가속도·장력", () => {
      const e = createEngine(example("train"));
      near(e.dynamic[0].ax, 2.4);
      near(e.solution.rows.find(r => r.kind === "rope").force, 4.8);
    });

    test("고정 도르래: 1초 동안 장력 일정·길이 보존", () => {
      const e = createEngine(example("atwood"));
      const T = e.solution.rows.find(r => r.kind === "rope").force;
      for (let i = 0; i < 240; i++) integrate(e);
      near(e.dynamic[0].y, 4.98);
      near(e.dynamic[1].y, 3.02);
      near(T, 23.52);
      near(
        geometry(e.model.ropes[0], e.map).length,
        e.model.ropes[0].length
      );
    });

    test("이동 도르래: 1초 후 2:1 변위·장력", () => {
      const e = createEngine(example("moving"));
      for (let i = 0; i < 240; i++) integrate(e);
      const p = e.dynamic.find(n => n.kind === "moving");
      const b = e.dynamic.find(n => n.kind === "body");
      near(p.y, 2.775);
      near(b.y, 5.45);
      near(e.solution.rows.find(r => r.kind === "rope").force, 14.7);
    });

    test("포물선: 정확한 등가속도 위치", () => {
      const e = createEngine(example("projectile"));
      for (let i = 0; i < 240; i++) integrate(e);
      near(e.dynamic[0].x, 8);
      near(e.dynamic[0].y, 4.1);
    });

    test("일반 모드: 정지 발판에서 반발 속도 없음", () => {
      const s = {
        version: 1,
        g: 9.8,
        nodes: [
          node("platform", 0, 0, { length: 12, mu: 0.3 }),
          node("body", 0, 0.35, { mu: 0.3 })
        ],
        ropes: []
      };
      const e = createEngine(s, false);
      for (let i = 0; i < 1200; i++) integrate(e);
      near(e.dynamic[0].y, 0.35, 1e-6);
      near(e.dynamic[0].vy, 0, 1e-6);
    });

    test("일반 모드: 도르래 줄 길이 드리프트 억제", () => {
      const e = createEngine(example("atwood"), false);
      for (let i = 0; i < 240; i++) integrate(e);
      near(
        geometry(e.model.ropes[0], e.map).length,
        e.model.ropes[0].length,
        1e-5
      );
    });

    test("진자: 가속도·장력을 고정값으로 판정하지 않음", () => {
      const a = node("anchor", 0, 5);
      const b = node("body", 2, 2);
      const s = {
        version: 1, g: 9.8, nodes: [a, b], ropes: []
      };
      s.ropes = [rope(s.nodes, [a.id, b.id])];
      if (createEngine(s).solution.constant) {
        throw Error("잘못된 일정값 판정");
      }
    });

    test("초기 충격: 유한한 순간 힘으로 표시하지 않음", () => {
      const s = example("atwood");
      s.nodes.find(DYNAMIC).vy = -1;
      if (createEngine(s).solution.reliable) {
        throw Error("초기 속도 불일치 미검출");
      }
    });

    test("충돌 경계: 일정 가속도 구간 자동 정지", () => {
      const e = createEngine(example("fall"));
      for (let i = 0; i < 1000 && !e.event; i++) integrate(e);
      if (!e.event || !e.phaseEnded) throw Error("경계 미검출");
      if (e.dynamic[0].y < 0.85 - 1e-6) {
        throw Error("바닥을 통과함");
      }
    });

    test("화면 초기화: 문제 풀이 패널·저속 옵션", () => {
      if (!$("studyPanel") || !$("revealSolution")) {
        throw Error("풀이 패널 없음");
      }
      if (!$("speed").querySelector('option[value=".1"]')) {
        throw Error("0.1배속 없음");
      }
    });

    $("checkResults").textContent +=
      "\n\n개선 기능 검증\n" + lines.join("\n") +
      "\n\n실제 갤럭시 탭 터치·브라우저 시각 검증은 별도로 필요합니다.";
  }

  $("runChecks").onclick = runStudyChecks;

  function frame(now) {
    if (running) {
      if (lastTime) {
        accumulated += Math.min(
          (now - lastTime) / 1000, 0.05
        ) * Number($("speed").value);

        while (accumulated >= DT && running) {
          integrate(engine);
          accumulated -= DT;
          if (!checkRuntime()) break;
        }
      }

      lastTime = now;

      if (now - renderAt > 33) {
        draw();
        metric();
        renderAt = now;
      }

      if (now - learningAt > 160) {
        if (activeTab === "analysis") analysis();
        if (activeTab === "graphs") graph();
        renderStudy();
        learningAt = now;
      }

      if (uiDirty) {
        inspector();
        sceneList();
        uiDirty = false;
      }
    } else {
      lastTime = 0;
    }

    requestAnimationFrame(frame);
  }

  document.addEventListener("visibilitychange", () => {
    if (document.hidden && running) {
      pause("탭을 벗어나 자동 일시정지");
      render(true);
    }
  });

  installStudyUI();
  reset();
  fitView();
  setTool("select");
  switchTab("analysis");
  quiz();
  render(true);
  message(
    "초기 실험이 준비되었습니다. 위에서 정답을 예측하고 ‘계산 결과와 풀이 확인’을 눌러 보세요."
  );
  requestAnimationFrame(frame);
})();
