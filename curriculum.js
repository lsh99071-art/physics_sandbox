"use strict";

function SchoolPhysicsCore() {
  const EPS = 1e-5;
  const dynamic = n => n.kind === "body" || n.kind === "moving";
  const surface = n => n.kind === "platform" || n.kind === "ramp";
  const dot = (a, b) => a.x * b.x + a.y * b.y;
  const norm = v => Math.hypot(v.x, v.y);
  const vec = (x, y) => ({ x, y });
  const scale = (v, k) => vec(v.x * k, v.y * k);
  const fail = reason => ({
    ok: false,
    title: "현재 학습 범위를 벗어난 문제",
    reason
  });

  const axes = p => {
    const a = p.angle * Math.PI / 180;
    return {
      t: { x: Math.cos(a), y: Math.sin(a) },
      n: { x: -Math.sin(a), y: Math.cos(a) }
    };
  };

  function supportFor(n, s) {
    for (const p of s.nodes.filter(surface)) {
      const u = axes(p);
      const r = vec(n.x - p.x, n.y - p.y);
      const along = Math.abs(dot(r, u.t));

      if (Math.abs(dot(r, u.n) - n.radius) < EPS &&
          Math.abs(along - p.length / 2) < EPS) {
        throw Error(
          "물체가 면의 끝에 있는 경계 상황입니다. 초기 위치를 면 안쪽으로 옮기거나 다음 운동 구간을 따로 구성하세요."
        );
      }
    }

    const found = s.nodes.filter(surface).filter(p => {
      const u = axes(p);
      const r = vec(n.x - p.x, n.y - p.y);
      return Math.abs(dot(r, u.n) - n.radius) < EPS &&
        Math.abs(dot(r, u.t)) < p.length / 2 - EPS;
    });

    if (found.length > 1) {
      throw Error(
        "한 물체가 여러 면과 동시에 접촉하는 구조는 검증된 풀이 유형에 포함되지 않습니다."
      );
    }

    return found[0] || null;
  }

  function routeGeometry(r, s) {
    const map = new Map(s.nodes.map(n => [n.id, n]));
    let length = 0;

    for (let i = 1; i < r.path.length; i++) {
      const a = map.get(r.path[i - 1]);
      const b = map.get(r.path[i]);

      if (!a || !b) throw Error("줄의 연결점을 확인할 수 없습니다.");

      const d = Math.hypot(b.x - a.x, b.y - a.y);
      if (d < 0.05) throw Error("줄의 연결점이 너무 가깝습니다.");
      length += d;
    }

    if (Math.abs(length - r.length) > EPS) {
      throw Error(
        "팽팽한 줄의 현재 경로와 설정 길이가 같아야 합니다. 줄의 자동 길이를 누르세요."
      );
    }

    return map;
  }

  function bodyData(n, s, u, p) {
    const z = {
      n,
      u,
      p,
      m: n.m,
      v: dot(vec(n.vx, n.vy), u),
      N: 0,
      cap: 0,
      f: 0,
      P: dot(vec(n.fx, n.fy - n.m * s.g), u),
      a: 0,
      forces: [
        {
          name: "중력 mg",
          x: 0,
          y: -n.m * s.g,
          why: "지구가 물체를 아래로 당기는 힘입니다."
        },
        {
          name: "외력 F",
          x: n.fx,
          y: n.fy,
          why: "사용자가 지정한 힘입니다."
        }
      ],
      ropes: []
    };

    if (Math.abs(n.vx * u.y - n.vy * u.x) > EPS) {
      throw Error("초기 속도가 정해진 직선 경로와 나란하지 않습니다.");
    }

    if (p) {
      const a = axes(p);

      if (Math.abs(dot(a.n, u)) > EPS) {
        throw Error("줄 방향과 접촉면의 운동 방향이 일치하지 않습니다.");
      }

      z.N = n.m * s.g * a.n.y - dot(vec(n.fx, n.fy), a.n);

      if (z.N <= EPS) {
        throw Error(
          "물체가 면에서 떨어지는 경계입니다. 접촉을 유지하는 문제로 풀 수 없습니다."
        );
      }

      z.cap = Math.min(n.mu, p.mu) * z.N;
      z.normalAxis = a.n;
      z.forces.push({
        name: "수직항력 N",
        ...scale(a.n, z.N),
        why: "면이 물체를 수직으로 밀어 접촉을 유지하는 힘입니다."
      });
    } else {
      const free = vec(n.fx, n.fy - n.m * s.g);

      if (Math.abs(free.x * u.y - free.y * u.x) > EPS) {
        throw Error(
          "매달린 물체에 경로와 수직인 힘이 작용하여 줄 방향이 바뀔 수 있습니다."
        );
      }
    }

    return z;
  }

  function motionOf(rows) {
    if (rows.some(r => Math.abs(r.v - rows[0].v) > EPS)) {
      throw Error(
        "팽팽한 줄로 연결된 물체의 초기 경로 속도가 서로 다릅니다. 충격이 먼저 발생하므로 등가속도 문제로 풀지 않습니다."
      );
    }

    const drive = rows.reduce((a, r) => a + r.P, 0);
    const cap = rows.reduce((a, r) => a + r.cap, 0);
    const v = rows[0].v;
    const m = rows.reduce((a, r) => a + r.m, 0);

    let a;
    let staticState = false;

    if (Math.abs(v) > EPS) {
      rows.forEach(r => {
        r.f = -Math.sign(v) * r.cap;
      });
      a = (drive + rows.reduce((x, r) => x + r.f, 0)) / m;
    } else if (Math.abs(drive) <= cap + EPS) {
      const frictionRows = rows.filter(r => r.cap > EPS);

      if (frictionRows.length > 1) {
        throw Error(
          "여러 정지 마찰력이 힘을 나눠 받습니다. 장력을 하나로 정하려면 추가 조건이 필요하므로 임의의 정답을 표시하지 않습니다."
        );
      }

      if (frictionRows.length === 1) {
        frictionRows[0].f = -drive;
      }

      a = 0;
      staticState = true;
    } else {
      rows.forEach(r => {
        r.f = -Math.sign(drive) * r.cap;
      });
      a = (drive + rows.reduce((x, r) => x + r.f, 0)) / m;
    }

    rows.forEach(r => {
      r.a = a;
      r.drive = r.P + r.f;

      if (r.p) {
        r.forces.push({
          name: "마찰력 f",
          ...scale(r.u, r.f),
          why: staticState
            ? "정지 마찰은 미끄러짐을 막는 데 필요한 만큼만 작용합니다."
            : "미끄러짐의 반대 방향으로 작용합니다."
        });
      }
    });

    return {
      a,
      totalMass: m,
      totalDrive: rows.reduce((x, r) => x + r.drive, 0),
      staticState
    };
  }

  function finish(result, s) {
    for (const row of result.rows) {
      const sum = row.forces.reduce(
        (v, force) => vec(v.x + force.x, v.y + force.y),
        vec(0, 0)
      );

      row.fx = sum.x;
      row.fy = sum.y;

      if (result.type !== "free") {
        row.ax = row.u.x * row.a;
        row.ay = row.u.y * row.a;

        if (Math.abs(sum.x - row.m * row.ax) > 1e-4 ||
            Math.abs(sum.y - row.m * row.ay) > 1e-4) {
          return fail(
            "이 배치의 힘과 운동 방향을 기본 유형의 식으로 일치시킬 수 없습니다."
          );
        }
      }

      row.maxT = Math.max(0, ...row.ropes.map(r => r.T));
    }

    return { ok: true, scene: s, ...result };
  }

  function solve(s) {
    try {
      if (!s || !Array.isArray(s.nodes) ||
          !Array.isArray(s.ropes) ||
          !Number.isFinite(s.g) || s.g < 0) {
        return fail("입력한 장면 정보를 확인하세요.");
      }

      const bodies = s.nodes.filter(dynamic);

      if (!bodies.length) return fail("물체를 한 개 이상 추가하세요.");

      if (bodies.length > 4) {
        return fail("이 도구의 검증된 풀이 범위는 움직이는 물체 최대 4개입니다.");
      }

      if (bodies.some(n => n.kind === "moving")) {
        return fail(
          "이동 도르래의 일반 연결망은 현재 검증된 풀이 유형이 아닙니다. 배치·운동 관찰은 유지되지만 고정 정답을 제시하지 않습니다."
        );
      }

      for (const n of bodies) {
        if (!(n.m > 0) ||
            ![
              n.x, n.y, n.m, n.radius, n.fx, n.fy,
              n.vx, n.vy, n.mu
            ].every(Number.isFinite)) {
          return fail("물체의 수치를 확인하세요.");
        }

        for (const p of s.nodes.filter(surface)) {
          const u = axes(p);
          const r = vec(n.x - p.x, n.y - p.y);
          const d = dot(r, u.n);

          if (Math.abs(dot(r, u.t)) <= p.length / 2 &&
              d >= 0 && d < n.radius - EPS) {
            return fail("물체가 발판을 파고듭니다. 먼저 초기 배치를 맞추세요.");
          }
        }
      }

      for (let i = 0; i < bodies.length; i++) {
        for (let j = i + 1; j < bodies.length; j++) {
          if (Math.hypot(
            bodies[i].x - bodies[j].x,
            bodies[i].y - bodies[j].y
          ) < bodies[i].radius + bodies[j].radius - EPS) {
            return fail(
              "물체가 겹치거나 충돌하는 초기 상황은 이 등가속도 풀이에 포함하지 않습니다."
            );
          }
        }
      }

      const supports = new Map(
        bodies.map(n => [n.id, supportFor(n, s)])
      );

      s.ropes.forEach(r => routeGeometry(r, s));

      if (!s.ropes.length) {
        if (bodies.every(n => !supports.get(n.id))) {
          const rows = bodies.map(n => ({
            n,
            m: n.m,
            p: null,
            u: vec(1, 0),
            N: 0,
            cap: 0,
            f: 0,
            ropes: [],
            ax: n.fx / n.m,
            ay: n.fy / n.m - s.g,
            forces: [
              {
                name: "중력 mg",
                x: 0,
                y: -n.m * s.g,
                why: "중력은 아래 방향입니다."
              },
              {
                name: "외력 F",
                x: n.fx,
                y: n.fy,
                why: "설정한 외력입니다."
              }
            ]
          }));

          return finish({
            type: "free",
            rows,
            tensions: [],
            constant: true
          }, s);
        }

        if (bodies.length !== 1) {
          return fail(
            "연결되지 않은 여러 접촉 물체는 각각 따로 문제를 만들어 확인하세요."
          );
        }

        const n = bodies[0];
        const p = supports.get(n.id);
        const u = axes(p).t;
        const row = bodyData(n, s, u, p);
        const motion = motionOf([row]);

        return finish({
          type: "single",
          rows: [row],
          tensions: [],
          ...motion
        }, s);
      }

      const ordered = bodies.slice().sort((a, b) => a.x - b.x);
      const horizontal = bodies.every(n =>
        supports.get(n.id) &&
        Math.abs(supports.get(n.id).angle) < EPS
      );
      const direct = s.ropes.every(r =>
        r.path.length === 2 &&
        r.path.every(id => bodies.some(n => n.id === id))
      );

      if (horizontal && direct &&
          s.ropes.length === bodies.length - 1) {
        const used = new Set();
        const links = [];

        for (let i = 0; i < ordered.length - 1; i++) {
          const a = ordered[i];
          const b = ordered[i + 1];

          if (Math.abs(a.y - b.y) > EPS) {
            return fail("수평 연결 물체의 줄 연결 높이를 같게 맞추세요.");
          }

          const r = s.ropes.find(r =>
            r.path.includes(a.id) &&
            r.path.includes(b.id) &&
            !used.has(r.id)
          );

          if (!r) {
            return fail(
              "수평 물체는 왼쪽부터 이웃한 물체끼리 한 줄씩 연결하세요."
            );
          }

          used.add(r.id);
          links.push(r);
        }

        const rows = ordered.map(n =>
          bodyData(n, s, vec(1, 0), supports.get(n.id))
        );
        const motion = motionOf(rows);
        const tensions = [];
        let mass = 0;
        let drive = 0;

        for (let i = 0; i < rows.length - 1; i++) {
          mass += rows[i].m;
          drive += rows[i].drive;
          const T = mass * motion.a - drive;

          if (T < -EPS) {
            return fail(
              "계산하려는 방향에서는 줄이 물체를 밀어야 합니다. 실제로는 줄이 느슨해져 공통 가속도 식을 적용할 수 없습니다."
            );
          }

          tensions.push({
            id: links[i].id,
            name: links[i].name,
            T: Math.max(0, T)
          });
        }

        rows.forEach((row, i) => {
          if (i > 0) {
            const t = tensions[i - 1];
            row.ropes.push(t);
            row.forces.push({
              name: t.name + " 장력 T",
              x: -t.T,
              y: 0,
              why: "왼쪽 줄이 물체를 왼쪽으로 당깁니다."
            });
          }

          if (i < tensions.length) {
            const t = tensions[i];
            row.ropes.push(t);
            row.forces.push({
              name: t.name + " 장력 T",
              x: t.T,
              y: 0,
              why: "오른쪽 줄이 물체를 오른쪽으로 당깁니다."
            });
          }
        });

        return finish({
          type: "train",
          rows,
          tensions,
          ...motion
        }, s);
      }

      if (bodies.length === 2 && s.ropes.length === 1) {
        const r = s.ropes[0];
        const map = routeGeometry(r, s);
        const a = map.get(r.path[0]);
        const b = map.get(r.path.at(-1));

        if (!a || !b || a.kind !== "body" || b.kind !== "body") {
          return fail("한 줄의 양 끝에 각각 물체를 연결해야 합니다.");
        }

        if (!r.path.slice(1, -1).every(id =>
          map.get(id).kind === "fixed"
        )) {
          return fail("이 유형의 줄 중간에는 고정 도르래만 사용할 수 있습니다.");
        }

        if (r.path.length < 3) {
          return fail("매달린 두 물체는 고정 도르래를 거쳐 연결하세요.");
        }

        const next = map.get(r.path[1]);
        const prev = map.get(r.path.at(-2));
        const d1 = vec(next.x - a.x, next.y - a.y);
        const d2 = vec(b.x - prev.x, b.y - prev.y);
        const ua = scale(d1, 1 / norm(d1));
        const ub = scale(d2, 1 / norm(d2));
        const pa = supports.get(a.id);
        const pb = supports.get(b.id);

        if (pa && pb) {
          return fail(
            "서로 다른 두 접촉면을 거친 도르래 연결은 현재 검증된 풀이 유형이 아닙니다."
          );
        }

        if ((!pa && Math.abs(ua.x) > EPS) ||
            (!pb && Math.abs(ub.x) > EPS)) {
          return fail(
            "매달린 물체의 줄은 수직이어야 합니다. 비스듬한 진자 운동의 값을 고정 정답으로 표시하지 않습니다."
          );
        }

        const rows = [
          bodyData(a, s, ua, pa),
          bodyData(b, s, ub, pb)
        ];
        const motion = motionOf(rows);
        const T = rows[1].drive - rows[1].m * motion.a;

        if (T < -EPS) {
          return fail(
            "이 초기 조건에서는 줄이 느슨해져 같은 경로 가속도를 사용할 수 없습니다."
          );
        }

        const tension = {
          id: r.id,
          name: r.name,
          T: Math.max(0, T)
        };

        rows[0].ropes.push(tension);
        rows[1].ropes.push(tension);

        rows[0].forces.push({
          name: "장력 T",
          ...scale(ua, tension.T),
          why: "줄이 첫 물체를 +방향으로 당깁니다."
        });
        rows[1].forces.push({
          name: "장력 T",
          ...scale(ub, -tension.T),
          why: "줄이 둘째 물체를 선택한 경로의 −방향으로 당깁니다."
        });

        return finish({
          type: "pulley",
          rows,
          tensions: [tension],
          ...motion
        }, s);
      }

      return fail(
        "여러 줄·도르래의 일반 연결망, 줄 방향이 바뀌는 운동 등은 현재 검증된 교과서 풀이 유형이 아닙니다."
      );
    } catch (e) {
      return fail(e.message);
    }
  }

  function sample(kind) {
    let serial = 0;

    const n = (kind, x, y, extra = {}) => ({
      id: "s" + (++serial),
      kind,
      name: kind,
      x,
      y,
      m: 2,
      radius: 0.35,
      mu: 0,
      fx: 0,
      fy: 0,
      vx: 0,
      vy: 0,
      length: 10,
      angle: 0,
      ...extra
    });

    const s = { version: 1, g: 9.8, nodes: [], ropes: [] };
    const add = (...args) => {
      const v = n(...args);
      s.nodes.push(v);
      return v;
    };
    const link = (...nodes) => {
      let length = 0;

      for (let i = 1; i < nodes.length; i++) {
        length += Math.hypot(
          nodes[i].x - nodes[i - 1].x,
          nodes[i].y - nodes[i - 1].y
        );
      }

      s.ropes.push({
        id: "r" + s.ropes.length,
        kind: "rope",
        name: "줄 " + (s.ropes.length + 1),
        path: nodes.map(n => n.id),
        length
      });
    };

    if (kind === "atwood") {
      const a = add("body", 5, 4, { name: "A", m: 2 });
      const p = add("fixed", 5, 8, { name: "P1" });
      const q = add("fixed", 11, 8, { name: "P2" });
      const b = add("body", 11, 4, { name: "B", m: 3 });
      link(a, p, q, b);
    } else if (kind === "incline") {
      const t = Math.PI / 6;
      add("ramp", 8, 3, {
        name: "경사면", angle: 30, length: 10, mu: 0.2
      });
      add(
        "body",
        8 - Math.sin(t) * 0.35,
        3 + Math.cos(t) * 0.35,
        { name: "A", m: 2, mu: 0.2 }
      );
    } else if (kind === "slopePulley") {
      const t = Math.PI / 6;
      add("ramp", 6, 3, {
        name: "경사면", angle: 30, length: 6
      });
      const a = add(
        "body",
        6 - 0.35 * Math.sin(t),
        3 + 0.35 * Math.cos(t),
        { name: "A", m: 2 }
      );
      const p = add(
        "fixed",
        a.x + 3 * Math.cos(t),
        a.y + 3 * Math.sin(t),
        { name: "P1" }
      );
      const q = add("fixed", 12, p.y, { name: "P2" });
      const b = add("body", 12, p.y - 2.5, { name: "B", m: 3 });
      link(a, p, q, b);
    } else if (kind === "fall") {
      add("body", 8, 8, { name: "A", m: 2 });
      add("platform", 8, 0.5, { name: "바닥", length: 14 });
    } else {
      const friction = kind === "friction";
      add("platform", 8, 1, {
        name: "발판", length: 14, mu: friction ? 0.2 : 0
      });
      const a = add("body", 4, 1.35, {
        name: "A", m: 2, mu: friction ? 0.2 : 0
      });
      const b = add("body", 7, 1.35, {
        name: "B",
        m: 3,
        mu: friction ? 0.2 : 0,
        fx: friction ? 18 : 12
      });

      if (kind === "three") {
        b.fx = 0;
        const c = add("body", 10, 1.35, {
          name: "C", m: 1, fx: 12
        });
        link(a, b);
        link(b, c);
      } else {
        link(a, b);
      }
    }

    return s;
  }

  return {
    solve, sample, axes, dot, scale, dynamic, surface
  };
}

function SchoolLessons(C) {
  const f = (x, n = 3) =>
    (Math.abs(x) < 0.5 * 10 ** -n ? 0 : x).toFixed(n);

  const esc = value => String(value).replace(/[&<>"']/g, c =>
    ({
      "&": "&amp;",
      "<": "&lt;",
      ">": "&gt;",
      '"': "&quot;",
      "'": "&#39;"
    }[c])
  );

  const sg = x => (x > 1e-8 ? "+" : "") + f(x);

  const dir = u => {
    if (Math.abs(u.y) < 1e-6) return u.x >= 0 ? "오른쪽" : "왼쪽";
    if (Math.abs(u.x) < 1e-6) return u.y >= 0 ? "위쪽" : "아래쪽";
    return (u.y > 0 ? "위" : "아래") +
      "·" + (u.x > 0 ? "오른" : "왼") + "쪽";
  };

  function card(
    name, principle, symbols, why,
    formula, numbers, arrange, answer
  ) {
    return `
      <article class="sf-card">
        <h4>${esc(name)}</h4>
        <p><b>원리:</b> ${principle}</p>
        <p><b>문자·단위:</b> ${symbols}</p>
        <p><b>이 문제에 쓰는 이유:</b> ${why}</p>
        <div class="sf-equation">${formula}</div>
        <p><b>수치 대입:</b></p>
        <div class="sf-equation">${numbers}</div>
        <p><b>정리:</b> ${arrange}</p>
        <p><b>답:</b> ${answer}</p>
      </article>
    `;
  }

  function ropeTerms(r, i) {
    if (r.type === "pulley") return i === 0 ? " + T₁" : " − T₁";
    if (r.type === "train") {
      return (i > 0 ? " − T" + i : "") +
        (i < r.tensions.length ? " + T" + (i + 1) : "");
    }
    return "";
  }

  function equation(r, row, i, numeric) {
    const m = numeric ? f(row.m) : "m" + (i + 1);
    const base = numeric
      ? `(${f(row.P)}) + (${f(row.f)})`
      : "외력의 경로성분 + 중력의 경로성분 + 마찰의 경로성분";

    return `${esc(row.n.name)}: ${m}a = ${base}${ropeTerms(r, i)}`;
  }

  function normalWork(r, row) {
    if (!row.p) return "";

    const theta = Math.abs(row.p.angle);
    const normal = C.axes(row.p).n;
    const ext = row.n.fx * normal.x + row.n.fy * normal.y;

    const normalText = card(
      "접촉면에 수직인 방향의 힘의 평형",
      "물체가 면을 뚫거나 면에서 떠오르지 않는 구간에는 이 방향의 가속도가 0입니다.",
      "N: 수직항력(N), m: 질량(kg), g: 중력가속도(m/s²), θ: 경사각(°), F⊥: 외력의 면 수직 성분(N).",
      `${esc(row.n.name)}의 마찰력을 구하려면 먼저 면이 미는 힘 N을 알아야 합니다.`,
      "N + F⊥ − mg cosθ = 0",
      `N + (${f(ext)}) − ${f(row.m)} × ${f(r.scene.g)}
       × cos ${f(theta, 1)}° = 0`,
      `중력이 면을 누르는 크기는
       ${f(row.m * r.scene.g * Math.cos(theta * Math.PI / 180))} N입니다.
       −mg cosθ를 반대편으로 옮기고 F⊥를 빼면
       N = mg cosθ − F⊥입니다.`,
      `N = ${f(row.N)} N. 수직항력은 면에서 바깥쪽으로 작용합니다.`
    );

    let friction = "";

    if (row.cap > 1e-9) {
      const mu = Math.min(row.n.mu, row.p.mu);

      friction = card(
        r.staticState ? "정지 마찰력의 한계" : "운동 마찰력",
        r.staticState
          ? "정지 마찰력은 미끄러짐을 막는 데 필요한 만큼 작용하며 한계를 넘지 못합니다."
          : "미끄러지는 물체의 운동을 방해하는 힘의 크기를 μN으로 나타냅니다.",
        "f: 마찰력(N), μ: 마찰계수(단위 없음), N: 수직항력(N). 이 모형은 정지·운동 마찰계수를 같은 값으로 둡니다.",
        `${esc(row.n.name)}의 경로 방향 알짜힘에 마찰을 포함해야 합니다.`,
        r.staticState ? "|f| ≤ μN" : "|f| = μN",
        `μN = ${f(mu)} × ${f(row.N)} = ${f(row.cap)} N`,
        r.staticState
          ? `평형에 필요한 마찰의 경로 성분은 ${sg(row.f)} N입니다.
             필요한 크기 ${f(Math.abs(row.f))} N이
             한계 ${f(row.cap)} N 이내인지 비교합니다.`
          : `크기 ${f(row.cap)} N을 구한 뒤 미끄럼 반대 방향의
             부호를 붙입니다. 초기 속도가 0이면 미끄러지기
             시작하는 방향을 기준으로 합니다.`,
        `경로의 +방향을 ${dir(row.u)}으로 정했을 때
         f = ${sg(row.f)} N.`
      );
    }

    return normalText + friction;
  }

  function mathSteps(r) {
    if (r.type === "free") {
      return r.rows.map(row => card(
        "뉴턴 제2법칙: 수평과 수직을 나누어 계산",
        "알짜힘은 질량과 가속도의 곱입니다. 서로 수직인 두 방향의 운동을 각각 계산합니다.",
        "Fₓ, Fᵧ: 외력 성분(N), m: 질량(kg), aₓ, aᵧ: 가속도 성분(m/s²), g: 중력가속도(m/s²).",
        "접촉면과 줄이 없는 초기 상태에서 중력과 외력이 속도를 어떻게 바꾸는지 구합니다.",
        "maₓ = Fₓ<br>maᵧ = Fᵧ − mg",
        `${f(row.m)}aₓ = ${f(row.n.fx)}<br>
         ${f(row.m)}aᵧ = ${f(row.n.fy)}
         − ${f(row.m)} × ${f(r.scene.g)}`,
        `두 식을 각각 질량 ${f(row.m)} kg으로 나눕니다.
         aₓ = ${f(row.n.fx)} / ${f(row.m)},
         aᵧ = (${f(row.n.fy)} − ${f(row.m * r.scene.g)})
         / ${f(row.m)}.`,
        `${esc(row.n.name)}:
         aₓ = ${sg(row.ax)} m/s²,
         aᵧ = ${sg(row.ay)} m/s².`
      )).join("") + fallEnergy(r);
    }

    const equations = r.rows.map((row, i) =>
      `<div class="sf-equation">${equation(r, row, i, true)}</div>`
    ).join("");

    const explain = r.rows.length > 1
      ? `
        <p>모든 식을 더합니다. 같은 장력 문자의 +항과 −항이
        더해져 없어집니다.
        ${r.type === "pulley"
          ? "도르래 양쪽 물체는 서로 다른 +방향을 정했습니다. 여기서는 경로 방향으로 쓴 두 식의 장력 항이 소거되는 것입니다."
          : "수평으로 연결된 계를 합쳐 보면 줄의 내부 힘이 합쳐진 식에서 없어집니다."}
        각 물체를 따로 볼 때에는 장력이 계속 작용합니다.</p>
      `
      : "<p>연결된 다른 물체가 없으므로 장력 미지수 없이 이 식 하나를 정리합니다.</p>";

    const total = r.rows.map(row => `(${f(row.m)})`).join(" + ");
    const forces = r.rows.map(row => `(${f(row.drive)})`).join(" + ");
    let substitute = "";

    if (r.type === "pulley") {
      const row = r.rows[0];
      const T = r.tensions[0].T;

      substitute = `
        <p>첫 물체의 식에서 T₁만 남깁니다.</p>
        <div class="sf-equation">
          ${f(row.m)}a = (${f(row.drive)}) + T₁<br>
          T₁ = ${f(row.m)}a − (${f(row.drive)})<br>
          T₁ = ${f(row.m)} × (${f(r.a)})
          − (${f(row.drive)}) = ${f(T)} N
        </div>
        <p>둘째 물체의 식에도 같은 a와 T₁을 넣으면
        맞는지 확인할 수 있습니다.</p>
      `;
    }

    if (r.type === "train") {
      r.tensions.forEach((t, i) => {
        const row = r.rows[i];
        const previous = i > 0 ? r.tensions[i - 1].T : 0;

        substitute += `
          <p>${esc(row.n.name)}의 식에 이미 구한 a
          ${i > 0 ? "와 왼쪽 줄 장력" : ""}를 넣고
          오른쪽 줄의 T${i + 1}을 남깁니다.</p>
          <div class="sf-equation">
            ${f(row.m)} × (${f(r.a)}) =
            (${f(row.drive)})
            ${i > 0 ? "− (" + f(previous) + ")" : ""}
            + T${i + 1}<br>
            T${i + 1} =
            ${f(row.m)} × (${f(r.a)}) − (${f(row.drive)})
            ${i > 0 ? "+ (" + f(previous) + ")" : ""}
            = ${f(t.T)} N
          </div>
        `;
      });
    }

    return r.rows.map(row => normalWork(r, row)).join("") +
      equations + explain +
      card(
        "뉴턴 제2법칙과 연립식의 덧셈",
        "함께 움직이는 물체의 경로 가속도를 공통 미지수 a로 놓고, 물체마다 세운 식을 더해 a를 구합니다.",
        "m: 질량(kg), a: 선택한 경로의 가속도(m/s²), F합: 그 경로 방향 힘을 더한 값(N), T: 장력(N).",
        r.rows.length > 1
          ? "여러 운동 방정식에 함께 등장하는 장력을 먼저 없애면 a 하나만 남기 때문입니다."
          : "경로 방향 알짜힘과 질량이 주어져 있으므로 a를 직접 구할 수 있습니다.",
        "F합 = ma",
        `(${total})a = ${forces}
         ${r.tensions.length ? "+ (장력 +항과 −항의 합 0)" : ""}`,
        `${f(r.totalMass)}a = ${f(r.totalDrive)}이므로 양변을
         ${f(r.totalMass)}으로 나눕니다.
         a = ${f(r.totalDrive)} / ${f(r.totalMass)}.`,
        `a = ${sg(r.a)} m/s².
         부호는 정한 경로의 +방향을 기준으로 합니다.`
      ) + substitute;
  }

  function fallEnergy(r) {
    if (r.rows.length !== 1) return "";

    const row = r.rows[0];
    const n = row.n;

    if (r.scene.g <= 0 ||
        Math.hypot(n.fx, n.fy, n.vx, n.vy) > 1e-9) return "";

    const floor = r.scene.nodes.filter(C.surface).filter(p =>
      Math.abs(p.angle) < 1e-9 &&
      Math.abs(n.x - p.x) < p.length / 2 &&
      p.y + n.radius < n.y
    ).sort((a, b) => b.y - a.y)[0];

    if (!floor) return "";

    const h = n.y - floor.y - n.radius;
    const m = n.m;
    const g = r.scene.g;
    const v = Math.sqrt(2 * g * h);

    return card(
      "역학적 에너지 보존으로 낙하 속력 확인",
      "마찰과 공기 저항이 없고 중력만 일을 하면 운동에너지와 중력 위치에너지의 합이 일정합니다.",
      "K=½mv²: 운동에너지(J), U=mgh: 위치에너지(J), m: kg, v: m/s, g: m/s², h: 기준 높이와의 차(m).",
      "처음 정지한 물체가 바닥에 닿기 직전의 속력을 다른 방법으로 확인합니다. 충돌 후까지 에너지가 보존된다고 가정하지 않습니다.",
      "K처음 + U처음 = K나중 + U나중<br>0 + mgh = ½mv² + 0",
      `h = ${f(n.y)} − (${f(floor.y)} + ${f(n.radius)})
       = ${f(h)} m<br>
       ${f(m)} × ${f(g)} × ${f(h)} = ½ × ${f(m)} × v²`,
      `양변을 질량 ${f(m)}으로 나누면 ${f(g * h)} = ½v²입니다.
       양변에 2를 곱해 v²=${f(2 * g * h)}을 얻고,
       속력은 음수가 아니므로 양의 제곱근을 구합니다.`,
      `바닥에 닿기 직전 속력 v=${f(v)} m/s.
       처음 위치에너지는 ${f(m * g * h)} J이고
       직전 운동에너지도 ${f(m * g * h)} J입니다.`
    );
  }

  function diagram(row) {
    const colors = [
      "#d79142", "#9b79cc", "#467eaf", "#bd607a", "#278f5b"
    ];
    let arrows = "";

    row.forces.filter(q => Math.hypot(q.x, q.y) > 1e-8)
      .forEach((q, i) => {
        const mag = Math.hypot(q.x, q.y);
        const length = Math.min(78, 30 + mag * 1.5);
        const ux = q.x / mag;
        const uy = -q.y / mag;
        const x = 180 + ux * length;
        const y = 135 + uy * length;
        const tx = x - ux * 8;
        const ty = y - uy * 8;
        const color = colors[i % colors.length];

        arrows += `
          <g>
            <path d="M180 135L${x} ${y}"
              stroke="${color}" stroke-width="3"/>
            <path d="M${x} ${y}
              L${tx - uy * 4} ${ty + ux * 4}
              L${tx + uy * 4} ${ty - ux * 4}Z"
              fill="${color}"/>
            <text x="${x + ux * 10}" y="${y + uy * 10 + 4}"
              text-anchor="${ux > .3 ? "start" : ux < -.3 ? "end" : "middle"}"
              font-size="12" fill="${color}"
              stroke="#f5f8ef" paint-order="stroke" stroke-width="3">
              ${esc(q.name)}
            </text>
          </g>
        `;
      });

    return `
      <svg viewBox="0 0 360 270" role="img"
        aria-label="${esc(row.n.name)} 자유물체도">
        <circle cx="180" cy="135" r="19" fill="#aaddb0"/>
        <text x="180" y="140" text-anchor="middle">
          ${esc(row.n.name)}
        </text>
        ${arrows}
      </svg>
    `;
  }

  function steps(r) {
    if (!r.ok) {
      return [{
        title: "학습 범위 확인",
        html: `
          <h3>현재 학습 범위를 벗어난 문제</h3>
          <p>${esc(r.reason)}</p>
          <p>이 안내는 이 도구가 기본 공식만으로 정확성을 검증한
          풀이 유형의 범위를 뜻합니다. 해당 현상이 2022 개정
          교육과정 전체에서 제외된다는 뜻은 아닙니다.
          배치와 운동 관찰은 계속 사용할 수 있습니다.</p>
        `
      }];
    }

    const given = r.rows.map(row => `
      <li><b>${esc(row.n.name)}</b>:
      m=${f(row.m)} kg,
      외력 (${sg(row.n.fx)}, ${sg(row.n.fy)}) N,
      초기 속도 (${sg(row.n.vx)}, ${sg(row.n.vy)}) m/s
      ${row.p
        ? `, 접촉면 각도 ${f(row.p.angle, 1)}°,
           적용 μ=${f(Math.min(row.n.mu, row.p.mu))}`
        : ", 접촉면 없음"}.</li>
    `).join("");

    const direction = r.rows.map(row => {
      const a = r.type === "free"
        ? { x: row.ax, y: row.ay }
        : C.scale(row.u, row.a);

      return `
        <p><b>${esc(row.n.name)}</b>:
        ${r.type === "free"
          ? "오른쪽 +x, 위쪽 +y"
          : dir(row.u) + "을 경로의 +방향"}으로 정합니다.
        ${Math.hypot(row.n.vx, row.n.vy) > 1e-8
          ? "처음에는 " + dir({ x: row.n.vx, y: row.n.vy }) + "으로 움직입니다."
          : "처음 속도는 0입니다."}
        알짜힘은
        ${Math.hypot(a.x, a.y) < 1e-8
          ? "균형을 이루므로 가속도가 0입니다."
          : dir(a) + " 방향이므로 속도가 그 방향으로 변합니다."}
        속도 방향과 가속도 방향이 반대라면 처음에는 느려집니다.</p>
      `;
    }).join("");

    const relation = r.type === "pulley"
      ? `
        <p>팽팽하고 늘어나지 않는 줄의 한 구간이 늘어나면
        다른 구간은 같은 길이만큼 줄어야 합니다.
        고정 도르래는 방향만 바꾸므로 두 물체는 각자 정한 경로에서
        같은 크기의 변위·속도·가속도를 갖습니다.
        전역 좌표에서는 한쪽은 위, 다른 쪽은 아래일 수 있습니다.</p>
      `
      : r.type === "train"
        ? `
          <p>팽팽한 줄이 늘어나지 않으므로 이웃한 물체 사이의
          간격이 유지됩니다. 이 조건에서 경로의 변위·속도·가속도가
          같습니다. 줄이 느슨해지면 이 설명을 적용할 수 없습니다.</p>
        `
        : "";

    const newton = r.rows.map(row =>
      card(
        "뉴턴 제2법칙",
        "알짜힘은 한 물체에 작용하는 모든 힘을 방향과 함께 더한 값이며, 질량과 가속도의 곱입니다.",
        "F합: 알짜힘(N), m: 질량(kg), a: 가속도(m/s²). N은 kg·m/s²와 같습니다.",
        `${esc(row.n.name)}의 질량과 작용하는 힘으로 속도 변화인 가속도를 구하기 위해 사용합니다.`,
        "F합 = ma",
        r.type === "free"
          ? `수평: ${f(row.m)}aₓ = ${f(row.n.fx)}<br>
             수직: ${f(row.m)}aᵧ =
             ${f(row.n.fy)} − ${f(row.m * r.scene.g)}`
          : `${f(row.m)}a = ${f(row.m * row.a)}`,
        `힘을 합한 뒤 질량 ${f(row.m)} kg으로 나눕니다.
         가속도는 힘 자체나 속도 자체가 아닙니다.`,
        r.type === "free"
          ? `aₓ=${sg(row.ax)} m/s², aᵧ=${sg(row.ay)} m/s²`
          : `경로 가속도 a=${sg(row.a)} m/s²`
      ) +
      card(
        "지표면 부근의 중력",
        "지구가 물체를 아래로 당기는 힘입니다. 이 실험에서는 g를 일정하게 둡니다.",
        "중력 mg: N, m: kg, g: m/s².",
        "자유낙하와 매달린 물체, 접촉면의 수직항력을 설명하려면 중력을 빠뜨리면 안 됩니다.",
        "중력의 크기 = mg",
        `mg = ${f(row.m)} × ${f(r.scene.g)}`,
        "질량과 중력가속도를 곱하고 힘의 방향을 아래쪽으로 표시합니다.",
        `중력의 크기 ${f(row.m * r.scene.g)} N, 방향은 아래쪽.`
      )
    ).join("");

    const components = r.rows
      .filter(row => row.p && Math.abs(row.p.angle) > 1e-8)
      .map(row => card(
        "경사면에서 중력의 성분",
        "중력을 두 번 세는 것이 아니라 같은 중력을 면에 나란한 방향과 수직인 방향으로 나누어 읽습니다.",
        "m: kg, g: m/s², θ: 경사각(°), mg sinθ와 mg cosθ: N.",
        "면에 수직으로는 가속도가 0이고 면을 따라 운동하므로 두 방향으로 나누면 식이 간단합니다.",
        "면 아래쪽: mg sin|θ|<br>면을 누르는 방향: mg cosθ",
        `mg sin|θ| = ${f(row.m)} × ${f(r.scene.g)}
         × sin ${f(Math.abs(row.p.angle), 1)}°<br>
         mg cosθ = ${f(row.m)} × ${f(r.scene.g)}
         × cos ${f(Math.abs(row.p.angle), 1)}°`,
        "삼각함수 값으로 각 성분의 크기를 구한 뒤, 선택한 +방향과 같은지 반대인지에 따라 부호를 붙입니다.",
        `아래쪽 성분
         ${f(row.m * r.scene.g * Math.abs(Math.sin(row.p.angle * Math.PI / 180)))} N,
         면을 누르는 성분
         ${f(row.m * r.scene.g * Math.cos(row.p.angle * Math.PI / 180))} N.`
      )).join("");

    const equations = r.type === "free"
      ? r.rows.map(row => `
          <div class="sf-equation">
            ${esc(row.n.name)}:
            ${f(row.m)}aₓ=Fₓ<br>
            ${f(row.m)}aᵧ=Fᵧ−${f(row.m)}g
          </div>
        `).join("")
      : r.rows.map((row, i) =>
          `<div class="sf-equation">
            ${equation(r, row, i, false)}
          </div>`
        ).join("");

    const final = `
      <div class="sf-scroll">
        <table>
          <thead><tr>
            <th>물체</th>
            <th>+방향</th>
            <th>가속도</th>
            <th>알짜힘</th>
            ${r.tensions.length ? "<th>연결된 최대 T</th>" : ""}
          </tr></thead>
          <tbody>
          ${r.rows.map(row => `
            <tr>
              <td>${esc(row.n.name)}</td>
              <td>${r.type === "free"
                ? "x 오른쪽 / y 위쪽" : dir(row.u)}</td>
              <td>${r.type === "free"
                ? `(${sg(row.ax)}, ${sg(row.ay)})`
                : sg(row.a)} m/s²</td>
              <td>${r.type === "free"
                ? `(${sg(row.fx)}, ${sg(row.fy)})`
                : sg(row.m * row.a)} N</td>
              ${r.tensions.length
                ? `<td>${f(row.maxT)} N</td>` : ""}
            </tr>
          `).join("")}
          </tbody>
        </table>
      </div>
      <p>이 답은 초기 조건에서 줄 방향과 접촉·마찰 상태가 유지되는
      구간에 적용합니다. 충돌 이후, 줄이 느슨해진 이후,
      면의 끝을 지난 이후, 마찰로 멈춘 이후에는 같은 답을
      계속 적용하지 않습니다.</p>
    `;

    return [
      {
        title: "주어진 조건 정리",
        html: `
          <ul>${given}</ul>
          <p>g=${f(r.scene.g)} m/s².
          줄은 질량·탄성이 없고, 고정 도르래의 마찰·회전 관성은
          무시합니다. 공기 저항도 무시합니다.</p>
        `
      },
      {
        title: "운동 방향 예측",
        html: direction + relation
      },
      {
        title: "자유물체도로 힘 설명",
        html: `
          <div class="sf-grid">
          ${r.rows.map(row => `
            <article>
              <button data-school-select="${row.n.id}">
                ${esc(row.n.name)} 선택
              </button>
              ${diagram(row)}
              <ul>
              ${row.forces
                .filter(q => Math.hypot(q.x, q.y) > 1e-8)
                .map(q => `
                  <li><b>${esc(q.name)}</b>: ${esc(q.why)}</li>
                `).join("")}
              </ul>
            </article>
          `).join("")}
          </div>
        `
      },
      {
        title: "사용할 교과서 공식과 이유",
        html: newton + components +
          r.rows.map(row => normalWork(r, row)).join("")
      },
      {
        title: "물체별 운동 방정식",
        html: relation + equations + `
          <p>장력 T는 줄이 물체를 당기는 힘입니다.
          줄을 가볍고 도르래 마찰을 무시했으므로 같은 한 줄의
          장력 크기를 같게 놓습니다.
          ${r.type === "pulley"
            ? "첫 물체에서는 +T, 둘째 물체에서는 −T입니다. 두 물체의 +축을 서로 다른 방향으로 정했기 때문입니다."
            : "수평 연결에서는 줄이 각 끝의 물체를 서로 향해 당깁니다."}
          두 물체에 작용하는 장력 둘이 곧 작용·반작용 쌍이라는
          뜻은 아닙니다. 각 물체에 대한 장력의 반작용은
          그 물체가 줄에 주는 힘입니다.</p>
        `
      },
      {
        title: "숫자 대입과 식 정리",
        html: mathSteps(r)
      },
      {
        title: "최종 답과 단위",
        html: final
      },
      {
        title: "핵심 개념과 자주 하는 실수",
        html: `
          <ul>
            <li>알짜힘은 힘 하나가 아니라 모든 힘의 합입니다.
            알짜힘 0은 각 힘이 전부 0이라는 뜻이 아닙니다.</li>
            <li>팽팽한 줄의 길이 조건을 확인한 뒤 공통 경로 가속도를
            씁니다. 도르래 양쪽의 전역 방향은 반대일 수 있습니다.</li>
            <li>장력 항을 소거해도 실제 장력이 사라지는 것은
            아닙니다. 각 물체의 식으로 돌아가 장력을 구합니다.</li>
            <li>경사면에서 mg와 mg sinθ를 같은 축의 별도 힘처럼
            함께 더하지 마세요.</li>
            <li>정지 마찰은 항상 μN이 아닙니다.
            필요한 힘이 한계 이내인지 확인합니다.</li>
            <li>음의 가속도는 +방향 반대라는 뜻입니다.
            속도 방향과 같지 않을 수 있습니다.</li>
            <li>장력·알짜힘 단위는 N, 가속도는 m/s²입니다.
            재생 일시정지는 실제 속도가 0이 된 것과 다릅니다.</li>
          </ul>
        `
      }
    ];
  }

  function reference() {
    return `
      <h2>기본 공식 참고 노트</h2>
      <p>아래 수치는 공식의 뜻을 설명하는 별도의 짧은 예입니다.
      현재 배치의 답은 힘과 풀이 탭에서 확인하세요.</p>
    ` +
    card(
      "등가속도 운동",
      "가속도가 일정한 한 구간에서 속도와 변위를 계산합니다.",
      "v₀, v: 처음·나중 속도(m/s), a: m/s², t: s, s: 변위(m).",
      "처음 속도 0, a=2 m/s²가 3초 동안 유지되는 예의 속도와 변위를 구합니다.",
      "v=v₀+at<br>s=v₀t+½at²",
      "v=0+2×3<br>s=0×3+½×2×3²",
      "곱셈과 제곱을 먼저 계산하면 v=6, s=9입니다.",
      "v=6 m/s, s=9 m. 충돌이나 마찰 상태가 바뀌는 구간을 가로질러 같은 a를 사용하면 안 됩니다."
    ) +
    card(
      "일: 힘과 이동이 나란한 경우",
      "힘이 이동을 따라 에너지를 전달한 양입니다. 반대 방향 힘의 일은 음수입니다.",
      "W: 일(J), F∥: 이동 방향 힘(N), s: 이동 거리(m).",
      "이동 방향으로 4 N의 일정한 힘이 3 m 동안 작용하는 예입니다.",
      "W=F∥s",
      "W=4×3",
      "힘과 이동이 같은 방향이므로 양수로 곱합니다.",
      "W=12 J."
    ) +
    card(
      "운동에너지와 일·운동에너지 관계",
      "물체에 작용한 알짜힘의 일은 운동에너지의 변화와 같습니다.",
      "K: 운동에너지(J), m: kg, v: m/s, W합: 알짜힘이 한 일(J).",
      "2 kg 물체의 속력이 1 m/s에서 3 m/s로 변하는 예에서 필요한 일을 구합니다.",
      "K=½mv²<br>W합=K나중−K처음",
      "W합=½×2×3²−½×2×1²",
      "½×2=1이므로 9−1을 계산합니다.",
      "W합=8 J. 마찰이 있다면 그 음의 일도 W합에 포함합니다."
    ) +
    card(
      "역학적 에너지 보존",
      "중력만 일을 하고 마찰·공기 저항을 무시하면 K+U가 일정합니다.",
      "K=½mv²: J, U=mgh: J, h: 기준에 대한 높이(m), g: m/s².",
      "2 kg 물체를 높이 5 m에서 정지 상태로 놓고 g=10 m/s²로 계산하는 예입니다.",
      "K처음+U처음=K나중+U나중",
      "0+2×10×5=½×2×v²+0",
      "양변을 1로 정리하면 v²=100입니다. 속력은 양의 제곱근입니다.",
      "바닥에 닿기 직전 v=10 m/s. 충돌 후에도 이 보존식을 그대로 적용하지 않습니다."
    ) +
    `
      <p class="sf-note">
        교과서 대조 상태: 교육부의 2022 개정 교육과정 고시와
        천재교과서 「물리학」 소개 자료를 확인했습니다.
        천재교과서 「역학과 에너지」 본문·쪽수는 접속 오류로
        대조하지 못했습니다. 이 도구는 사용자 요청의 기본 역학
        공식으로 검증한 유형만 풀이하며 천재교과서 전체 범위를
        재현하거나 공식 인증을 받은 자료가 아닙니다.
      </p>
      <p>
        <a href="https://www.moe.go.kr/boardCnts/viewRenew.do?boardID=141&boardSeq=93458&lev=0"
          target="_blank" rel="noopener">교육부 교육과정 고시</a>
        ·
        <a href="https://text.tsherpa.co.kr/high/book.html?bookcode=text-high-science-20-sci-01"
          target="_blank" rel="noopener">천재교과서 물리학</a>
      </p>
    `;
  }

  return { steps, reference, card, diagram, f, esc, dir };
}

function SchoolUI() {
  const C = SchoolPhysicsCore();
  const L = SchoolLessons(C);
  const $ = id => document.getElementById(id);

  const TYPE = {
    body: "물체",
    platform: "수평 발판",
    ramp: "경사면",
    fixed: "고정 도르래",
    moving: "이동 도르래",
    anchor: "고정점",
    rope: "줄"
  };

  let capturing = false;
  let timer = 0;
  let lastJSON = "";
  let result = null;
  let currentStep = 0;
  let showAnswers = false;

  const analysis = $("analysis");
  if (!analysis || !$("sceneList")) return;

  const style = document.createElement("style");
  style.textContent = `
    #studyPanel,.school-legacy{display:none!important}
    .sf-panel{padding:18px}
    .sf-panel h2{margin-bottom:12px}
    .sf-panel p,.sf-panel li{line-height:1.9}
    .sf-panel h3{font-size:18px;margin:18px 0 10px}
    .sf-panel h4{font-size:15px;margin:0 0 10px}
    .sf-card{border:1px solid #dce6d6;padding:16px;border-radius:12px;background:#fff;margin:14px 0}
    .sf-card p{margin:9px 0}
    .sf-equation{background:#edf5e7;padding:12px;border-radius:9px;margin:10px 0;font:14px/1.9 ui-monospace,monospace;overflow-wrap:anywhere}
    .sf-note{padding:12px 14px;background:#edf5e8;border-left:3px solid #69a17b;border-radius:0 9px 9px 0;font-size:12px;white-space:pre-line}
    .sf-note.bad{background:#fff0e4;border-color:#cf9868}
    .sf-nav{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:14px 0}
    .sf-scroll{overflow:auto}
    .sf-scroll table{font:12px/1.65 system-ui,sans-serif}
    .sf-scroll td,.sf-scroll th{padding:8px}
    .sf-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:14px}
    .sf-grid svg{width:100%;background:#f5f8ef;border-radius:10px}
    #schoolDock{position:sticky;top:0;z-index:10;background:#fbfdf8;align-self:start;padding:12px}
    #schoolDock .sf-scroll{max-height:150px}
    #schoolDock table{margin-top:5px}
    .sf-predict{max-width:110px;width:100%}
    .sf-choice{display:block;text-align:left;width:100%;margin:9px 0}
    .sf-current{font-size:12px;color:#64805f;padding:5px}
    @media(max-width:600px){
      .sf-panel{padding:12px}
      .sf-equation{font-size:12px}
      .sf-card{padding:12px}
      .sf-panel p{font-size:14px}
    }
  `;
  document.head.append(style);

  const old = document.createElement("div");
  old.className = "school-legacy";
  old.hidden = true;

  for (const child of [...analysis.childNodes]) {
    old.append(child);
  }
  analysis.append(old);

  const root = document.createElement("section");
  root.id = "schoolRoot";
  root.className = "sf-panel";
  root.innerHTML = `
    <h2>교과서 기본 원리로 푸는 역학</h2>
    <p class="small">
      초기 배치의 조건으로 풀이합니다.
      운동 중 수치 관찰값과 초기 문제의 답을 구분하세요.
    </p>

    <div class="sf-nav">
      <button id="schoolRead">현재 배치 다시 분석</button>
      <select id="schoolExample" aria-label="교과서형 예제">
        <option value="train">수평면 두 물체</option>
        <option value="atwood">고정 도르래 두 물체</option>
        <option value="slopePulley">경사면과 고정 도르래</option>
        <option value="friction">마찰이 있는 연결 물체</option>
        <option value="three">수평면 세 물체</option>
        <option value="incline">경사면 한 물체</option>
        <option value="fall">자유낙하와 에너지</option>
      </select>
      <button id="schoolLoad">예제 배치</button>
    </div>

    <p id="schoolScope" class="sf-note" role="status"></p>
    <div id="schoolPredict"></div>

    <div class="sf-nav">
      <button id="schoolReveal" class="primary">답과 풀이 확인</button>
      <button id="schoolAgain">다시 예측</button>
    </div>

    <div class="sf-nav">
      <button id="schoolPrev">이전 단계</button>
      <b id="schoolStepLabel"></b>
      <button id="schoolNext">다음 단계</button>
    </div>

    <div id="schoolLesson"></div>
  `;
  analysis.append(root);

  const column = document.querySelector(".work-column");
  const dock = document.createElement("section");
  dock.id = "schoolDock";
  dock.className = "card";
  column.insertBefore(dock, column.firstChild);

  const current = document.createElement("p");
  current.className = "sf-current";
  current.textContent =
    "시뮬레이션 수치는 운동 관찰용입니다. 교과서 문제의 답은 학습 풀이에서 검증한 초기 조건과 적용 구간을 기준으로 확인하세요.";

  const metrics = document.querySelector(".metrics");
  metrics.parentElement.insertBefore(current, metrics);

  $("notes").innerHTML = L.reference();

  function freshButton(id) {
    return [...$("sceneList").querySelectorAll("[data-list-id]")]
      .find(b => b.dataset.listId === id);
  }

  function readScene() {
    const entries =
      [...$("sceneList").querySelectorAll("[data-list-id]")]
        .map(b => {
          const label = b.textContent.trim();
          const kind = Object.keys(TYPE).find(k =>
            label.startsWith(TYPE[k] + " · ")
          );

          if (!kind) {
            throw Error(
              "항목 종류를 읽을 수 없습니다. 현재 app.js 버전과 파일을 확인하세요."
            );
          }

          return { id: b.dataset.listId, kind };
        });

    if (entries.length > 24) {
      throw Error(
        "학습 풀이에서는 항목 24개 이하로 정리하세요. 기존 운동 관찰은 유지됩니다."
      );
    }

    const selected =
      $("sceneList").querySelector(".chosen")?.dataset.listId;

    const nodes = [];
    const ropes = [];
    capturing = true;

    try {
      for (const item of entries) {
        const button = freshButton(item.id);

        if (!button) {
          throw Error("장면 목록이 바뀌었습니다. 다시 분석하세요.");
        }

        button.click();

        const values = {};
        $("inspector").querySelectorAll("[data-prop]").forEach(el => {
          values[el.dataset.prop] =
            el.dataset.prop === "name"
              ? el.value
              : Number(el.value);
        });

        if (item.kind === "rope") {
          const paragraph =
            $("inspector").querySelector("p.small");

          if (!paragraph) {
            throw Error("줄의 경로를 읽을 수 없습니다.");
          }

          const names = paragraph.textContent
            .replace(/^\s*현재 경로\s*[-+\d.eE]+\s*m\s*/, "")
            .trim()
            .split("→")
            .map(n => n.trim());

          ropes.push({
            ...item,
            name: values.name,
            length: values.length,
            names
          });
        } else {
          nodes.push({
            m: 2,
            radius: 0.35,
            mu: 0,
            fx: 0,
            fy: 0,
            vx: 0,
            vy: 0,
            length: 6,
            angle: 0,
            ...item,
            ...values
          });
        }
      }

      const allNames = nodes.map(n => n.name);

      if (allNames.some(name =>
        typeof name !== "string" || name.includes("→")
      ) || new Set(allNames).size !== allNames.length) {
        throw Error(
          "물체·면·도르래의 이름을 서로 다르게 정하고 ‘→’ 문자는 이름에 사용하지 마세요."
        );
      }

      for (const rope of ropes) {
        rope.path = rope.names.map(name => {
          const n = nodes.find(n => n.name === name);
          if (!n) {
            throw Error("줄의 연결점 이름을 확인할 수 없습니다.");
          }
          return n.id;
        });
        delete rope.names;
      }

      return {
        version: 1,
        g: Number($("gravity").value),
        nodes,
        ropes
      };
    } finally {
      if (selected) freshButton(selected)?.click();
      capturing = false;
    }
  }

  function schedule() {
    if (capturing) return;
    clearTimeout(timer);
    timer = setTimeout(() => analyze(), 180);
  }

  function analyze(force = false) {
    if (capturing) return;

    try {
      const scene = readScene();
      const json = JSON.stringify(scene);

      if (!force && json === lastJSON) return;

      lastJSON = json;
      result = C.solve(scene);
      currentStep = 0;
      showAnswers = false;
      render(true);
    } catch (e) {
      result = {
        ok: false,
        title: "조건을 읽을 수 없습니다",
        reason: e.message
      };
      currentStep = 0;
      render(true);
    }
  }

  function dockHTML() {
    if (!result?.ok) {
      return `
        <b>학습 범위 확인</b>
        <p class="sf-note bad">
          ${L.esc(result?.reason || "현재 배치를 분석하세요.")}
        </p>
      `;
    }

    const r = result;

    return `
      <b>초기 문제 조건과 계산 결과</b>
      <span class="small">
        · 줄·접촉 상태가 바뀌기 전의 구간
      </span>

      <div class="sf-scroll">
        <table>
          <thead><tr>
            <th>물체</th>
            <th>m (kg)</th>
            <th>외력 (N)</th>
            <th>가속도 (m/s²)</th>
            ${r.tensions.length ? "<th>최대 T (N)</th>" : ""}
          </tr></thead>
          <tbody>
          ${r.rows.map(row => `
            <tr>
              <td>${L.esc(row.n.name)}</td>
              <td>${L.f(row.m)}</td>
              <td>(${L.f(row.n.fx)}, ${L.f(row.n.fy)})</td>
              <td>${showAnswers
                ? r.type === "free"
                  ? `(${L.f(row.ax)}, ${L.f(row.ay)})`
                  : L.f(row.a)
                : "예측 후 확인"}</td>
              ${r.tensions.length
                ? `<td>${showAnswers ? L.f(row.maxT) : "—"}</td>`
                : ""}
            </tr>
          `).join("")}
          </tbody>
        </table>
      </div>
    `;
  }

  function render(rebuild = false) {
    if (!result) return;

    const steps = L.steps(result);
    currentStep = Math.min(currentStep, steps.length - 1);

    $("schoolScope").textContent = result.ok
      ? "이 유형은 뉴턴 운동 법칙과 기본 연립식으로 검증했습니다. 줄이 느슨해지거나 접촉·마찰 상태가 바뀌면 초기 답의 적용이 끝납니다. 천재교과서 본문 전체와의 대조는 완료하지 못했습니다."
      : (result.title || "현재 학습 범위를 벗어난 문제") +
        "\n" + result.reason;

    $("schoolScope").classList.toggle("bad", !result.ok);
    $("schoolStepLabel").textContent =
      `${currentStep + 1} / ${steps.length} · ${steps[currentStep].title}`;

    $("schoolPrev").disabled = currentStep === 0;
    $("schoolNext").disabled = currentStep === steps.length - 1;
    $("schoolNext").textContent =
      currentStep === 3 && !showAnswers
        ? "계산 공개하고 다음 단계"
        : "다음 단계";

    $("schoolLesson").innerHTML = steps[currentStep].html;
    $("schoolReveal").disabled = !result.ok;
    $("schoolDock").innerHTML = dockHTML();

    if (rebuild) {
      $("schoolPredict").innerHTML = result.ok
        ? `
          <h3>먼저 예측하기 <small>선택 사항</small></h3>
          <p class="small">
            ${result.type === "free"
              ? "가속도는 위쪽 +y 성분을 입력하세요."
              : "가속도는 각 물체에 정한 경로 +방향 기준입니다."}
            장력은 연결된 줄 중 가장 큰 크기입니다.
          </p>
          <div class="sf-scroll">
            <table>
              <thead><tr>
                <th>물체</th>
                <th>a (m/s²)</th>
                ${result.tensions.length ? "<th>T (N)</th>" : ""}
              </tr></thead>
              <tbody>
              ${result.rows.map(row => `
                <tr>
                  <td>${L.esc(row.n.name)}</td>
                  <td>
                    <input class="sf-predict"
                      data-answer="${row.n.id}:a"
                      type="number" step="any"
                      aria-label="${L.esc(row.n.name)} 가속도 예측">
                  </td>
                  ${result.tensions.length ? `
                    <td>
                      <input class="sf-predict"
                        data-answer="${row.n.id}:T"
                        type="number" step="any"
                        aria-label="${L.esc(row.n.name)} 장력 예측">
                    </td>
                  ` : ""}
                </tr>
              `).join("")}
              </tbody>
            </table>
          </div>
          <p id="schoolGrade" role="status"></p>
        `
        : "";
    }
  }

  function reveal() {
    if (!result?.ok) return;
    showAnswers = true;
    const grades = [];

    for (const row of result.rows) {
      for (const [key, value] of [
        ["a", result.type === "free" ? row.ay : row.a],
        ["T", row.maxT]
      ]) {
        const input =
          [...$("schoolPredict").querySelectorAll("[data-answer]")]
            .find(el => el.dataset.answer === row.n.id + ":" + key);

        if (!input || input.value.trim() === "") continue;

        const ok = Math.abs(Number(input.value) - value) <=
          Math.max(0.02, Math.abs(value) * 0.01);

        grades.push(
          `${row.n.name} ${key}: ` +
          `${ok ? "허용 오차 내 정답" : "다시 확인"} · ` +
          `계산값 ${L.f(value)} ${key === "T" ? "N" : "m/s²"}`
        );
      }
    }

    if ($("schoolGrade")) {
      $("schoolGrade").textContent = grades.join("\n") ||
        "현재 배치의 수치 대입과 중간 계산을 단계별로 확인하세요.";
    }

    render();
  }

  $("schoolRead").onclick = () => analyze(true);
  $("schoolReveal").onclick = reveal;
  $("schoolAgain").onclick = () => {
    showAnswers = false;
    currentStep = 0;
    render(true);
  };
  $("schoolPrev").onclick = () => {
    currentStep = Math.max(0, currentStep - 1);
    render();
  };
  $("schoolNext").onclick = () => {
    if (!result) return;

    currentStep = Math.min(
      L.steps(result).length - 1,
      currentStep + 1
    );

    if (currentStep >= 4) reveal();
    else render();
  };

  root.addEventListener("click", e => {
    const button = e.target.closest("[data-school-select]");
    if (button) {
      freshButton(button.dataset.schoolSelect)?.click();
    }
  });

  const nativeFileChange = $("fileInput").onchange;
  $("fileInput").onchange = async e => {
    await nativeFileChange(e);
    schedule();
  };

  $("schoolLoad").onclick = async () => {
    try {
      const data = C.sample($("schoolExample").value);
      const text = JSON.stringify(data);
      const file = new File(
        [text],
        "school-example.json",
        { type: "application/json" }
      );

      await $("fileInput").onchange({
        target: { files: [file] }
      });
      analyze(true);
    } catch (e) {
      $("schoolScope").textContent =
        "예제 배치 실패: " + e.message;
    }
  };

  document.addEventListener("change", e => {
    if (e.target.closest("#inspector") ||
        e.target.id === "gravity") {
      schedule();
    }
  }, true);

  document.addEventListener("click", e => {
    if (e.target.closest("#loadExample") ||
        e.target.closest("#deleteItem") ||
        e.target.closest("#loadLocal") ||
        e.target.closest("#finishRope")) {
      schedule();
    }
  }, true);

  $("workspace").addEventListener("pointerup", schedule);
  document.addEventListener("keyup", e => {
    if (e.key === "Delete") schedule();
  });

  function verify() {
    const results = [];

    const near = (a, b) => {
      if (Math.abs(a - b) > 1e-7) {
        throw Error(a + " ≠ " + b);
      }
    };

    const test = (name, fn) => {
      try {
        fn();
        results.push("✓ " + name);
      } catch (e) {
        results.push("✕ " + name + ": " + e.message);
      }
    };

    test("예제 1 수평 연결: a=2.4, T=4.8", () => {
      const r = C.solve(C.sample("train"));
      if (!r.ok) throw Error(r.reason);
      near(r.a, 2.4);
      near(r.tensions[0].T, 4.8);
    });

    test("예제 2 고정 도르래: a=1.96, T=23.52", () => {
      const r = C.solve(C.sample("atwood"));
      if (!r.ok) throw Error(r.reason);
      near(r.a, 1.96);
      near(r.tensions[0].T, 23.52);
    });

    test("예제 3 마찰 연결: a=1.64, T=7.2", () => {
      const r = C.solve(C.sample("friction"));
      if (!r.ok) throw Error(r.reason);
      near(r.a, 1.64);
      near(r.tensions[0].T, 7.2);
    });

    test("경사면과 도르래: a=3.92, T=17.64", () => {
      const r = C.solve(C.sample("slopePulley"));
      if (!r.ok) throw Error(r.reason);
      near(r.a, 3.92);
      near(r.tensions[0].T, 17.64);
    });

    test("세 물체: a=2, T1=4, T2=10", () => {
      const r = C.solve(C.sample("three"));
      near(r.a, 2);
      near(r.tensions[0].T, 4);
      near(r.tensions[1].T, 10);
    });

    test("정지 마찰 한계와 실제 마찰 구별", () => {
      const s = C.sample("incline");
      s.nodes.find(C.surface).angle = 0;

      const body = s.nodes.find(C.dynamic);
      body.x = 8;
      body.y = 3.35;
      body.fx = 1;

      const r = C.solve(s);
      if (!r.ok) throw Error(r.reason);
      near(r.a, 0);
      near(r.rows[0].f, -1);
    });

    test("느슨한 줄을 공통 가속도로 풀지 않음", () => {
      const s = C.sample("train");
      s.ropes[0].length += 1;
      if (C.solve(s).ok) throw Error("범위 검사 실패");
    });

    test("분담이 불확실한 정지 마찰 장력을 단정하지 않음", () => {
      const s = C.sample("friction");
      s.nodes.filter(C.dynamic)[1].fx = 0;
      if (C.solve(s).ok) throw Error("추가 조건 안내 없음");
    });

    test("이동 도르래의 미검증 풀이 차단", () => {
      const s = C.sample("train");
      s.nodes.find(C.dynamic).kind = "moving";
      if (C.solve(s).ok) throw Error("범위 검사 실패");
    });

    test("지원 예제 모두 8단계, 고급 수학 풀이 없음", () => {
      for (const kind of [
        "train", "atwood", "friction", "incline",
        "slopePulley", "three", "fall"
      ]) {
        const steps = L.steps(C.solve(C.sample(kind)));

        if (steps.length !== 8) {
          throw Error(kind + " 단계 누락");
        }

        if (/가우스|행렬|야코비|미분방정식|J M/.test(
          steps.map(s => s.html).join("")
        )) {
          throw Error("고급 풀이 노출");
        }
      }
    });

    test("현재 배치의 조건 읽기와 선택 복원", () => {
      const original =
        $("sceneList").querySelector(".chosen")?.dataset.listId;

      const s = readScene();
      if (!s.nodes.some(C.dynamic)) throw Error("물체 없음");

      if (original !==
          $("sceneList").querySelector(".chosen")?.dataset.listId) {
        throw Error("선택 복원 실패");
      }
    });

    $("schoolCheckOutput").textContent =
      results.join("\n") +
      "\n\n이 검사는 실제 모바일 렌더링·터치 테스트를 대신하지 않습니다.";

    return results;
  }

  const testBox = document.createElement("section");
  testBox.innerHTML = `
    <h3>교과서 풀이 검증</h3>
    <button id="schoolCheck">학습 풀이 검증 실행</button>
    <pre id="schoolCheckOutput"
      aria-live="polite">아직 실행하지 않았습니다.</pre>
  `;
  $("checks").append(testBox);
  $("schoolCheck").onclick = verify;

  const quizRoot = $("quiz");
  const quizOld = document.createElement("div");
  quizOld.className = "school-legacy";
  quizOld.hidden = true;

  for (const child of [...quizRoot.childNodes]) {
    quizOld.append(child);
  }
  quizRoot.append(quizOld);

  const quizNew = document.createElement("div");
  quizNew.className = "sf-panel";
  quizNew.innerHTML = `
    <h2 id="sqQuestion"></h2>
    <div id="sqChoices"></div>
    <button id="sqSubmit" class="primary">정답 제출</button>
    <button id="sqNext">다음 문제</button>
    <p id="sqFeedback" class="sf-note"
      role="status" hidden></p>
  `;
  quizRoot.append(quizNew);

  const questions = [
    {
      q: "알짜힘 F합=ma에서 F합은 무엇인가요?",
      options: [
        "중력만",
        "물체에 작용하는 힘의 방향을 고려한 합",
        "장력만"
      ],
      answer: 1,
      why: "여러 힘을 같은 축에서 부호와 함께 더한 값입니다."
    },
    {
      q: "팽팽한 한 줄과 고정 도르래의 두 물체를 공통 경로 가속도로 푸는 이유는?",
      options: ["질량이 같아서", "줄 길이가 일정해서", "중력이 없어져서"],
      answer: 1,
      why: "한쪽 길이가 늘어난 만큼 다른 쪽이 줄어야 합니다. 전역 좌표 방향은 반대일 수 있습니다."
    },
    {
      q: "정지 마찰력의 크기는 언제나 μN인가요?",
      options: ["항상 그렇다", "필요한 만큼 작용하며 최대 μN이다", "항상 0이다"],
      answer: 1,
      why: "미끄러짐을 막는 데 필요한 힘이 한계 이내이면 그만큼만 작용합니다."
    },
    {
      q: "경사면 아래쪽 중력 성분은?",
      options: ["mg cosθ", "mg sinθ", "mg+mg sinθ"],
      answer: 1,
      why: "같은 중력을 두 축으로 나눈 성분입니다. mg를 별도 힘처럼 다시 더하지 않습니다."
    },
    {
      q: "두 식을 더해 장력 항이 소거되면 실제 장력은 없어지나요?",
      options: ["없어진다", "각 물체에는 계속 작용한다", "질량이 0이 된다"],
      answer: 1,
      why: "미지수를 없애는 식의 정리입니다. 각 물체의 식으로 돌아가 장력을 구해야 합니다."
    },
    {
      q: "마찰이 없는 낙하에서 위치에너지가 감소하면?",
      options: ["에너지가 사라진다", "운동에너지가 증가할 수 있다", "중력이 0이 된다"],
      answer: 1,
      why: "중력만 일을 하는 구간에서는 운동에너지와 위치에너지의 합이 일정합니다."
    }
  ];

  let questionIndex = 0;
  let questionChoice = -1;
  let questionDone = false;

  function showQuiz() {
    const q = questions[questionIndex];
    $("sqQuestion").textContent =
      `${questionIndex + 1}/${questions.length}. ${q.q}`;

    $("sqChoices").innerHTML = q.options.map((o, i) => `
      <button data-sq="${i}" class="sf-choice"
        aria-pressed="${questionChoice === i}"
        ${questionDone ? "disabled" : ""}>
        ${i + 1}. ${L.esc(o)}
      </button>
    `).join("");

    $("sqSubmit").disabled = questionDone || questionChoice < 0;
    $("sqFeedback").hidden = !questionDone;

    if (questionDone) {
      $("sqFeedback").textContent =
        (questionChoice === q.answer
          ? "정답입니다. "
          : "정답은 " + (q.answer + 1) + "번입니다. ") +
        q.why;
    }
  }

  $("sqChoices").onclick = e => {
    const b = e.target.closest("[data-sq]");
    if (b && !questionDone) {
      questionChoice = Number(b.dataset.sq);
      showQuiz();
    }
  };
  $("sqSubmit").onclick = () => {
    if (questionChoice >= 0) {
      questionDone = true;
      showQuiz();
    }
  };
  $("sqNext").onclick = () => {
    questionIndex = (questionIndex + 1) % questions.length;
    questionChoice = -1;
    questionDone = false;
    showQuiz();
  };

  showQuiz();

  window.PhysicsSchool = {
    solve: C.solve,
    sample: C.sample,
    readScene,
    analyze,
    verify
  };

  analyze();
}

SchoolUI();
