#!/usr/bin/env python3
"""Plan de "soldadura" sobre la placa a partir de sus 4 esquinas. No mueve el robot.

    python3 plan_placa.py                     # revisar el plan (en el host o en el contenedor)
    python3 plan_placa.py --rejilla 2x3 --margen-cm 3 --altura-mm 3

Lee datos/poses.json (home, soldadura) y datos/poses_info.json (esquina_1..4). Cada punto de la
rejilla se resuelve con cinemática inversa a ángulos articulares, para ejecutarlo con
goto_joints igual que el ciclo. La cinemática es la DH del Panda en Python puro; se
comprobó que coincide con fa.get_pose() a 0.1 mm en las poses grabadas.
"""
import argparse
import json
import math
import os
import sys

DIR_DATOS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "datos"))
TCP_FRANKAPY = 0.1034       # m, de la brida al TCP por defecto de frankapy (entre los dedos)
DH = [(0, 0.333, 0), (0, 0, -math.pi / 2), (0, 0.316, math.pi / 2), (0.0825, 0, math.pi / 2),
      (-0.0825, 0.384, -math.pi / 2), (0, 0, math.pi / 2), (0.088, 0, math.pi / 2)]
LIM_INF = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
LIM_SUP = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
MARGEN_LIM = 0.05           # rad de margen a los límites articulares

ALTURA = 0.003              # m sobre la superficie al "soldar"
SUBIDA = 0.06               # m sobre la superficie al desplazarse entre puntos
MARGEN = 0.03               # m desde el borde de la placa
ESPERA = 1.5                # s "soldando" en cada punto
FACTOR_BAJADA = 1.5         # la bajada hacia la placa dura 1.5 veces más (más suave)
DESV_MAX = 0.01             # m, desviación lateral máxima aceptada en subidas/bajadas


# ---------- cinemática ----------

def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _dh(a, d, al, th):
    ca, sa, ct, st = math.cos(al), math.sin(al), math.cos(th), math.sin(th)
    return [[ct, -st, 0, a], [st * ca, ct * ca, -sa, -d * sa], [st * sa, ct * sa, ca, d * ca], [0, 0, 0, 1]]


def fk(q, largo):
    """Posición de la punta del bolígrafo y rotación de la herramienta."""
    T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    for (a, d, al), th in zip(DH, q):
        T = _mul(T, _dh(a, d, al, th))
    T = _mul(T, _dh(0, 0.107 + TCP_FRANKAPY + largo, 0, 0))
    return [T[i][3] for i in range(3)], [row[:3] for row in T[:3]]


def _error(q, p_obj, R_obj, largo):
    p, R = fk(q, largo)
    e = [p_obj[i] - p[i] for i in range(3)]
    rot = [0.0, 0.0, 0.0]
    for c in range(3):
        a = [R[i][c] for i in range(3)]
        b = [R_obj[i][c] for i in range(3)]
        rot[0] += 0.5 * (a[1] * b[2] - a[2] * b[1])
        rot[1] += 0.5 * (a[2] * b[0] - a[0] * b[2])
        rot[2] += 0.5 * (a[0] * b[1] - a[1] * b[0])
    return e + rot


def _resolver(A, b):
    """Gauss con pivoteo parcial."""
    n = len(b)
    M = [list(A[i]) + [b[i]] for i in range(n)]
    for c in range(n):
        f = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[f] = M[f], M[c]
        for r in range(c + 1, n):
            k = M[r][c] / M[c][c]
            for j in range(c, n + 1):
                M[r][j] -= k * M[c][j]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (M[r][n] - sum(M[r][j] * x[j] for j in range(r + 1, n))) / M[r][r]
    return x


def ik(p_obj, R_obj, semilla, largo, iters=200):
    """Mínimos cuadrados amortiguados desde la semilla (se queda cerca de esa postura)."""
    q = list(semilla)
    h, lam2 = 1e-6, 1e-4
    for _ in range(iters):
        e = _error(q, p_obj, R_obj, largo)
        if max(abs(x) for x in e[:3]) < 1e-5 and max(abs(x) for x in e[3:]) < 1e-4:
            return q
        # Jacobiano numérico de la posición/orientación alcanzada (= -d error / dq)
        J = []
        for j in range(7):
            qh = list(q)
            qh[j] += h
            eh = _error(qh, p_obj, R_obj, largo)
            J.append([(e[i] - eh[i]) / h for i in range(6)])
        JJt = [[sum(J[k][i] * J[k][j] for k in range(7)) + (lam2 if i == j else 0.0) for j in range(6)]
               for i in range(6)]
        y = _resolver(JJt, e)
        dq = [sum(J[k][i] * y[i] for i in range(6)) for k in range(7)]
        paso = max(abs(x) for x in dq)
        if paso > 0.1:
            dq = [x * 0.1 / paso for x in dq]
        q = [q[k] + dq[k] for k in range(7)]
    return None


def dentro_limites(q):
    return all(lo + MARGEN_LIM <= x <= hi - MARGEN_LIM for x, lo, hi in zip(q, LIM_INF, LIM_SUP))


def revisar_tramo(qa, qb, largo, n=40):
    """Altura mínima de la punta y desviación lateral respecto a la recta entre extremos."""
    pa, _ = fk(qa, largo)
    pb, _ = fk(qb, largo)
    z_min, desv = 9.0, 0.0
    for k in range(n + 1):
        s = k / float(n)
        p, _ = fk([x + s * (y - x) for x, y in zip(qa, qb)], largo)
        z_min = min(z_min, p[2])
        recta = [pa[i] + s * (pb[i] - pa[i]) for i in range(2)]
        desv = max(desv, math.hypot(p[0] - recta[0], p[1] - recta[1]))
    return z_min, desv, pa, pb


# ---------- placa ----------

def bilineal(E, u, v):
    """E = esquinas 1..4; u va de 1 a 2 (eje y), v de 1 a 4 (eje x, hacia el operador)."""
    return [(1 - u) * (1 - v) * E[0][i] + u * (1 - v) * E[1][i] + u * v * E[2][i] + (1 - u) * v * E[3][i]
            for i in range(3)]


def puntos_rejilla(E, filas, cols, margen):
    largo_u = math.hypot(E[1][0] - E[0][0], E[1][1] - E[0][1])
    largo_v = math.hypot(E[3][0] - E[0][0], E[3][1] - E[0][1])
    mu, mv = margen / largo_u, margen / largo_v
    if mu >= 0.5 or mv >= 0.5:
        raise ValueError("el margen es mayor que media placa")

    def reparto(n, m):
        return [0.5] if n == 1 else [m + (1 - 2 * m) * k / float(n - 1) for k in range(n)]

    puntos = []
    for i, v in enumerate(reparto(filas, mv)):
        us = reparto(cols, mu)
        for u in (us if i % 2 == 0 else us[::-1]):   # en zigzag: menos recorrido
            puntos.append(bilineal(E, u, v))
    return puntos


def construir_plan(poses, info, filas=2, cols=3, margen=MARGEN, altura=ALTURA, subida=SUBIDA,
                   espera=ESPERA, factor_bajada=FACTOR_BAJADA):
    """Devuelve (plan, informe). plan = lista de {"nombre", "q", "espera", "factor"}."""
    faltan = [n for n in ("esquina_1", "esquina_2", "esquina_3", "esquina_4", "soldadura") if n not in info]
    if faltan or "home" not in poses or "soldadura" not in poses:
        raise ValueError("faltan datos: %s (graba con grabar_poses.py)" % ", ".join(faltan or ["home/soldadura"]))
    largo = info["esquina_1"]["largo_boligrafo_cm"] / 100.0
    E = [info["esquina_%d" % i]["punta_xyz"] for i in range(1, 5)]
    _, R = fk(poses["soldadura"], largo)        # orientación del bolígrafo grabada en soldadura

    informe = ["Placa: rejilla %dx%d, margen %.1f cm, soldar a %.1f mm, desplazarse a %.1f cm, %.1f s por punto"
               % (filas, cols, margen * 100, altura * 1000, subida * 100, espera)]
    plan = [{"nombre": "home", "q": list(poses["home"]), "espera": 0.0, "factor": 1.0}]
    semilla = list(poses["soldadura"])
    for k, sup in enumerate(puntos_rejilla(E, filas, cols, margen), 1):
        p_sobre = [sup[0], sup[1], sup[2] + subida]
        p_baja = [sup[0], sup[1], sup[2] + altura]
        q_sobre = ik(p_sobre, R, semilla, largo)
        q_baja = ik(p_baja, R, q_sobre, largo) if q_sobre else None
        if q_sobre is None or q_baja is None:
            raise ValueError("sin solución de IK para el punto p%d (%.3f, %.3f)" % (k, sup[0], sup[1]))
        for nombre, q in (("sobre", q_sobre), ("baja", q_baja)):
            if not dentro_limites(q):
                raise ValueError("p%d (%s) queda cerca de un límite articular" % (k, nombre))
        semilla = q_sobre
        informe.append("  p%d: x=%.1f y=%.1f cm, superficie z=%.1f cm" % (k, sup[0] * 100, sup[1] * 100, sup[2] * 100))
        plan.append({"nombre": "sobre_p%d" % k, "q": q_sobre, "espera": 0.0, "factor": 1.0})
        plan.append({"nombre": "baja_p%d" % k, "q": q_baja, "espera": espera, "factor": factor_bajada})
        plan.append({"nombre": "sube_p%d" % k, "q": q_sobre, "espera": 0.0, "factor": 1.0})
    plan.append({"nombre": "home", "q": list(poses["home"]), "espera": 0.0, "factor": 1.0})

    # Revisar cada tramo tal como lo ejecuta goto_joints (interpolación articular)
    z_sup = min(p[2] for p in E)
    peor_desv = 0.0
    for a, b in zip(plan[1:-1], plan[2:-1]):    # tramos entre puntos (sin los de home)
        z_min, desv, pa, pb = revisar_tramo(a["q"], b["q"], largo)
        if z_min < min(pa[2], pb[2]) - 0.002 or z_min < z_sup + altura - 0.002:
            raise ValueError("el tramo %s -> %s baja a z=%.1f cm" % (a["nombre"], b["nombre"], z_min * 100))
        vertical = b["nombre"].startswith(("baja", "sube"))
        if vertical:
            if desv > DESV_MAX:
                raise ValueError("el tramo %s -> %s se desvía %.1f mm" % (a["nombre"], b["nombre"], desv * 1000))
            peor_desv = max(peor_desv, desv)
    for a, b in ((plan[0], plan[1]), (plan[-2], plan[-1])):
        z_min, _, pa, pb = revisar_tramo(a["q"], b["q"], largo)
        if z_min < min(pa[2], pb[2]) - 0.002:
            raise ValueError("el tramo %s -> %s baja por debajo de sus extremos" % (a["nombre"], b["nombre"]))
    informe.append("Revisado: %d segmentos, ningún tramo baja de sus extremos, desviación máx. en "
                   "bajadas/subidas %.1f mm" % (len(plan), peor_desv * 1000))
    return plan, informe


def cargar(dir_datos=DIR_DATOS):
    with open(os.path.join(dir_datos, "poses.json")) as f:
        poses = json.load(f)
    with open(os.path.join(dir_datos, "poses_info.json")) as f:
        info = json.load(f)
    return poses, info


def rejilla(texto):
    try:
        f, c = [int(x) for x in texto.lower().split("x")]
        assert 1 <= f <= 5 and 1 <= c <= 5
        return f, c
    except (ValueError, AssertionError):
        raise argparse.ArgumentTypeError("usa FILASxCOLUMNAS, p. ej. 2x3 (máx. 5x5)")


def agregar_argumentos(p):
    p.add_argument("--rejilla", type=rejilla, default=(2, 3), help="filas x columnas (por defecto 2x3)")
    p.add_argument("--margen-cm", type=float, default=MARGEN * 100)
    p.add_argument("--altura-mm", type=float, default=ALTURA * 1000, help="altura de soldadura sobre la placa")
    p.add_argument("--subida-cm", type=float, default=SUBIDA * 100, help="altura al desplazarse entre puntos")
    p.add_argument("--espera", type=float, default=ESPERA, help="s soldando en cada punto")


def plan_desde_args(args, poses, info):
    if not 2.0 <= args.altura_mm <= 20.0:
        raise ValueError("--altura-mm debe estar entre 2 y 20")
    if not 3.0 <= args.subida_cm <= 15.0:
        raise ValueError("--subida-cm debe estar entre 3 y 15")
    return construir_plan(poses, info, args.rejilla[0], args.rejilla[1], args.margen_cm / 100.0,
                          args.altura_mm / 1000.0, args.subida_cm / 100.0, max(0.0, args.espera))


def main():
    p = argparse.ArgumentParser(description="Revisar el plan de soldadura sobre la placa (no mueve el robot)")
    agregar_argumentos(p)
    args = p.parse_args()
    poses, info = cargar()
    try:
        plan, informe = plan_desde_args(args, poses, info)
    except ValueError as e:
        sys.exit("[error] plan no válido: %s" % e)
    print("\n".join(informe))
    print("Segmentos: " + ", ".join(s["nombre"] for s in plan))


if __name__ == "__main__":
    main()
