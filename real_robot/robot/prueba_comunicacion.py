#!/usr/bin/env python3
"""Prueba qué diferencia a move_robot.py (funciona) de controlador_ciclo.py (falla con
communication_constraints_violation). Se ejecuta en el contenedor, con el stack arrancado.

    python3 prueba_comunicacion.py          # las 4 pruebas, con Enter antes de cada una
    python3 prueba_comunicacion.py 3        # sólo la prueba 3

Cada prueba gira la muñeca (articulación 7) 20 grados y vuelve, en 5 s por tramo:
  1. goto_joints bloqueante                         (como move_robot.py)
  2. goto_joints(block=False) + espera con is_skill_done()
  3. igual que 2 + get_joints() a 20 Hz               (lo que hace el controlador ahora)
  4. igual que 3 + get_pose() a 20 Hz                 (lo que hacía el controlador antes)
Si una falla, mira en la terminal de franka_interface el control_command_success_rate.
"""
import math
import sys
import time

GIRO = math.radians(20)
DURACION = 5.0
HZ = 20.0
TOL = 0.02                  # rad
LIMITE_Q7 = 2.8973

DESCRIPCION = {
    1: "goto_joints bloqueante (como move_robot.py)",
    2: "goto_joints(block=False) + is_skill_done()",
    3: "block=False + get_joints() a 20 Hz",
    4: "block=False + get_joints() y get_pose() a 20 Hz",
}


def mover(fa, objetivo, modo):
    """Devuelve (error final en rad, segundos que tardó)."""
    t0 = time.time()
    if modo == 1:
        fa.goto_joints(objetivo, duration=DURACION)
    else:
        fa.goto_joints(objetivo, duration=DURACION, block=False)
        while not fa.is_skill_done():
            if modo >= 3:
                fa.get_joints()
            if modo >= 4:
                fa.get_pose()
            time.sleep(1.0 / HZ)
    q = [float(x) for x in fa.get_joints()]
    return max(abs(a - b) for a, b in zip(q, objetivo)), time.time() - t0


def main():
    modos = [int(a) for a in sys.argv[1:]] or [1, 2, 3, 4]
    if any(m not in DESCRIPCION for m in modos):
        sys.exit("[error] pruebas válidas: 1, 2, 3, 4")

    from frankapy import FrankaArm
    fa = FrankaArm()
    q0 = [float(x) for x in fa.get_joints()]
    signo = 1.0 if q0[6] + GIRO < LIMITE_Q7 - 0.1 else -1.0
    q1 = list(q0)
    q1[6] += signo * GIRO
    print("\nSólo gira la muñeca %+.0f grados y vuelve (%.0f s por tramo)." % (math.degrees(signo * GIRO), DURACION))
    print("Comprueba que el bolígrafo no toca nada. Paro de emergencia en la mano.")

    resultados = []
    for modo in modos:
        input("\nPrueba %d: %s. Enter para lanzarla (Ctrl+C para salir)... " % (modo, DESCRIPCION[modo]))
        ok = True
        for nombre, objetivo in (("ida", q1), ("vuelta", q0)):
            try:
                err, dur = mover(fa, objetivo, modo)
            except ValueError as e:
                # frankapy rechaza el destino antes de mover (paredes virtuales): probar el otro sentido
                aviso = ("  [error] %s\n  El brazo está en una postura donde girar la muñeca choca con las"
                         " paredes virtuales de frankapy. Lleva el robot a una postura más alta"
                         " (p. ej. home) y repite." % e)
                if nombre != "ida" or q1[6] == q0[6] - signo * GIRO:
                    sys.exit(aviso)
                print("  frankapy rechazó el giro (%s): pruebo hacia el otro lado" % e)
                q1[6] = q0[6] - signo * GIRO
                objetivo = q1
                try:
                    err, dur = mover(fa, objetivo, modo)
                except ValueError:
                    sys.exit(aviso)
            bien = err < TOL
            print("  %-6s %s (error %.3f rad, %.1f s)" % (nombre, "OK" if bien else "FALLO: no llegó", err, dur))
            if not bien:
                ok = False
                break
        resultados.append((modo, ok))
        if not ok:
            print("  -> Mira la terminal de franka_interface (control_command_success_rate).")
            break

    print("\n=== Resumen ===")
    for modo, ok in resultados:
        print("  %d. %-48s %s" % (modo, DESCRIPCION[modo], "OK" if ok else "FALLO"))


if __name__ == "__main__":
    main()
