#!/usr/bin/env python3
"""Graba las poses del ciclo guiando el brazo a mano. Se ejecuta en el contenedor.

    python3 grabar_poses.py                          # las 4 poses en orden
    python3 grabar_poses.py soldadura                # regrabar sólo una (o varias)
    python3 grabar_poses.py --largo-boligrafo 4.5    # cm de bolígrafo bajo el TCP de frankapy
    python3 grabar_poses.py --esquinas --largo-boligrafo 1.5   # las 4 esquinas de la placa

Para cada pose: Enter -> modo guía (brazo libre) -> llevas el brazo -> Enter -> se fija
y se muestra la posición; confirmas y se guarda en poses.json al momento.
Además guarda posición/orientación en poses_info.json y la comprueba contra mesa.json.
"""
import argparse
import json
import os
import shutil
import sys
import time

DIR_DATOS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "datos"))
ORDEN = ["home", "aproximacion", "soldadura", "retraccion"]
ESQUINAS = ["esquina_1", "esquina_2", "esquina_3", "esquina_4"]
DESCRIPCION = {
    "home": "posición segura: brazo recogido y alto, bolígrafo lejos de la mesa y del operador",
    "aproximacion": "bolígrafo vertical, punta ~8 cm justo ENCIMA del punto de soldadura",
    "soldadura": "punta del bolígrafo ~5 mm sobre el cartón (sin tocarlo), misma orientación",
    "retraccion": "subir ~15 cm desde soldadura y un poco hacia el robot (lejos del operador)",
    # Esquinas en orden alrededor de la placa (x hacia el operador, y a la izquierda del robot)
    "esquina_1": "punta TOCANDO APENAS la esquina más cercana al robot, a la DERECHA del robot",
    "esquina_2": "punta TOCANDO APENAS la esquina más cercana al robot, a la IZQUIERDA del robot",
    "esquina_3": "punta TOCANDO APENAS la esquina más lejana del robot, a la IZQUIERDA del robot",
    "esquina_4": "punta TOCANDO APENAS la esquina más lejana del robot, a la DERECHA del robot",
}
DURACION_GUIA = 120.0       # s máximo en modo guía por pose


def dentro_poligono(x, y, puntos):
    """Ray casting: ¿(x, y) está dentro del polígono?"""
    dentro = False
    for i in range(len(puntos)):
        (x1, y1), (x2, y2) = puntos[i], puntos[i - 1]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            dentro = not dentro
    return dentro


def cargar_json(ruta, defecto):
    if os.path.exists(ruta):
        with open(ruta) as f:
            return json.load(f)
    return defecto


def guardar_json(ruta, datos):
    tmp = ruta + ".tmp"
    with open(tmp, "w") as f:
        json.dump(datos, f, indent=2)
    os.replace(tmp, ruta)


def punta_boligrafo(T, largo_m):
    """Punta = TCP de frankapy + largo a lo largo del eje z de la herramienta."""
    R = T.rotation
    return [float(T.translation[i] + R[i][2] * largo_m) for i in range(3)]


def revisar(nombre, punta, info, mesa):
    """Avisos de sentido común (no bloquean)."""
    avisos = []
    contorno = [mesa["arucos"][k] for k in mesa["orden_contorno"]]
    altura = punta[2] - mesa["z_mesa"]
    if altura < 0.0:
        avisos.append("la punta está %.1f cm POR DEBAJO de la mesa: ¿largo del bolígrafo correcto?" % (-altura * 100))
    if nombre == "soldadura" or nombre in ESQUINAS:
        if not dentro_poligono(punta[0], punta[1], contorno):
            avisos.append("el punto queda FUERA del área de los ArUco")
    if nombre in ESQUINAS and altura > 0.015:
        avisos.append("la punta está a %.1f cm de la mesa: ¿toca el cartón?" % (altura * 100))
        if altura > 0.03:
            avisos.append("la punta está a %.1f cm de la mesa (¿demasiado alta?)" % (altura * 100))
    if nombre == "aproximacion" and "soldadura" in info:
        s = info["soldadura"]["punta_xyz"]
        dxy = ((punta[0] - s[0]) ** 2 + (punta[1] - s[1]) ** 2) ** 0.5
        if dxy > 0.01:
            avisos.append("aproximación desplazada %.1f cm en XY respecto a soldadura" % (dxy * 100))
    if nombre == "soldadura" and "aproximacion" in info:
        a = info["aproximacion"]["punta_xyz"]
        dxy = ((punta[0] - a[0]) ** 2 + (punta[1] - a[1]) ** 2) ** 0.5
        if dxy > 0.01:
            avisos.append("soldadura desplazada %.1f cm en XY respecto a aproximación" % (dxy * 100))
    return avisos


def resumen_esquinas(info):
    """Tamaño, desnivel y forma de la placa a partir de las 4 esquinas."""
    if not all(n in info for n in ESQUINAS):
        return
    pts = [info[n]["punta_xyz"] for n in ESQUINAS]
    print("\n=== Placa (punta del bolígrafo en cada esquina) ===")
    giros = []
    for i in range(4):
        a, b, c = pts[i], pts[(i + 1) % 4], pts[(i + 2) % 4]
        lado = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
        print("  lado %d-%d: %.1f cm" % (i + 1, (i + 1) % 4 + 1, lado * 100))
        giros.append((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]))
    zs = [p[2] for p in pts]
    print("  centro: x=%.3f y=%.3f m | altura de la superficie: %.1f a %.1f cm"
          % (sum(p[0] for p in pts) / 4, sum(p[1] for p in pts) / 4, min(zs) * 100, max(zs) * 100))
    if not (all(g > 0 for g in giros) or all(g < 0 for g in giros)):
        print("  [aviso] las esquinas no forman un cuadrilátero convexo en ese orden: revisa cuál es cuál")
    if max(zs) - min(zs) > 0.01:
        print("  [aviso] más de 1 cm de desnivel entre esquinas: ¿placa inclinada o alguna sin tocar?")


def main():
    p = argparse.ArgumentParser(description="Graba poses del ciclo guiando el brazo a mano")
    p.add_argument("poses", nargs="*", help="poses a grabar (por defecto las 4 del ciclo)")
    p.add_argument("--esquinas", action="store_true", help="grabar las 4 esquinas de la placa")
    p.add_argument("--largo-boligrafo", type=float, default=0.0,
                   help="cm que sobresale el bolígrafo por debajo del TCP de frankapy (punto entre los dedos)")
    args = p.parse_args()
    if not args.poses:
        args.poses = ESQUINAS if args.esquinas else ORDEN
    for nombre in args.poses:
        if nombre not in ORDEN + ESQUINAS:
            sys.exit("[error] pose desconocida '%s'. Válidas: %s" % (nombre, ", ".join(ORDEN + ESQUINAS)))

    ruta_poses = os.path.join(DIR_DATOS, "poses.json")
    ruta_info = os.path.join(DIR_DATOS, "poses_info.json")
    mesa = cargar_json(os.path.join(DIR_DATOS, "mesa.json"), None)
    if mesa is None:
        sys.exit("[error] falta %s" % os.path.join(DIR_DATOS, "mesa.json"))
    poses = cargar_json(ruta_poses, {})
    info = cargar_json(ruta_info, {})
    if os.path.exists(ruta_poses):
        copia = ruta_poses + time.strftime(".bak-%Y%m%d_%H%M%S")
        shutil.copy(ruta_poses, copia)
        print("Copia de seguridad: %s" % copia)

    from frankapy import FrankaArm
    fa = FrankaArm()
    largo = args.largo_boligrafo / 100.0

    print("\n=== Grabar poses: %s ===" % ", ".join(args.poses))
    print("Largo del bolígrafo bajo el TCP: %.1f cm" % args.largo_boligrafo)
    print("Sujeta el brazo con una mano y ten el paro de emergencia a mano.")

    for nombre in args.poses:
        while True:
            print("\n--- %s: %s" % (nombre.upper(), DESCRIPCION[nombre]))
            r = input("Enter = activar modo guía | s = saltar esta pose: ").strip().lower()
            if r == "s":
                break
            fa.run_guide_mode(duration=DURACION_GUIA, block=False)
            print("Modo guía activo (máx. %.0f s). Lleva el brazo a la pose." % DURACION_GUIA)
            input("SUJETA el brazo y pulsa Enter para fijarlo... ")
            fa.stop_skill()
            time.sleep(0.5)

            q = [round(float(x), 5) for x in fa.get_joints()]
            T = fa.get_pose()
            punta = punta_boligrafo(T, largo)
            print("q (rad):         %s" % q)
            print("TCP frankapy (m): x=%.3f y=%.3f z=%.3f" % tuple(T.translation))
            print("Punta (m):        x=%.3f y=%.3f z=%.3f  -> %.1f cm sobre la mesa"
                  % (punta[0], punta[1], punta[2], (punta[2] - mesa["z_mesa"]) * 100))
            for aviso in revisar(nombre, punta, info, mesa):
                print("  [aviso] " + aviso)

            r = input("Enter = guardar | r = repetir | s = saltar: ").strip().lower()
            if r == "r":
                continue
            if r == "s":
                break
            poses[nombre] = q
            info[nombre] = {
                "tcp_xyz": [round(float(v), 4) for v in T.translation],
                "tcp_quat_wxyz": [round(float(v), 5) for v in T.quaternion],
                "punta_xyz": [round(v, 4) for v in punta],
                "largo_boligrafo_cm": args.largo_boligrafo,
                "fecha": time.strftime("%Y-%m-%d %H:%M:%S"),
            }
            guardar_json(ruta_poses, poses)
            guardar_json(ruta_info, info)
            print("Guardada '%s' en %s" % (nombre, ruta_poses))
            break

    faltan = [n for n in ORDEN if n not in poses]
    print("\nListo. %s" % ("Faltan: " + ", ".join(faltan) if faltan else "poses del ciclo completas."))
    resumen_esquinas(info)


if __name__ == "__main__":
    main()
