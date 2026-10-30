#!/usr/bin/env python3
"""Ciclo de "soldadura" por segmentos controlado por UDP (fase 1 del puente).

Se ejecuta en el contenedor con frankapy:
    python3 controlador_ciclo.py                # ciclo aproximacion/soldadura/retraccion
    python3 controlador_ciclo.py --placa        # rejilla de puntos sobre la placa (plan_placa.py)
    python3 controlador_ciclo.py --simulado     # sin robot (brazo falso, para la laptop)

- Escucha comandos del supervisor en UDP 5005 (JSON):
    {"t": .., "enabled": true, "speed_scale": 0.3, "retract": false, "state": "NORMAL"}
- Envía el estado del robot a UDP 5006 a ~20 Hz.
- PAUSED, enabled=false o >0.5 s sin comandos válidos -> stop_skill() y quieto.
- retract=true (con enabled=true) -> detener el segmento, subir la herramienta 5 cm en
  vertical, despacio y a velocidad fija, y quedarse quieto. Sin comunicación nunca se retrae.
- Al volver a NORMAL/CAUTIOUS se reanuda el segmento actual desde la posición actual.
- speed_scale se fija al empezar cada segmento (o al reanudarlo tras una pausa).
- Si se para durante la espera de un punto ("soldando"), ese punto se da por terminado.
"""
import argparse
import json
import math
import os
import socket
import sys
import time

import plan_placa

IP = "127.0.0.1"            # el contenedor usa --net=host
PUERTO_CMD = 5005
PUERTO_ESTADO = 5006
HZ = 20.0
TIMEOUT_CMD = 0.5           # s sin comandos -> PAUSED (fail-safe)

# Duración de cada segmento según lo que se mueve (la punta y las articulaciones), no fija:
#   duración = 1.875 * max(distancia_tcp / V_TCP_MAX, max|dq| / VEL_PICO_MAX) / speed_scale
# speed_scale multiplica todas las velocidades, así CAUTIOUS (0.3) tarda el doble que NORMAL (0.6).
V_TCP_MAX = 0.25            # m/s, velocidad pico de la herramienta con speed_scale = 1.0; --vel-tcp-max
V_TCP_TECHO = 0.5           # m/s, nunca permitir más por línea de comandos
DURACION_MIN = 0.5          # s, nunca mandar movimientos más cortos
ESCALA_MIN = 0.05           # evita duraciones absurdas
VEL_PICO_MAX = 1.0          # rad/s, velocidad articular pico con speed_scale = 1.0; --vel-pico-max
VEL_PICO_TECHO = 1.0        # rad/s, nunca permitir más por línea de comandos
FACTOR_PICO = 1.875         # pico / media en una trayectoria min-jerk
TOL_LLEGADA = 0.02          # rad, error máximo aceptado al terminar un segmento
RETRACCION_M = 0.05         # m que sube la herramienta al retraerse
RETRACCION_DUR = 2.5        # s, velocidad fija y lenta (pico ~3.8 cm/s); --retraccion-dur
RETRACCION_DUR_MIN = 2.0    # s, la retracción nunca más rápida que esto

SECUENCIA = ["aproximacion", "soldadura", "retraccion"]
ESTADOS = ("NORMAL", "CAUTIOUS", "PAUSED")

# Sólo para --simulado si no existe poses.json (valores de zones.yaml del repo)
POSES_DEMO = {
    "home": [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398],
    "aproximacion": [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.5],
    "soldadura": [0.05, -0.45, 0.0, -1.95, 0.0, 1.45, 0.45],
    "retraccion": [0.0, -0.6, 0.0, -2.1, 0.0, 1.5, 0.6],
}


class Pose(object):
    def __init__(self, translation):
        self.translation = translation


class BrazoSimulado(object):
    """Imita la parte de FrankaArm que usamos, con interpolación min-jerk."""

    def __init__(self, q0):
        self._q = list(q0)
        self._q_ini = None
        self._obj = None
        self._t0 = 0.0
        self._dur = 1.0

    def _actualizar(self):
        if self._obj is None:
            return
        s = min(1.0, (time.time() - self._t0) / self._dur)
        a = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        self._q = [qi + a * (qf - qi) for qi, qf in zip(self._q_ini, self._obj)]
        if s >= 1.0:
            self._obj = None

    def goto_joints(self, joints, duration=5, block=True, **kwargs):
        # Misma comprobación que frankapy
        assert type(duration) in (int, float), "Incorrect time type. Should be int or float."
        self._actualizar()
        self._q_ini = list(self._q)
        self._obj = list(joints)
        self._t0 = time.time()
        self._dur = float(duration)
        while block and not self.is_skill_done():
            time.sleep(0.01)

    def goto_pose(self, pose, duration=3, block=True, **kwargs):
        # Inversa de la posición ficticia de get_pose(): z depende sólo de q[1]
        q = self.get_joints()
        q[1] += (pose.translation[2] - self.get_pose().translation[2]) / 0.2
        self.goto_joints(q, duration=duration, block=block)

    def stop_skill(self):
        self._actualizar()
        self._obj = None

    def is_skill_done(self):
        self._actualizar()
        return self._obj is None

    def get_joints(self):
        self._actualizar()
        return list(self._q)

    def get_pose(self):
        # Posición ficticia, sólo para tener números que cambien
        q = self.get_joints()
        r = 0.3 + 0.1 * (q[3] + 2.4)
        return Pose([r * math.cos(q[0]), r * math.sin(q[0]), 0.3 + 0.2 * (q[1] + 0.8)])


def dist_max(a, b):
    """Mayor diferencia articular (rad) entre dos configuraciones."""
    return max(abs(x - y) for x, y in zip(a, b))


def calcular_duracion(q, objetivo, escala, v_tcp, vel_max):
    """Duración según lo que falta por recorrer (sirve igual al reanudar desde una pausa)."""
    resto_q = dist_max(q, objetivo)
    resto_tcp = math.sqrt(sum((a - b) ** 2 for a, b in zip(plan_placa.fk(q, 0.0)[0],
                                                           plan_placa.fk(objetivo, 0.0)[0])))
    dur = FACTOR_PICO * max(resto_tcp / v_tcp, resto_q / vel_max) / escala
    # float() explícito: frankapy exige float/int de Python, no numpy
    return float(max(DURACION_MIN, dur))


def cargar_poses(ruta, simulado):
    if not os.path.exists(ruta):
        if simulado:
            print("[aviso] No existe %s: uso poses de ejemplo (sólo simulado)." % ruta)
            return dict(POSES_DEMO)
        sys.exit("[error] No existe %s. Graba las poses con grabar_poses.py." % ruta)
    with open(ruta) as f:
        poses = json.load(f)
    for nombre in ["home"] + SECUENCIA:
        if nombre not in poses or len(poses[nombre]) != 7:
            sys.exit("[error] %s: falta la pose '%s' (7 valores)." % (ruta, nombre))
    return {k: [float(x) for x in v] for k, v in poses.items()}


def plan_ciclo(poses, ciclos):
    nombres = SECUENCIA * ciclos + ["home"]
    return [{"nombre": n, "q": poses[n], "espera": 0.0, "factor": 1.0} for n in nombres]


def parsear_comando(datos):
    """Devuelve el comando validado o None si no es válido (se ignora)."""
    try:
        cmd = json.loads(datos.decode("utf-8"))
        estado = cmd["state"]
        escala = float(cmd["speed_scale"])
        habilitado = bool(cmd.get("enabled", True))
    except (ValueError, KeyError, TypeError, AttributeError):
        return None
    if estado not in ESTADOS or math.isnan(escala) or not 0.0 <= escala <= 1.0:
        return None
    return {"state": estado, "speed_scale": escala, "enabled": habilitado,
            "retract": bool(cmd.get("retract", False))}


def main():
    p = argparse.ArgumentParser(description="Ciclo de soldadura controlado por UDP")
    p.add_argument("--simulado", action="store_true", help="sin robot: brazo falso")
    p.add_argument("--poses", default=os.path.join(plan_placa.DIR_DATOS, "poses.json"))
    p.add_argument("--ciclos", type=int, default=3)
    p.add_argument("--placa", action="store_true", help="rutina de puntos sobre la placa en vez del ciclo")
    p.add_argument("--vel-tcp-max", type=float, default=V_TCP_MAX,
                   help="m/s, velocidad pico de la herramienta con speed_scale 1.0 (por defecto %.2f)" % V_TCP_MAX)
    p.add_argument("--vel-pico-max", type=float, default=VEL_PICO_MAX,
                   help="rad/s, velocidad articular pico con speed_scale 1.0 (por defecto %.1f)" % VEL_PICO_MAX)
    p.add_argument("--retraccion-dur", type=float, default=RETRACCION_DUR,
                   help="s que tarda en subir %.0f cm (por defecto %.1f)" % (RETRACCION_M * 100, RETRACCION_DUR))
    p.add_argument("--escala-max", type=float, default=0.3,
                   help="tope de speed_scale (por defecto 0.3, primera prueba)")
    plan_placa.agregar_argumentos(p)
    args = p.parse_args()
    if not 0.0 < args.escala_max <= 1.0:
        sys.exit("[error] --escala-max debe estar en (0, 1].")
    if not 0.0 < args.vel_tcp_max <= V_TCP_TECHO:
        sys.exit("[error] --vel-tcp-max debe estar en (0, %.2f] m/s." % V_TCP_TECHO)
    if not 0.0 < args.vel_pico_max <= VEL_PICO_TECHO:
        sys.exit("[error] --vel-pico-max debe estar en (0, %.1f] rad/s." % VEL_PICO_TECHO)
    if args.retraccion_dur < RETRACCION_DUR_MIN:
        sys.exit("[error] --retraccion-dur debe ser >= %.1f s." % RETRACCION_DUR_MIN)

    poses = cargar_poses(args.poses, args.simulado)
    if args.placa:
        try:
            _, info = plan_placa.cargar(os.path.dirname(os.path.abspath(args.poses)))
            plan, informe = plan_placa.plan_desde_args(args, poses, info)
        except (IOError, ValueError, KeyError) as e:
            sys.exit("[error] plan de placa no válido: %s" % e)
    else:
        plan, informe = plan_ciclo(poses, args.ciclos), [
            "Plan: %d ciclos %s y vuelta a home." % (args.ciclos, " -> ".join(SECUENCIA))]

    sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_cmd.bind((IP, PUERTO_CMD))
    sock_cmd.setblocking(False)
    sock_estado = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destino = (IP, PUERTO_ESTADO)

    if args.simulado:
        fa = BrazoSimulado(poses["home"])
    else:
        from frankapy import FrankaArm
        fa = FrankaArm()

    print("\n=== controlador_ciclo (%s) ===" % ("SIMULADO" if args.simulado else "ROBOT REAL"))
    print("\n".join(informe))
    print("Total: %d segmentos." % len(plan))
    print("Tope speed_scale: %.2f | vel. pico <= %.2f rad/s | duración mín. %.1f s"
          % (args.escala_max, args.vel_pico_max * args.escala_max, DURACION_MIN))
    print("Velocidad pico con speed_scale 1.0: herramienta %.2f m/s, articulaciones %.1f rad/s"
          % (args.vel_tcp_max, args.vel_pico_max))
    print("Retracción: sube %.0f cm en %.1f s" % (RETRACCION_M * 100, args.retraccion_dur))
    print("Primer segmento: desde la posición actual hasta '%s' (dist. máx. %.2f rad)."
          % (plan[0]["nombre"], dist_max(fa.get_joints(), plan[0]["q"])))
    print("El robot sólo se mueve si el supervisor envía NORMAL/CAUTIOUS.")
    input("Paro de emergencia en la mano. Pulsa Enter para empezar (Ctrl+C para salir)... ")

    idx = 0                  # segmento actual dentro de plan
    q_inicio_seg = None      # None = el segmento aún no ha empezado
    escala_seg = 0.0
    accion = None            # None, "segmento", "retraccion" o "soldando" (espera en un punto)
    t_fin_espera = 0.0
    retirado = False         # ya se hizo la retracción de esta parada
    avisado_tope = False     # ya se avisó de que el supervisor pide más que --escala-max
    ultimo_cmd = None
    t_ultimo_cmd = None
    efectivo_prev = None
    evento = "esperando supervisor"
    error = ""
    print("[evento] " + evento)

    def avisar(texto):
        print("[evento] " + texto)
        return texto

    periodo = 1.0 / HZ
    try:
        while (idx < len(plan) or accion == "soldando") and not error:
            t_tick = time.time()

            # 1) Leer todos los comandos pendientes, quedarse con el último válido
            while True:
                try:
                    datos, _ = sock_cmd.recvfrom(4096)
                except (BlockingIOError, socket.error):
                    break
                cmd = parsear_comando(datos)
                if cmd is not None:
                    ultimo_cmd, t_ultimo_cmd = cmd, time.time()

            # 2) Estado efectivo (fail-safe: ante la duda, PAUSED)
            comm_ok = t_ultimo_cmd is not None and time.time() - t_ultimo_cmd <= TIMEOUT_CMD
            if not comm_ok:
                efectivo, motivo = "PAUSED", "sin comunicacion"
            elif not ultimo_cmd["enabled"]:
                efectivo, motivo = "PAUSED", "movimiento deshabilitado"
            elif ultimo_cmd["retract"]:
                efectivo, motivo = "RETRACT", "supervisor"
            elif ultimo_cmd["speed_scale"] <= 0.0:
                efectivo, motivo = "PAUSED", "movimiento deshabilitado"
            else:
                efectivo, motivo = ultimo_cmd["state"], "supervisor"
            if efectivo != efectivo_prev:
                evento = avisar("estado %s (%s)" % (efectivo, motivo))
                efectivo_prev = efectivo

            # 3) Actuar
            if efectivo in ("PAUSED", "RETRACT"):
                if accion == "segmento":
                    fa.stop_skill()
                    accion = None
                    evento = avisar("pausa: stop_skill() en '%s'" % plan[idx]["nombre"])
                elif accion == "soldando":
                    accion = None
                    evento = avisar("pausa: punto '%s' interrumpido" % plan[idx - 1]["nombre"])
                elif accion == "retraccion" and efectivo == "PAUSED":
                    fa.stop_skill()
                    accion = None
                    evento = avisar("pausa: retracción detenida")

            if efectivo == "RETRACT":
                if accion == "retraccion" and fa.is_skill_done():
                    accion = None
                    retirado = True
                    evento = avisar("retracción completada: quieto")
                elif accion is None and not retirado:
                    T = fa.get_pose()
                    t = [float(x) for x in T.translation]
                    t[2] += RETRACCION_M
                    T.translation = t
                    fa.goto_pose(T, duration=float(args.retraccion_dur), block=False)
                    accion = "retraccion"
                    evento = avisar("retracción: subo %.0f cm en %.1f s" % (RETRACCION_M * 100, args.retraccion_dur))
            elif efectivo != "PAUSED":
                if accion == "retraccion":
                    fa.stop_skill()
                    accion = None
                retirado = False
                if accion == "soldando" and time.time() >= t_fin_espera:
                    accion = None
                    evento = avisar("punto '%s' terminado" % plan[idx - 1]["nombre"])
                if accion == "segmento" and fa.is_skill_done():
                    accion = None
                    seg = plan[idx]
                    q_fin = fa.get_joints()
                    err = dist_max(q_fin, seg["q"])
                    if err > TOL_LLEGADA:
                        texto = "ERROR: '%s' terminó a %.3f rad del objetivo" % (seg["nombre"], err)
                        if dist_max(q_fin, q_inicio_seg) < TOL_LLEGADA:
                            # frankapy no avisa (ignore_errors=True): el robot rechazó el movimiento
                            texto += (" -- el robot NO se movió: mira errores en la terminal de"
                                      " franka_interface y las luces/Desk (paro pulsado o error de reflejo)")
                        error = evento = avisar(texto)
                        break
                    evento = avisar("segmento '%s' completado (%d/%d)" % (seg["nombre"], idx + 1, len(plan)))
                    idx += 1
                    q_inicio_seg = None
                    if seg["espera"] > 0.0:
                        accion = "soldando"
                        t_fin_espera = time.time() + seg["espera"]
                        evento = avisar("soldando en '%s' (%.1f s)" % (seg["nombre"], seg["espera"]))

                if accion is None and idx < len(plan):
                    seg = plan[idx]
                    q = fa.get_joints()
                    reanuda = q_inicio_seg is not None
                    if not reanuda:
                        q_inicio_seg = q
                    if dist_max(q, seg["q"]) < TOL_LLEGADA:
                        # Ya estamos ahí (p. ej. pausa justo al final): pasar al siguiente
                        idx += 1
                        q_inicio_seg = None
                    else:
                        pedida = ultimo_cmd["speed_scale"]
                        escala_seg = min(max(pedida, ESCALA_MIN), args.escala_max)
                        dur = seg["factor"] * calcular_duracion(q, seg["q"], escala_seg,
                                                                args.vel_tcp_max, args.vel_pico_max)
                        if pedida > args.escala_max + 1e-9 and not avisado_tope:
                            avisado_tope = True
                            avisar("[aviso] el supervisor pide escala %.2f pero el tope es %.2f:"
                                   " lanza con --escala-max %.2f" % (pedida, args.escala_max, pedida))
                        fa.goto_joints([float(x) for x in seg["q"]], duration=float(dur), block=False)
                        accion = "segmento"
                        evento = avisar("%s '%s' (%d/%d) escala %.2f, %.1f s"
                                        % ("reanudo" if reanuda else "inicio", seg["nombre"], idx + 1,
                                           len(plan), escala_seg, dur))

            # 4) Enviar estado del robot. Una sola consulta a frankapy por ciclo: cada consulta
            # es un servicio ROS y, sin kernel RT, la carga extra provoca
            # communication_constraints_violation. La posición del TCP sale de la FK.
            q = [float(x) for x in fa.get_joints()]
            estado = {
                "t": time.time(),
                "q": [round(x, 5) for x in q],
                "ee_xyz": [round(x, 4) for x in plan_placa.fk(q, 0.0)[0]],
                "segment": plan[idx]["nombre"] if idx < len(plan) else "fin",
                "seg_index": idx,
                "n_segments": len(plan),
                "moving": accion in ("segmento", "retraccion"),
                "action": accion or "",
                "state": efectivo,
                "speed_scale": escala_seg,
                "comm_ok": comm_ok,
                "event": evento,
                "done": False,
                "error": "",
            }
            sock_estado.sendto(json.dumps(estado).encode("utf-8"), destino)

            time.sleep(max(0.0, periodo - (time.time() - t_tick)))
    except KeyboardInterrupt:
        evento = avisar("Ctrl+C: parando")
    except Exception as e:  # goto rechazado (límites, paredes virtuales...), fallo de ROS, etc.
        error = evento = avisar("ERROR: %r" % (e,))
    finally:
        if accion in ("segmento", "retraccion"):
            try:
                fa.stop_skill()
            except Exception as e:
                print("[error] stop_skill() falló: %r -> usa el paro de emergencia" % (e,))
        if not error and idx >= len(plan):
            evento = avisar("rutina terminada")
        final = {"t": time.time(), "q": [], "ee_xyz": [], "segment": "fin", "seg_index": idx,
                 "n_segments": len(plan), "moving": False, "action": "", "state": "PAUSED",
                 "speed_scale": 0.0, "comm_ok": False, "event": evento, "done": True, "error": error}
        for _ in range(5):
            sock_estado.sendto(json.dumps(final).encode("utf-8"), destino)
            time.sleep(0.05)
        sock_cmd.close()
        sock_estado.close()


if __name__ == "__main__":
    main()
