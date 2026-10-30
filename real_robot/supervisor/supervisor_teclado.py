#!/usr/bin/env python3
"""Supervisor de teclado (fase 1 del puente UDP). Se ejecuta en el host.

    python3 supervisor_teclado.py [--v-normal 0.3] [--v-cautious 0.15]

Teclas: n = NORMAL, c = CAUTIOUS, p = PAUSED, r = RETRACT (sube 5 cm y espera), q = salir.
- Envía el comando a UDP 5005 a ~20 Hz (empieza en PAUSED).
- Recibe y muestra el estado del robot en UDP 5006.
- Guarda todo en logs/supervisor_<fecha>.csv (una fila por tick).
"""
import argparse
import csv
import json
import os
import select
import socket
import sys
import termios
import time
import tty

IP = "127.0.0.1"
PUERTO_CMD = 5005
PUERTO_ESTADO = 5006
HZ = 20.0
TIMEOUT_ROBOT = 0.5         # s sin estado del robot -> se muestra como perdido
HZ_PANTALLA = 5.0           # redibujados por segundo como máximo (el envío UDP sigue a HZ)

TECLAS = {"n": "NORMAL", "c": "CAUTIOUS", "p": "PAUSED", "r": "RETRACT"}
COLOR = {"NORMAL": "\033[42;30m", "CAUTIOUS": "\033[43;30m", "PAUSED": "\033[41;37m",
         "RETRACT": "\033[44;37m"}
RESET = "\033[0m"

COLUMNAS = (["t_host", "tecla", "cmd_state", "cmd_speed_scale", "cmd_enabled", "cmd_retract",
             "rob_edad_s", "rob_t", "rob_state", "rob_segment", "rob_seg_index", "rob_n_segments",
             "rob_moving", "rob_action", "rob_speed_scale", "rob_comm_ok", "rob_done", "rob_error", "rob_event"]
            + ["q%d" % i for i in range(1, 8)] + ["ee_x", "ee_y", "ee_z"])


def color(estado):
    return COLOR.get(estado, "") + " %-8s " % estado + RESET


def main():
    p = argparse.ArgumentParser(description="Supervisor de teclado para controlador_ciclo.py")
    p.add_argument("--v-normal", type=float, default=0.3, help="speed_scale en NORMAL")
    p.add_argument("--v-cautious", type=float, default=0.15, help="speed_scale en CAUTIOUS")
    p.add_argument("--log-dir", default=os.path.abspath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), os.pardir, "logs")))
    args = p.parse_args()
    velocidad = {"NORMAL": args.v_normal, "CAUTIOUS": args.v_cautious, "PAUSED": 0.0, "RETRACT": 0.0}
    if not sys.stdin.isatty():
        sys.exit("[error] Hace falta una terminal para leer el teclado.")

    os.makedirs(args.log_dir, exist_ok=True)
    ruta_csv = os.path.join(args.log_dir, time.strftime("supervisor_%Y%m%d_%H%M%S.csv"))
    f_csv = open(ruta_csv, "w", newline="")
    log = csv.writer(f_csv)
    log.writerow(COLUMNAS)

    sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_estado = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_estado.bind((IP, PUERTO_ESTADO))
    sock_estado.setblocking(False)

    estado = "PAUSED"
    robot = {}
    t_robot = None
    ultimo_evento = None
    texto_prev, t_pantalla, t_flush = "", 0.0, 0.0

    def enviar(est):
        # RETRACT viaja como PAUSED + retract=true (contrato UDP); enabled=true porque se mueve
        cmd = {"t": time.time(), "enabled": est != "PAUSED", "speed_scale": velocidad[est],
               "retract": est == "RETRACT", "state": "PAUSED" if est == "RETRACT" else est}
        sock_cmd.sendto(json.dumps(cmd).encode("utf-8"), (IP, PUERTO_CMD))
        return cmd

    def linea(texto):
        nonlocal texto_prev
        sys.stdout.write("\r\033[K" + texto + "\n")
        texto_prev = ""         # la línea de estado se borró: redibujarla en el próximo tick

    print("Supervisor de teclado. n = NORMAL (%.2f), c = CAUTIOUS (%.2f), p = PAUSED,"
          " r = RETRACT (sube 5 cm y espera), q = salir" % (args.v_normal, args.v_cautious))
    print("Empieza en PAUSED. Log: %s\n" % ruta_csv)

    fd = sys.stdin.fileno()
    config_terminal = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    periodo = 1.0 / HZ
    try:
        while True:
            t_tick = time.time()

            # Teclado
            # (os.read y no sys.stdin.read: el búfer de Python escondía teclas a select)
            tecla = ""
            if select.select([fd], [], [], 0)[0]:
                tecla = os.read(fd, 32).decode("utf-8", "ignore")[-1:].lower()
                if tecla == "q":
                    break
                if tecla in TECLAS:
                    estado = TECLAS[tecla]
                    linea("[tecla %s] comando -> %s" % (tecla, color(estado)))
                elif tecla.strip():
                    linea("[tecla %r] no reconocida (n, c, p, r, q)" % tecla)

            cmd = enviar(estado)

            # Estado del robot (el último que haya llegado)
            while True:
                try:
                    datos, _ = sock_estado.recvfrom(65536)
                except (BlockingIOError, socket.error):
                    break
                try:
                    robot = json.loads(datos.decode("utf-8"))
                    t_robot = time.time()
                except ValueError:
                    pass

            edad = time.time() - t_robot if t_robot is not None else None
            if robot.get("event") and robot.get("event") != ultimo_evento:
                ultimo_evento = robot["event"]
                linea("[robot] " + ultimo_evento)

            # Pantalla
            if edad is None:
                rob_txt = ("\033[41;37m SIN DATOS DEL ROBOT \033[0m "
                           "(¿controlador_ciclo.py corriendo y con Enter pulsado?)")
            elif robot.get("done"):
                rob_txt = "terminado " + (("ERROR: " + robot["error"]) if robot.get("error") else "OK")
            elif edad > TIMEOUT_ROBOT:
                rob_txt = "\033[41;37m ROBOT PERDIDO (%.1f s) \033[0m" % edad
            else:
                rob_txt = "%s %-12s %2d/%-2d %s v=%.2f%s" % (
                    color(robot.get("state", "?")), robot.get("segment", "?"),
                    robot.get("seg_index", 0) + 1, robot.get("n_segments", 0),
                    {"segmento": "MOVIENDO", "retraccion": "SUBIENDO", "soldando": "SOLDANDO"}.get(robot.get("action"), "quieto  "),
                    robot.get("speed_scale", 0.0),
                    "" if robot.get("comm_ok") else " sin-com")
            # Redibujar sólo si cambia y como mucho a HZ_PANTALLA: redibujar a 20 Hz (sobre todo en el
            # terminal de VS Code) gasta CPU que necesita franka_interface sin kernel RT
            texto = "\r\033[KENVIADO %s v=%.2f | ROBOT %s" % (color(estado), velocidad[estado], rob_txt)
            if texto != texto_prev and t_tick - t_pantalla >= 1.0 / HZ_PANTALLA:
                sys.stdout.write(texto)
                sys.stdout.flush()
                texto_prev, t_pantalla = texto, t_tick

            # CSV
            q = robot.get("q") or [""] * 7
            ee = robot.get("ee_xyz") or [""] * 3
            log.writerow([round(t_tick, 3), tecla, cmd["state"], cmd["speed_scale"], cmd["enabled"], cmd["retract"],
                          "" if edad is None else round(edad, 3)]
                         + [robot.get(k, "") for k in ("t", "state", "segment", "seg_index", "n_segments",
                                                       "moving", "action", "speed_scale", "comm_ok", "done",
                                                       "error", "event")]
                         + list(q) + list(ee))
            if t_tick - t_flush >= 1.0:
                f_csv.flush()
                t_flush = t_tick

            time.sleep(max(0.0, periodo - (time.time() - t_tick)))
    except KeyboardInterrupt:
        pass
    finally:
        # Al salir, dejar el robot en PAUSED (y aunque no llegue, el timeout lo para)
        for _ in range(5):
            enviar("PAUSED")
            time.sleep(0.02)
        termios.tcsetattr(fd, termios.TCSADRAIN, config_terminal)
        f_csv.close()
        print("\nSupervisor cerrado (último comando: PAUSED). Log en %s" % ruta_csv)


if __name__ == "__main__":
    main()
