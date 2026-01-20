import argparse
import math
import sys
import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - needed for 3D projection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

try:
    import serial
except ImportError:
    serial = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualizza in 3D il vettore magnetico letto via seriale (X, Y, Z, pitch, roll, yaw).",
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Porta seriale (es. COM3 su Windows, /dev/ttyUSB0 su Linux)",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="Baud rate della seriale (default: 115200)",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=100.0,
        help="Intervallo assi +/- in unità raw (LSB) per il grafico (default: 100)",
    )
    return parser.parse_args()


def open_serial(port: str, baudrate: int):
    if serial is None:
        print("Errore: il pacchetto 'pyserial' non è installato. Installa con: pip install pyserial", file=sys.stderr)
        sys.exit(1)

    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=0.05)
        # Piccola attesa per stabilizzare la connessione
        time.sleep(2.0)
        return ser
    except Exception as exc:  # noqa: BLE001
        print(f"Impossibile aprire la porta seriale {port}: {exc}", file=sys.stderr)
        sys.exit(1)


def parse_line(line: str):
    """Parsa una riga CSV "X, Y, Z, pitch, roll, yaw".

    Ritorna (x, y, z, pitch_deg, roll_deg, yaw_deg), dove X/Y/Z sono interi (LSB)
    e gli angoli sono float in gradi. Ritorna None in caso di parsing fallito.
    """
    # Rimuove eventuali whitespace e separa per virgola
    parts = [p.strip() for p in line.strip().split(",")]

    if len(parts) < 3:
        return None

    try:
        x = int(parts[0])
        y = int(parts[1])
        z = int(parts[2])
    except ValueError:
        return None

    pitch = roll = yaw = 0.0
    if len(parts) >= 6:
        try:
            pitch = float(parts[3])
            roll = float(parts[4])
            yaw = float(parts[5])
        except ValueError:
            pitch = roll = yaw = 0.0

    return x, y, z, pitch, roll, yaw


def main() -> None:
    args = parse_args()

    ser = open_serial(args.port, args.baudrate)

    plt.ion()
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="3d")

    max_range = args.max_range

    # Setup iniziale assi (eseguito una sola volta)
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-max_range, max_range)
    ax.set_xlabel("X [raw LSB]")
    ax.set_ylabel("Y [raw LSB]")
    ax.set_zlabel("Z [raw LSB]")
    ax.set_title("Vettore magnetico TLx493D")
    # Disegna gli assi di riferimento globali
    ax.quiver(0, 0, 0, max_range, 0, 0, color="r", arrow_length_ratio=0.1)
    ax.quiver(0, 0, 0, 0, max_range, 0, color="g", arrow_length_ratio=0.1)
    ax.quiver(0, 0, 0, 0, 0, max_range, color="b", arrow_length_ratio=0.1)

    # Elementi grafici aggiornati a ogni campione
    vector_quiver = None           # vettore magnetico
    pyramid_poly = None            # piramide triangolare 3D
    text_info = None               # testo con valori correnti

    print("In ascolto sulla seriale... (Ctrl+C per uscire)")

    try:
        while plt.fignum_exists(fig.number):
            try:
                # Legge una riga e poi svuota il buffer per tenere solo il campione più recente
                raw = ser.readline()
                if ser.in_waiting:
                    while ser.in_waiting:
                        raw = ser.readline()
            except Exception:
                # in caso di problemi temporanei di lettura, riprova
                continue

            if not raw:
                # nessun dato, prova ancora
                plt.pause(0.01)
                continue

            try:
                line = raw.decode(errors="ignore").strip()
            except Exception:
                continue

            if not line:
                plt.pause(0.01)
                continue

            parsed = parse_line(line)
            if parsed is None:
                # linea non valida, salta
                continue

            x, y, z, pitch_deg, roll_deg, yaw_deg = parsed

            # Aggiorna solo il vettore magnetico in magenta
            if vector_quiver is not None:
                vector_quiver.remove()

            vector_quiver = ax.quiver(
                0,
                0,
                0,
                x,
                y,
                z,
                color="m",
                arrow_length_ratio=0.1,
            )

            # Calcola orientamento dell'oggetto dai tre angoli (in radianti)
            yaw_rad = math.radians(yaw_deg)
            pitch_rad = math.radians(pitch_deg)
            roll_rad = math.radians(roll_deg)

            cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
            cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
            cr, sr = math.cos(roll_rad), math.sin(roll_rad)

            # Matrice di rotazione Z-Y-X (yaw-pitch-roll)
            R00 = cy * cp
            R01 = cy * sp * sr - sy * cr
            R02 = cy * sp * cr + sy * sr
            R10 = sy * cp
            R11 = sy * sp * sr + cy * cr
            R12 = sy * sp * cr - cy * sr
            R20 = -sp
            R21 = cp * sr
            R22 = cp * cr

            axis_len = 0.7 * max_range

            # Piramide triangolare 3D che ruota con l'orientamento
            # Definisce i vertici nel frame corpo (piramide centrata all'origine)
            base = 0.4 * max_range
            height = 0.6 * max_range

            v_apex = (0.0, 0.0, height)
            v_b1 = (base, 0.0, -height)
            v_b2 = (-0.5 * base, 0.5 * math.sqrt(3.0) * base, -height)
            v_b3 = (-0.5 * base, -0.5 * math.sqrt(3.0) * base, -height)

            def rotate_vertex(v):
                vx, vy, vz = v
                gx = R00 * vx + R01 * vy + R02 * vz
                gy = R10 * vx + R11 * vy + R12 * vz
                gz = R20 * vx + R21 * vy + R22 * vz
                return (gx, gy, gz)

            V_apex = rotate_vertex(v_apex)
            V_b1 = rotate_vertex(v_b1)
            V_b2 = rotate_vertex(v_b2)
            V_b3 = rotate_vertex(v_b3)

            faces = [
                [V_apex, V_b1, V_b2],  # lato 1
                [V_apex, V_b2, V_b3],  # lato 2
                [V_apex, V_b3, V_b1],  # lato 3
                [V_b1, V_b2, V_b3],    # base
            ]

            # Colori: lati azzurri semitrasparenti, base arancione semitrasparente
            side_color = (0.3, 0.7, 1.0, 0.4)
            base_color = (1.0, 0.6, 0.0, 0.6)
            face_colors = [side_color, side_color, side_color, base_color]

            if pyramid_poly is not None:
                pyramid_poly.remove()

            pyramid_poly = Poly3DCollection(
                faces,
                facecolors=face_colors,
                edgecolors="k",
                linewidths=0.5,
            )
            ax.add_collection3d(pyramid_poly)

            # Testo con valori correnti (vettore + angoli)
            if text_info is not None:
                text_info.remove()
            text_info = ax.text2D(
                0.05,
                0.95,
                f"X={x} LSB, Y={y} LSB, Z={z} LSB  |  pitch={pitch_deg:.1f}°, roll={roll_deg:.1f}°, yaw={yaw_deg:.1f}°",
                transform=ax.transAxes,
            )

            plt.draw()
            plt.pause(0.005)
    except KeyboardInterrupt:
        print("\nTerminazione richiesta dall'utente.")
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
