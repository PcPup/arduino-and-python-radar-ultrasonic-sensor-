#!/usr/bin/env python3
"""
Arduino Radar Visualizer (pygame)

- Reads CSV lines from Arduino over serial in the format:
    time_ms,angle_deg,distance_cm
  (This matches the output of the example Arduino sketches provided earlier.)

- Visualizes readings in real time using pygame:
  - A radar-style sweep (angle + distance)
  - Points colored on a gradient from GREEN (far) to RED (close)
  - "out" or invalid readings are shown in blue/gray
  - Shows current numeric distance and connection status

Usage (from VS Code terminal or command line):
    python arduino_radar_visualizer.py --port COM5 --baud 9600

Dependencies:
    pip install pyserial pygame

Notes:
- If your Arduino prints a different CSV format, adjust parse_line().
- Adjust --min-distance and --max-distance to match your HC-SR04 environment/range.
- On Linux use a device like /dev/ttyACM0 or /dev/ttyUSB0 for --port.
"""

import argparse
import math
import threading
import time
from collections import deque

import pygame

import serial
import serial.tools.list_ports

# ---------------------------------------
# Configurable defaults
# ---------------------------------------
DEFAULT_BAUD = 9600
DEFAULT_PORT = None  # if None, the script will try to auto-detect
READ_HISTORY = 200   # how many recent points to keep and render
RECONNECT_DELAY = 2.0  # seconds between reconnect attempts
# Distance mapping (cm)
DEFAULT_MIN_DISTANCE = 5    # distance that maps to RED (very close)
DEFAULT_MAX_DISTANCE = 400  # distance that maps to GREEN (far)
# Visual layout
WIDTH, HEIGHT = 900, 700
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT - 60
MAX_RADIUS_PX = min(WIDTH // 2 - 40, HEIGHT - 140)  # max radar radius in pixels

# ---------------------------------------
# Serial reader thread
# ---------------------------------------
class SerialReader(threading.Thread):
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self._stop = threading.Event()
        self.serial = None
        self.connected = False
        self.last_error = ""
        self.queue = deque(maxlen=READ_HISTORY)  # store tuples (timestamp_ms, angle_deg, distance_cm_or_None)

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            if not self.connected:
                self._attempt_connect()
            if self.connected:
                try:
                    line = self.serial.readline().decode(errors="ignore").strip()
                    if line:
                        parsed = self.parse_line(line)
                        if parsed is not None:
                            # parsed -> (t_ms, angle_deg, distance_cm_or_None)
                            self.queue.append(parsed)
                except serial.SerialException as e:
                    self.last_error = f"Serial error: {e}"
                    self._close_serial()
                    time.sleep(RECONNECT_DELAY)
                except Exception as e:
                    # Non-fatal parse/other errors
                    self.last_error = f"Read error: {e}"
                    time.sleep(0.01)
            else:
                time.sleep(RECONNECT_DELAY)

    def _attempt_connect(self):
        try:
            if self.port is None:
                # Try to auto-detect Arduino-like serial ports (best-effort)
                ports = list(serial.tools.list_ports.comports())
                candidate = None
                for p in ports:
                    # prefer ports with Arduino or USB-SERIAL in description
                    desc = (p.description or "").lower()
                    if "arduino" in desc or "usb serial" in desc or "ch340" in desc or "cp210" in desc:
                        candidate = p.device
                        break
                if candidate is None and ports:
                    candidate = ports[0].device
                if candidate is None:
                    self.last_error = "No serial ports found"
                    return
                self.port = candidate

            self.serial = serial.Serial(self.port, self.baud, timeout=1)
            # Flush input
            time.sleep(0.1)
            self.serial.reset_input_buffer()
            self.connected = True
            self.last_error = ""
        except Exception as e:
            self.last_error = f"Connect fail: {e}"
            self.connected = False
            self.serial = None
            # keep port as None if it was not provided so we try others next time
            if self.port is None:
                time.sleep(RECONNECT_DELAY)

    def _close_serial(self):
        try:
            if self.serial:
                self.serial.close()
        except Exception:
            pass
        self.serial = None
        self.connected = False

    def parse_line(self, line: str):
        """
        Expected CSV: time_ms,angle_deg,distance_cm
        Example: 1234,45,150
        If distance is 'out' or non-numeric -> distance is None
        Return None on completely unrecognized line.
        """
        # guard: sometimes Arduino prints extra text, ignore header lines
        if line.lower().startswith("time") or line.lower().startswith("angle"):
            return None
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 3:
            # try split by whitespace if comma not used
            parts = [p.strip() for p in line.split()]
            if len(parts) < 3:
                return None
        try:
            t = int(parts[0])
            angle = float(parts[1])
            dist_raw = parts[2]
            if dist_raw.lower() in ("out", "nan", ""):
                dist = None
            else:
                # If the Arduino sends extra text like "120 cm", extract numeric prefix
                # Keep parsing robust
                numstr = ""
                for ch in dist_raw:
                    if (ch.isdigit() or ch == "." or ch == "-"):
                        numstr += ch
                    else:
                        break
                dist = float(numstr) if numstr != "" else None
            return (t, angle, dist)
        except Exception:
            # ignore lines that cannot be parsed
            return None

# ---------------------------------------
# Utilities: color interpolation
# ---------------------------------------
def lerp(a, b, t):
    return a + (b - a) * t

def color_red_to_green(t):
    """
    t: 0..1 where 0 -> RED (close), 1 -> GREEN (far)
    returns an (R,G,B) tuple in 0..255
    """
    t = max(0.0, min(1.0, t))
    r = int(lerp(255, 0, t))
    g = int(lerp(0, 255, t))
    b = 0
    return (r, g, b)

# ---------------------------------------
# Pygame visualization
# ---------------------------------------
def run_visualizer(args):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Arduino Radar Visualizer")
    font = pygame.font.SysFont("Arial", 18)
    clock = pygame.time.Clock()

    sr = SerialReader(args.port, args.baud)
    sr.start()

    min_d = args.min_distance
    max_d = args.max_distance

    running = True
    while running:
        # handle events
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False

        # background
        screen.fill((20, 20, 30))

        # Draw radar circles and grid
        draw_radar_grid(screen, min_d, max_d, font)

        # draw readings from sr.queue
        readings = list(sr.queue)  # shallow copy
        if readings:
            # Draw historical points - older points more transparent
            for i, (t, angle, dist) in enumerate(readings):
                age_index = len(readings) - 1 - i  # 0 = newest, larger = older
                alpha = int(255 * (1 - (age_index / max(1, len(readings) - 1)))) if len(readings) > 1 else 255
                draw_reading(screen, angle, dist, min_d, max_d, alpha)

            # draw sweep line and last numeric reading
            last_t, last_angle, last_dist = readings[-1]
            draw_sweep_line(screen, last_angle, last_dist, min_d, max_d)
            draw_status_text(screen, sr, last_angle, last_dist, font)
        else:
            # no readings yet
            draw_status_text(screen, sr, None, None, font)

        pygame.display.flip()
        clock.tick(60)

    # cleanup
    sr.stop()
    sr.join(timeout=1.0)
    pygame.quit()


def draw_radar_grid(screen, min_d, max_d, font):
    # draw concentric circles
    pygame.draw.rect(screen, (5, 5, 10), (0, 0, WIDTH, HEIGHT))  # solid background
    num_rings = 4
    for i in range(1, num_rings + 1):
        radius = int((i / num_rings) * MAX_RADIUS_PX)
        color = (40, 60, 40)
        pygame.draw.circle(screen, color, (CENTER_X, CENTER_Y), radius, 1)
        # label ring distance
        dist_label = int(lerp(min_d, max_d, i / num_rings))
        label_surf = font.render(f"{dist_label} cm", True, (200, 200, 200))
        screen.blit(label_surf, (CENTER_X + 8, CENTER_Y - radius - 8))

    # baseline and center marker
    pygame.draw.line(screen, (40, 40, 40), (20, CENTER_Y), (WIDTH - 20, CENTER_Y), 2)
    pygame.draw.circle(screen, (180, 180, 180), (CENTER_X, CENTER_Y), 4)


def draw_reading(screen, angle_deg, dist_cm, min_d, max_d, alpha=255):
    """
    Draw a point for a reading. alpha 0..255 used to fade older points.
    """
    pos = polar_to_screen(angle_deg, dist_cm, min_d, max_d)
    if pos is None:
        # out of range - draw small blue dot at max radius in that direction
        pos = polar_to_screen(angle_deg, max_d, min_d, max_d)
        color = (100, 150, 255)
    else:
        t = distance_to_fraction(dist_cm, min_d, max_d)
        color = color_red_to_green(t)

    # apply alpha by drawing on a temporary surface
    surf = pygame.Surface((12, 12), pygame.SRCALPHA)
    rcol = (color[0], color[1], color[2], alpha)
    pygame.draw.circle(surf, rcol, (6, 6), 6)
    screen.blit(surf, (pos[0] - 6, pos[1] - 6))


def draw_sweep_line(screen, angle_deg, dist_cm, min_d, max_d):
    # draw line showing current sweep
    if angle_deg is None:
        return
    end = polar_to_screen(angle_deg, max_d, min_d, max_d)
    if end is None:
        return
    pygame.draw.line(screen, (200, 200, 200), (CENTER_X, CENTER_Y), end, 2)
    # also draw a highlighted point for the latest reading
    pos = polar_to_screen(angle_deg, dist_cm, min_d, max_d)
    col = (180, 180, 180) if pos is None else color_red_to_green(distance_to_fraction(dist_cm, min_d, max_d))
    if pos is None:
        pos = end
    pygame.draw.circle(screen, col, pos, 8, 2)


def draw_status_text(screen, sr: SerialReader, angle, dist, font):
    # connection status
    status = "CONNECTED" if sr.connected else "DISCONNECTED"
    status_col = (50, 220, 50) if sr.connected else (220, 50, 50)
    txt = font.render(f"Status: {status}", True, status_col)
    screen.blit(txt, (10, 8))

    porttxt = font.render(f"Port: {sr.port or 'auto'}  Baud: {sr.baud}", True, (200, 200, 200))
    screen.blit(porttxt, (10, 34))

    if sr.last_error:
        errtxt = font.render(f"Last error: {sr.last_error}", True, (220, 120, 80))
        screen.blit(errtxt, (10, 60))

    if angle is not None:
        angtxt = font.render(f"Angle: {angle:.1f}°", True, (200, 200, 200))
        screen.blit(angtxt, (10, 100))
    if dist is None:
        disttxt = font.render(f"Distance: out", True, (100, 150, 255))
        screen.blit(disttxt, (10, 126))
    elif dist is not None:
        disttxt = font.render(f"Distance: {dist:.1f} cm", True, (220, 220, 220))
        screen.blit(disttxt, (10, 126))

    hint = font.render("Press ESC to quit", True, (140, 140, 140))
    screen.blit(hint, (WIDTH - 170, 8))


def distance_to_fraction(dist_cm, min_d, max_d):
    """
    Convert distance to 0..1 fraction where 0 => min_d (closest) -> RED,
    1 => max_d (furthest) -> GREEN.
    If dist is None or out of range, returns 0.0 for mapping (closest) but caller may treat None separately.
    """
    if dist_cm is None:
        return 0.0
    # clamp
    d = max(min_d, min(max_d, dist_cm))
    if max_d == min_d:
        return 1.0
    return (d - min_d) / (max_d - min_d)


def polar_to_screen(angle_deg, dist_cm, min_d, max_d):
    """
    Convert polar (angle in degrees, distance in cm) to screen (x,y).
    If dist_cm is None or beyond max_d, returns None.
    Angle mapping:
      - By convention, servo angle 90 is considered "straight ahead / up".
      - We map angle_deg -> rad with 90 deg as up, increasing to the right.
    """
    if dist_cm is None:
        return None
    # clamp distance
    if dist_cm < min_d:
        # still draw, but at smallest scale
        d = min_d
    elif dist_cm > max_d:
        return None  # report as out of range
    else:
        d = dist_cm
    frac = (d - min_d) / (max_d - min_d) if max_d != min_d else 1.0
    frac = max(0.0, min(1.0, frac))
    radius = int(frac * MAX_RADIUS_PX)
    # convert angle: we want 90 deg -> upward (0 rad), angle increase to right
    rad = math.radians(angle_deg - 90)
    x = CENTER_X + int(math.cos(rad) * radius)
    y = CENTER_Y + int(math.sin(rad) * radius)  # screen y increases downwards
    return (x, y)


# ---------------------------------------
# Command-line interface
# ---------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description="Arduino HC-SR04 Radar Visualizer")
    p.add_argument("--port", "-p", default=DEFAULT_PORT, help="Serial port (e.g. COM5 or /dev/ttyACM0). If omitted the script will try to auto-detect.")
    p.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD, help="Serial baud rate (default 9600)")
    p.add_argument("--min-distance", type=float, default=DEFAULT_MIN_DISTANCE, help="Minimum distance (cm) for coloring (closest -> red)")
    p.add_argument("--max-distance", type=float, default=DEFAULT_MAX_DISTANCE, help="Maximum distance (cm) for coloring (farthest -> green)")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        run_visualizer(args)
    except KeyboardInterrupt:
        pass
