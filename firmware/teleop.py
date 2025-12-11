#!/usr/bin/env python3
import curses
import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

LINEAR_SPEED = 0.2   # m/s
ANGULAR_SPEED = 0.8  # rad/s
HZ = 20.0            # send frequency
STOP_TIMEOUT = 0.5   # seconds with no key → auto-stop

def teleop(stdscr):
    ser = serial.Serial(PORT, BAUD, timeout=0.01)

    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.noecho()

    vx = 0.0
    vy = 0.0
    wz = 0.0
    last_key_time = time.monotonic()

    stdscr.addstr(0, 0, "Teleop: arrow keys to drive, SPACE to stop, q to quit")
    stdscr.refresh()

    period = 1.0 / HZ

    try:
        while True:
            last_key = None
            while True:
                key = stdscr.getch()
                if key == -1:
                    break
                last_key = key

            if last_key is not None:
                if last_key == curses.KEY_UP:
                    vx = LINEAR_SPEED
                    vy = 0.0
                    wz = 0.0
                elif last_key == curses.KEY_DOWN:
                    vx = -LINEAR_SPEED
                    vy = 0.0
                    wz = 0.0
                elif last_key == curses.KEY_LEFT:
                    vx = 0.0
                    vy = 0.0
                    wz = +ANGULAR_SPEED
                elif last_key == curses.KEY_RIGHT:
                    vx = 0.0
                    vy = 0.0
                    wz = -ANGULAR_SPEED
                elif last_key == ord(' '):  # space = stop
                    vx = vy = wz = 0.0
                elif last_key == ord('q'):
                    vx = vy = wz = 0.0
                    # send one last stop command
                    cmd = f"V {vx:.3f} {vy:.3f} {wz:.3f}\n"
                    ser.write(cmd.encode("ascii"))
                    break

                last_key_time = time.monotonic()

                # Debug display
                stdscr.move(1, 0)
                stdscr.clrtoeol()
                stdscr.addstr(1, 0, f"V vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}")
                stdscr.refresh()

            if time.monotonic() - last_key_time > STOP_TIMEOUT:
                vx = vy = wz = 0.0

            cmd = f"V {vx:.3f} {vy:.3f} {wz:.3f}\n"
            ser.write(cmd.encode("ascii"))

            time.sleep(period)

    finally:
        ser.close()

if __name__ == "__main__":
    curses.wrapper(teleop)
