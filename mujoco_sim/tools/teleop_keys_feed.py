#!/usr/bin/env python3
"""Feed stdin keystrokes into the teleop FIFO (run_mujoco_sim.sh 'keys').

Must be a standalone file — a bash heredoc would occupy fd 0 and steal the
very keystrokes this exists to forward. Probes for a reader with O_NONBLOCK,
then KEEPS that write fd for the whole session: probing with a separate
open/close would EOF the dispatcher (closing a FIFO's last writer wakes every
blocked reader). The caller puts the terminal in raw mode (stty min 1).
"""
import errno
import os
import sys
import time

path = sys.argv[1]
said = False
while True:
    try:
        fd = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
        break
    except OSError as e:
        if e.errno != errno.ENXIO:
            print(f"cannot open {path}: {e}")
            sys.exit(1)
        if not said:
            print("waiting for the teleop dispatcher to attach"
                  " (teleop profile running? sequence auto-starts ~15 s in)...")
            said = True
        time.sleep(1.0)
os.set_blocking(fd, True)
print("teleop dispatcher is LISTENING — keys go live now.")
try:
    while True:
        b = os.read(0, 1)
        if not b:
            print("stdin closed — key reader exiting.")
            break
        os.write(fd, b)
except KeyboardInterrupt:
    print("Ctrl+C — key reader exiting.")
except BrokenPipeError:
    print("teleop side closed — key reader exiting.")
