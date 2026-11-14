#!/usr/bin/env python3
"""Exercise CLI parsing without opening hardware. Licence: GPL-3.0-or-later."""
import os
import socket
import tempfile
import subprocess
import sys
import unittest

BIN = sys.argv.pop(1)


class CliEdges(unittest.TestCase):
    def test_reject_invalid_arguments(self):
        for args in (("--ip", "192.0.2.10/-1"), ("--ip", "192.0.2.10/33"),
                     ("--ip", "192.0.2.10/999"), ("--ip", "invalid"),
                     ("--port", "0"), ("--port", "7"),
                     ("--rt", "0"), ("--rt", "100")):
            with self.subTest(args=args):
                with subprocess.Popen([BIN, "--backend", "model", "-q", *args],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) as proc:
                    try:
                        code = proc.wait(timeout=0.3)
                    except subprocess.TimeoutExpired:
                        proc.terminate()
                        proc.wait(timeout=2)
                        self.fail("invalid input started the daemon")
                    self.assertEqual(code, 2)

    def test_hardware_offsets_rejected_before_open(self):
        for args in (("--clock-port", "7"), ("--port-stride", "0xfffffffc"),
                     ("--port-stride", "4097"), ("--mem-size", "16"),
                     ("--dclk-base", "0xfffffffc"), ("--cs2k-base", "1048572"),
                     ("--port-stride", "0x100001000"), ("--mem-size", "1048577"),
                     ("--dclk-base", "garbage"), ("--backend", "typo")):
            with self.subTest(args=args):
                result = subprocess.run([BIN, *args], capture_output=True, timeout=2)
                self.assertEqual(result.returncode, 2)
                self.assertNotIn(b"/dev/mem", result.stderr)

    def test_accept_prefix_boundaries(self):
        for prefix in (0, 32):
            with self.subTest(prefix=prefix):
                with subprocess.Popen([BIN, "--backend", "model", "-q", "--ip", f"192.0.2.10/{prefix}"],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) as proc:
                    try:
                        code = proc.wait(timeout=0.1)
                    except subprocess.TimeoutExpired:
                        proc.terminate()
                        self.assertEqual(proc.wait(timeout=2), 0)
                    else:
                        self.fail(f"valid prefix rejected with exit {code}")

    def test_rt_priority_and_loop_gap(self):
        # LOG M442: a SCHED_OTHER portd on the busy AMU left cpu_pktq undrained long enough to overflow it.
        # --rt makes the poll loop SCHED_FIFO; status reports the worst gap between two drains.
        with tempfile.TemporaryDirectory() as directory:
            path = directory + "/control"
            with subprocess.Popen([BIN, "--backend", "model", "-q", "--rt", "10", "--ctl", path],
                                  stdout=subprocess.DEVNULL, stderr=subprocess.PIPE) as proc:
                try:
                    out = b""
                    for _ in range(50):
                        if proc.poll() is not None:
                            self.fail(f"--rt 10 exited {proc.returncode}: {proc.stderr.read()!r}")
                        out = subprocess.run([BIN, "ctl", path, "status"], capture_output=True, timeout=2).stdout
                        if out.endswith(b"ok\n"):
                            break
                        subprocess.run(["sleep", "0.1"])
                    gap = [line for line in out.split(b"\n") if line.startswith(b"loop_gap_max_us=")]
                    self.assertEqual(len(gap), 1, out)
                    self.assertLess(int(gap[0].split(b"=")[1]), 1000000)
                    if os.geteuid() == 0:
                        self.assertEqual(os.sched_getscheduler(proc.pid), os.SCHED_FIFO)
                        self.assertEqual(os.sched_getparam(proc.pid).sched_priority, 10)
                finally:
                    proc.terminate()
                    proc.wait(timeout=2)

    def test_oversized_control_command(self):
        # Rejected before connecting, regardless of the destination socket's availability.
        result = subprocess.run([BIN, "ctl", "/no/such/socket", "a" * 1022, "b" * 64],
                                capture_output=True, timeout=2)
        self.assertEqual(result.returncode, 2)
        self.assertIn(b"too long", result.stderr)

    def test_maximum_control_command(self):
        with tempfile.TemporaryDirectory() as directory, socket.socket(socket.AF_UNIX) as server:
            path = directory + "/control"
            server.bind(path)
            server.listen(1)
            server.settimeout(2)
            with subprocess.Popen([BIN, "ctl", path, "a" * 1022], stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE) as proc:
                with server.accept()[0] as conn:
                    conn.settimeout(2)
                    data = b""
                    while not data.endswith(b"\n"):
                        chunk = conn.recv(1024)
                        self.assertTrue(chunk)
                        data += chunk
                    self.assertEqual(data, b"a" * 1022 + b"\n")
                    conn.sendall(b"ok\n")
                out, err = proc.communicate(timeout=2)
                self.assertEqual(proc.returncode, 0, err)
                self.assertEqual(out, b"ok\n")

    def test_empty_and_partial_control_replies(self):
        for reply in (b"", b"name=test\n", b"ok\n", b"err refused\n"):
            with self.subTest(reply=reply), tempfile.TemporaryDirectory() as directory:
                with socket.socket(socket.AF_UNIX) as server:
                    path = directory + "/control"
                    server.bind(path)
                    server.listen(1)
                    server.settimeout(2)
                    with subprocess.Popen([BIN, "ctl", path, "status"], stdout=subprocess.PIPE,
                                          stderr=subprocess.PIPE) as proc:
                        with server.accept()[0] as conn:
                            conn.settimeout(2)
                            self.assertIn(b"status\n", conn.recv(1024))
                            conn.sendall(reply)
                        out, err = proc.communicate(timeout=2)
                        self.assertEqual(out, reply)
                        self.assertEqual(proc.returncode, 0 if reply == b"ok\n" else 1, err)


if __name__ == "__main__":
    unittest.main()
