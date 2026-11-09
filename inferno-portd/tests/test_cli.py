#!/usr/bin/env python3
"""Exercise CLI parsing without opening hardware. Licence: GPL-3.0-or-later."""
import subprocess
import sys
import unittest

BIN = sys.argv.pop(1)


class CliEdges(unittest.TestCase):
    def test_reject_invalid_arguments(self):
        for args in (("--ip", "192.0.2.10/-1"), ("--ip", "192.0.2.10/33"),
                     ("--ip", "192.0.2.10/999"), ("--ip", "invalid"),
                     ("--port", "0"), ("--port", "7")):
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


if __name__ == "__main__":
    unittest.main()
