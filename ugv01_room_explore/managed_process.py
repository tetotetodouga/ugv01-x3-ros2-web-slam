"""Exec a software child that receives SIGINT if its dashboard parent dies.

This runs after exec in a fresh Python process, avoiding preexec_fn in the
multithreaded HTTP server. Linux preserves the parent-death signal across exec.
"""
import ctypes
import os
import signal
import sys


def main():
    parent = int(sys.argv[1])
    command = sys.argv[2:]
    if not command:
        raise SystemExit('Missing child command')
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGINT, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    if os.getppid() != parent:
        raise SystemExit('Dashboard parent exited before child startup')
    os.execvp(command[0], command)


if __name__ == '__main__':
    main()
