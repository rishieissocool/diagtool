"""Background UDP receiver for onboard ball-detection telemetry.

Each robot running RobotFramework sends a UDP packet to this listener.
`OnboardReceiver` runs a daemon thread that binds to a port, parses each
packet, tags it with `robot_id` (from the sender IP if the packet
doesn't carry one), and writes it into an `OnboardObservationStore`.
"""

import socket
import threading
import time

from .observation import parse_packet
from .store import OnboardObservationStore


DEFAULT_PORT = 50513  # matches RobotFramework's sender_port


class OnboardReceiver:
    """Daemon UDP listener. Construct, then `start()`.

    `ip_to_robot` lets the PC map sender IPs to (is_yellow, robot_id)
    when the packet itself doesn't include them — a common case with
    the current RobotFramework packet format.
    """

    def __init__(
        self,
        store=None,
        port=DEFAULT_PORT,
        bind_addr="0.0.0.0",
        ip_to_robot=None,
        recv_timeout=0.5,
    ):
        self.store = store if store is not None else OnboardObservationStore()
        self.port = port
        self.bind_addr = bind_addr
        self.ip_to_robot = dict(ip_to_robot) if ip_to_robot else {}
        self._recv_timeout = recv_timeout

        self._sock = None
        self._thread = None
        self._running = threading.Event()
        self._packets = 0
        self._errors = 0

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.bind_addr, self.port))
        self._sock.settimeout(self._recv_timeout)
        self._running.set()
        self._thread = threading.Thread(
            target=self._run, name="OnboardReceiver", daemon=True)
        self._thread.start()

    def stop(self):
        self._running.clear()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _run(self):
        while self._running.is_set():
            try:
                data, addr = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            obs = parse_packet(data)
            if obs is None:
                self._errors += 1
                continue
            obs.recv_ts = time.time()

            if obs.robot_id < 0:
                mapping = self.ip_to_robot.get(addr[0])
                if mapping is not None:
                    obs.is_yellow = bool(mapping[0])
                    obs.robot_id = int(mapping[1])

            if obs.robot_id < 0:
                # Packet can't be attributed to a known robot — drop it
                # rather than polluting the store with a bogus key.
                self._errors += 1
                continue

            self.store.put(obs)
            self._packets += 1

    @property
    def packets_received(self):
        return self._packets

    @property
    def parse_errors(self):
        return self._errors
