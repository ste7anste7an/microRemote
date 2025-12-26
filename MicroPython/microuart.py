# ============================================================
#  MicroUART – unified Pybricks + MicroPython ESP32 library
# ============================================================

import __main__

# ---------- Platform detection ----------

IS_PYBRICKS = False
try:
    from pybricks.iodevices import UARTDevice
    from pybricks.tools import StopWatch
    IS_PYBRICKS = True
except ImportError:
    pass

if not IS_PYBRICKS:
    import time
    import machine
    from lms_esp32 import RX_PIN, TX_PIN

# ---------- Constants ----------

PREAMBLE = b'<$MU'



# ============================================================
#  MicroUART
# ============================================================

class MicroUART:
    def __init__(
        self,
        port_or_uart=1,
        baudrate=115200,
        wait_recv=1000,
        uart_timeout=1000,
        rx=RX_PIN,
        tx=TX_PIN,
    ):
        self.byte_timeout = 10
        self.wait_recv = wait_recv

        if IS_PYBRICKS:
            # Pybricks backend
            self.uart = UARTDevice(port_or_uart, timeout=uart_timeout)
            self.uart.set_baudrate(baudrate)
        else:
            # MicroPython ESP32 backend
            
            self.uart = machine.UART(
                port_or_uart,
                baudrate=baudrate,
                rx=machine.Pin(rx),
                tx=machine.Pin(tx),
                timeout=uart_timeout,
            )

    # ---------- Timing abstraction ----------

    def _now(self):
        if IS_PYBRICKS:
            return StopWatch()
        return time.ticks_ms()

    def _elapsed(self, start):
        if IS_PYBRICKS:
            return start.time()
        return time.ticks_diff(time.ticks_ms(), start)

    # ---------- UART abstraction ----------

    def _waiting(self):
        if IS_PYBRICKS:
            return self.uart.waiting()
        return self.uart.any()

    def _read(self, n=1):
        return self.uart.read(n)

    def _read_all(self):
        if IS_PYBRICKS:
            self.uart.read_all()
        else:
            self.uart.read()

    def _write(self, b):
        self.uart.write(b)

    # ---------- Utility ----------

    def flush(self):
        while self._waiting():
            self._read_all()

    # ---------- Framing ----------

    def send_bytes(self, payload):
        b = PREAMBLE + payload
        b = bytes([len(b)]) + b
        self._write(b)

    def receive_bytes(self):
        start = self._now()

        # wait for length byte
        while self._elapsed(start) < self.wait_recv and self._waiting() == 0:
            if not IS_PYBRICKS:
                time.sleep_ms(1)

        if self._waiting() == 0:
            self.flush()
            return b''

        length = self._read(1)[0]

        payload = bytearray()
        total_start = self._now()
        byte_start = self._now()
        preamble_index = 0

        while len(payload) < length:
            if self._elapsed(total_start) > self.wait_recv:
                self.flush()
                return b''

            if self._waiting():
                b = self._read(1)
                if b:
                    payload.append(b[0])

                    if preamble_index < 4:
                        if b[0] != PREAMBLE[preamble_index]:
                            self.flush()
                            return b''
                        preamble_index += 1

                    byte_start = self._now()
            else:
                if self._elapsed(byte_start) > self.byte_timeout:
                    self.flush()
                    return b''
                if not IS_PYBRICKS:
                    time.sleep_ms(1)

        return bytes(payload[4:])

    # ---------- Encode / decode ----------

    def encode(self, cmd, *argv):
        encoded = bytes([len(cmd)]) + cmd.encode("utf-8")

        for arg in argv:
            if isinstance(arg, int):
                s = str(arg)
                encoded += bytes([78, len(s)]) + s.encode("utf-8")
            elif isinstance(arg, bytes):
                encoded += bytes([65, len(arg)]) + arg
            elif isinstance(arg, str):
                encoded += bytes([83, len(arg)]) + arg.encode("utf-8")
            elif isinstance(arg, bool):
                encoded += bytes([66, 1, 1 if arg else 0])

        return encoded

    def decode(self, encoded):
        cmd_len = encoded[0]
        cmd = encoded[1:1 + cmd_len].decode("utf-8")

        decoded = []
        p = 1 + cmd_len

        while p < len(encoded):
            t = encoded[p]
            length = encoded[p + 1]
            p += 2

            payload = encoded[p:p + length]
            p += length

            if t == 78:
                decoded.append(int(payload.decode("utf-8")))
            elif t == 65:
                decoded.append(payload)
            elif t == 83:
                decoded.append(payload.decode("utf-8"))
            elif t == 66:
                decoded.append(bool(payload[0]))
            else:
                raise ValueError("Unknown type code")

        return cmd, decoded

    # ---------- High-level API ----------

    def send_command(self, cmd, *data):
        self.send_bytes(self.encode(cmd, *data))

    def receive_command(self):
        b = self.receive_bytes()
        if b:
            try:
                return self.decode(b)
            except Exception:
                self.flush()
                return "!ERROR", "decode error"
        return "!ERROR", "no bytes received"

    def call(self, cmd, *data):
        self.send_command(cmd, *data)
        self.flush()
        return self.receive_command()

    def process(self):
        cmd, data = self.receive_command()

        if cmd != "!ERROR":
            if hasattr(__main__, cmd):
                func = getattr(__main__, cmd)
                resp = func(*data)
                if resp is None:
                    resp = ()
                self.send_command(cmd + "_ack", *resp)
        else:
            self.send_command(cmd + "_err", "recv error")
