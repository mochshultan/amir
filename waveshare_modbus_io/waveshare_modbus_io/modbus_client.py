# Kompatibel pymodbus 2.x (termasuk 2.5.3) dan aman bila nanti upgrade ke 3.x
try:
    # pymodbus >= 3.x
    from pymodbus.client import ModbusSerialClient
except Exception:
    # pymodbus 2.x
    from pymodbus.client.sync import ModbusSerialClient

import threading, time, logging

class WaveshareIOClient:
    """
    Pembungkus akses Modbus RTU untuk Waveshare 8DI/8DO.

    Mapping standar:
      - Read DO: FC 0x01 read_coils start=0 qty=8
      - Read DI: FC 0x02 read_discrete_inputs start=0 qty=8
      - Write single DO: FC 0x05 write_coil(addr,bool)
      - Write multi DO:  FC 0x0F write_coils(start, list[bool])

    Toggle diimplementasi portabel: read_coils(addr,1) -> write_coil(addr, not state)
    """
    def __init__(self, port, baudrate, parity, stopbits, bytesize, unit, reconnect_sec=2.0):
        self.client = ModbusSerialClient(
            method="rtu",
            port="/dev/ttyS3",
            baudrate=int(baudrate),
            parity=str(parity),
            stopbits=int(stopbits),
            bytesize=int(bytesize),
            timeout=1.5
        )
        self.unit = int(unit)
        self.lock = threading.Lock()
        self.reconnect_sec = float(reconnect_sec)
        self._ensure_connect()

    def _ensure_connect(self):
        # di pymodbus 2.x properti 'connected' tidak selalu ada; handle keduanya
        if not getattr(self.client, "connected", False):
            self.client.connect()

    def _with_retry(self, fn, *args, **kwargs):
        with self.lock:
            for _ in range(2):
                try:
                    self._ensure_connect()
                    res = fn(*args, **kwargs)
                    if hasattr(res, "isError") and res.isError():
                        raise RuntimeError(str(res))
                    return res
                except Exception as e:
                    logging.warning(f"Modbus error: {e}; reconnecting...")
                    try:
                        self.client.close()
                    except Exception:
                        pass
                    time.sleep(self.reconnect_sec)
            raise RuntimeError("Modbus operation failed after retries")

    # ---- READS ----
    def read_inputs_8(self):
        rr = self._with_retry(self.client.read_discrete_inputs, 0x0000, 8, unit=self.unit)
        return [bool(rr.bits[i]) if i < len(rr.bits) else False for i in range(8)]
     
    def read_outputs_8(self):
        rr = self._with_retry(self.client.read_coils, 0x0000, 8, unit=self.unit)
        return [bool(rr.bits[i]) if i < len(rr.bits) else False for i in range(8)]

    # ---- WRITES ----
    def write_single(self, ch, cmd):
        """
        ch: 1..8
        cmd: "on"|"off"|"toggle"
        """
        addr = int(ch) - 1
        if not (0 <= addr <= 7):
            raise ValueError("channel harus 1..8")

        if cmd == "on":
            self._with_retry(self.client.write_coil, addr, True, unit=self.unit)
            return True
        elif cmd == "off":
            self._with_retry(self.client.write_coil, addr, False, unit=self.unit)
            return True
        elif cmd == "toggle":
            # ---- FIX: baca block 8 bit dari alamat 0, lalu toggle bit target ----
            rr = self._with_retry(self.client.read_coils, 0x0000, 8, unit=self.unit)
            cur_list = [bool(rr.bits[i]) if i < len(rr.bits) else False for i in range(8)]
            cur = cur_list[addr]
            new_val = not cur
            # Tulis single (aman & sudah terbukti jalan di "on/off")
            self._with_retry(self.client.write_coil, addr, new_val, unit=self.unit)
            # (opsional) kalau Anda lebih suka tulis block:
            # cur_list[addr] = new_val
            # self._with_retry(self.client.write_coils, 0x0000, cur_list, unit=self.unit)
            return True
        else:
            raise ValueError("cmd must be on/off/toggle")


    def write_mask(self, mask_byte):
        vals = [bool((mask_byte >> i) & 1) for i in range(8)]
        self._with_retry(self.client.write_coils, 0x0000, vals, unit=self.unit)
        return True
