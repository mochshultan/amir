#!/usr/bin/env python3
# ZLAC8015D Modbus RTU driver - stable version (no auto reconnect)
import importlib.metadata

# --- Support pymodbus 2.x & 3.x ---
try:
    from pymodbus.client import ModbusSerialClient  # pymodbus >=3.x
    PM_NEW = True
except Exception:
    from pymodbus.client.sync import ModbusSerialClient  # pymodbus 2.x legacy
    PM_NEW = False

try:
    _ver = importlib.metadata.version("pymodbus")
except Exception:
    _ver = "unknown"

print(f"[ZLAC] pymodbus version {_ver} (new_api={PM_NEW})")

# --- Registers ---
REG_CONTROL_MODE     = 0x200D
REG_CONTROL_WORD     = 0x200E
REG_TARGET_VEL_L     = 0x2088
REG_TARGET_VEL_R     = 0x2089
REG_ACTUAL_VEL_L     = 0x20AB
REG_ACTUAL_VEL_R     = 0x20AC
REG_POS_L_HI         = 0x20A7
REG_POS_L_LO         = 0x20A8
REG_POS_R_HI         = 0x20A9
REG_POS_R_LO         = 0x20AA
REG_BUS_VOLT         = 0x20A1
REG_ACT_TORQUE_L     = 0x20AD
REG_ACT_TORQUE_R     = 0x20AE
REG_DRIVER_TEMP      = 0x20B0

# --- Helper conversion ---
def i16(v: int) -> int:
    return v - 0x10000 if v > 0x7FFF else v

def i32(hi: int, lo: int) -> int:
    val = ((hi & 0xFFFF) << 16) | (lo & 0xFFFF)
    if val & 0x80000000:
        val -= 0x100000000
    return val

# --- Main driver ---
class ZLACDriver:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, parity='N', stopbits=1, slave_id=1, timeout=0.1):
        self.unit = slave_id
        self.client = ModbusSerialClient(
            method='rtu', port=port, baudrate=baudrate,
            parity=parity, stopbits=stopbits, timeout=timeout
        )
        self._last_cmd_l = None
        self._last_cmd_r = None

        if self.client.connect():
            print(f"[ZLAC] ✅ Connected {port}@{baudrate}")
            try:
                self._write_u16(REG_CONTROL_MODE, 3)  # Profile velocity mode
            except Exception as e:
                print(f"[ZLAC] ⚠️ Cannot set mode: {e}")
        else:
            print(f"[ZLAC] ❌ Failed to connect to {port}")

    # --- Low-level IO ---
    def _write_u16(self, addr, val):
        try:
            rr = self.client.write_register(addr, int(val) & 0xFFFF, unit=self.unit)
            if hasattr(rr, "isError") and rr.isError():
                print(f"[ZLAC] ⚠️ Write failed 0x{addr:04X}")
        except Exception as e:
            print(f"[ZLAC] ⚠️ Write error 0x{addr:04X}: {e}")

    def _write_u16s(self, addr, values):
        try:
            payload = [(int(v) & 0xFFFF) for v in values]
            rr = self.client.write_registers(addr, payload, unit=self.unit)
            if hasattr(rr, "isError") and rr.isError():
                print(f"[ZLAC] ⚠️ Multi-write failed 0x{addr:04X}")
                return False
            return True
        except Exception as e:
            print(f"[ZLAC] ⚠️ Multi-write error 0x{addr:04X}: {e}")
            return False

    def read_registers(self, addr, count=1):
        try:
            rr = self.client.read_holding_registers(addr, count, unit=self.unit)
            if hasattr(rr, "isError") and rr.isError():
                return None
            return rr.registers
        except Exception:
            return None

    # --- Commands ---
    def enable(self):
        self._write_u16(REG_CONTROL_WORD, 0x0008)

    def disable(self):
        self._write_u16(REG_CONTROL_WORD, 0x0007)

    def stop(self):
        self.set_speed(0, 0)

    def set_speed(self, left_rpm, right_rpm):
        left_cmd = int(left_rpm)
        right_cmd = int(right_rpm)
        if left_cmd == self._last_cmd_l and right_cmd == self._last_cmd_r:
            return

        # 0x2088 and 0x2089 are contiguous, so we can update both wheels in one RTU frame.
        if self._write_u16s(REG_TARGET_VEL_L, [left_cmd, right_cmd]):
            self._last_cmd_l = left_cmd
            self._last_cmd_r = right_cmd
        else:
            # Force retry in next control tick if write failed.
            self._last_cmd_l = None
            self._last_cmd_r = None

    # --- Reading data ---
    def read_u16(self, addr):
        regs = self.read_registers(addr, 1)
        if regs is None:
            return None
        return regs[0]

    def read_u32(self, reg_hi):
        regs = self.read_registers(reg_hi, 2)
        if regs is None or len(regs) < 2:
            return None
        return (regs[0] << 16) | regs[1]

    def read_positions(self):
        regsL = self.read_registers(REG_POS_L_HI, 2)
        regsR = self.read_registers(REG_POS_R_HI, 2)
        if regsL is None or regsR is None:
            return None, None
        encL = i32(regsL[0], regsL[1])
        encR = i32(regsR[0], regsR[1])
        return encL, encR

    def read_actual_velocity(self):
        regs = self.read_registers(REG_ACTUAL_VEL_L, 2)
        if regs is None:
            return None, None
        return i16(regs[0]) / 10.0, i16(regs[1]) / 10.0

    def read_feedback_core(self):
        # Read [0x20A7..0x20B0] in one shot:
        # pos L/R (4 regs), actual velocity (2), torque L/R (2), reserved (1), temp (1)
        start = REG_POS_L_HI
        count = (REG_DRIVER_TEMP - REG_POS_L_HI) + 1
        regs = self.read_registers(start, count)
        if regs is None or len(regs) < count:
            return None

        def at(reg):
            return regs[reg - start]

        return {
            "enc_l_hi": at(REG_POS_L_HI),
            "enc_l_lo": at(REG_POS_L_LO),
            "enc_r_hi": at(REG_POS_R_HI),
            "enc_r_lo": at(REG_POS_R_LO),
            "rpm_l_raw": at(REG_ACTUAL_VEL_L),
            "rpm_r_raw": at(REG_ACTUAL_VEL_R),
            "torq_l_raw": at(REG_ACT_TORQUE_L),
            "torq_r_raw": at(REG_ACT_TORQUE_R),
            "temp_raw": at(REG_DRIVER_TEMP),
        }

    def close(self):
        try:
            self.client.close()
        except Exception:
            pass
