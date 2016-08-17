#!/usr/bin/env python

# Copyright 2016 Preferred Networks, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import bitstring
import time
import threading


_client_lock = threading.Lock()


class Controller(object):
    def __init__(self, client, ctrl_id):
        self.client = client
        self.id = ctrl_id

    def initialize(self):
        try:
            stat = self.status()
        except Exception as e:
            print "status() call returned exception:", e
            print "try to discover axes via the GUI program"
            raise e

        if not stat["PMSS"]:
            # enable MODBUS
            self.write_PMSL(True)
            time.sleep(0.5)

        # reset alerts
        self.write_ALRS(True)
        self.write_ALRS(False)

        # switch on servo, if needed
        if not stat["SV"]:
            self.write_SON(True)
            time.sleep(1.0)  # wait for servo to come on

        # go home
        self.write_HOME(True)
        self.write_HOME(False)

        while True:
            stat = self.status()
            if not stat["ERR"]:
                # sleep as long as servo is not turned on
                if not stat["SV"]:
                    time.sleep(0.1)
                    continue
            # break on error of if SV is on
            break

        return stat

    def shutdown(self):
        self.write_SON(False)

    def __read(self, address, count):
        v = self.client.read_holding_registers(address, count,
                                               unit=self.id)
        if v is None:
            raise ValueError("read_holding_registers returned None")
        return v

    def __read_x(self, name, address, count=1):
        N = 10
        # execute the read call multiple times if failed
        for i in range(N):
            with _client_lock:
                try:
                    val = self.__read(address, count)
                    binary = ('0b' + ''.join(
                        ['{0:016b}'.format(val.getRegister(i)) for i in range(count)]
                    ))
                except Exception as e:
                    print "[unit %s] exception \"%s\" while reading, retry" % (self.id, e)
                    time.sleep(0.03)
                    continue
            return bitstring.BitArray(binary)
        # if we arrive here, we have failed too often
        raise ValueError("[unit %s] __read failed to often; last error: %s" %
                         (self.id, e))

    def read_POSR(self):
        return self.__read_x("POSR", 0xD03)

    def write_POSR_and_move(self, value):
        bs = bitstring.Bits(int=value, length=16)
        with _client_lock:
            self.client.write_register(0x9800, bs.uint,
                                       unit=self.id)

    def write_PCMD_and_move(self, value):
        if value < -999999 or value > 999999:
            print "value out of range for MODBUS: %d" % value
            return
        bs = bitstring.Bits(int=value, length=32)
        with _client_lock:
            self.client.write_registers(0x9900,
                [bs[:16].uint, bs[16:].uint], unit=self.id)

    def read_ALA0(self):
        return self.__read_x("ALA0", 0x500)

    def read_ALC0(self):
        return self.__read_x("ALC0", 0x503)

    def read_DRG1(self):
        return self.__read_x("DRG1", 0xD00)

    def read_DSS1(self):
        return self.__read_x("DSS1", 0x9005)

    def read_DSSE(self):
        return self.__read_x("DSSE", 0x9007)

    def read_STAT(self):
        return self.__read_x("STAT", 0x9008, 2)

    def read_PNOW(self):
        return self.__read_x("PNOW", 0x9000, 2)

    def write_SON(self, value):
        with _client_lock:
            self.client.write_coil(0x403, value,
                                   unit=self.id)

    def write_HOME(self, value):
        with _client_lock:
            self.client.write_coil(0x40B, value,
                                   unit=self.id)

    def write_ALRS(self, value):
        with _client_lock:
            self.client.write_coil(0x407, value,
                                   unit=self.id)

    def write_PMSL(self, value):
        with _client_lock:
            self.client.write_coil(0x427, value,
                                   unit=self.id)

    def status(self):
        # see ME0162-7A MODBUS manual for explanation
        # cf. p45
        DSSE = self.read_DSSE()
        # cf. p43
        DSS1 = self.read_DSS1()
        if DSS1[-11] or DSS1[-10] or DSS1[-9]:
            ALC0 = self.read_ALC0().hex
        else:
            ALC0 = None

        return {"HEND": DSS1[-5],
                "MOVE": DSSE[-6],
                "GHMS": DSSE[-12],
                "PEND": DSS1[-4],
                "STP": DSS1[-6],
                "SFTY": DSS1[-15],
                "PWR": DSS1[-14],
                "SV": DSS1[-13],
                "CEND": DSS1[-3],
                "CLBS": DSS1[-2],
                "PMSS": DSSE[-9],
                "ALMH": DSS1[-11],
                "ALML": DSS1[-10],
                "ABER": DSS1[-9],
                "ERR": DSS1[-11] or DSS1[-10] or DSS1[-9],
                "ALC0": ALC0}


class LinearServoController(Controller):
    def __init__(self, client, ctrl_id,
                 lower_lim=0, upper_lim=20):
        super(LinearServoController, self).__init__(client, ctrl_id)
        self.lower_lim = lower_lim
        self.upper_lim = upper_lim

    def move_to_preset(self, idx):
        """Moves the servo to one of the positions
        predefined in the position table."""
        if not isinstance(idx, (int, long)):
            raise ValueError("parameter must be an integer, not %s" % idx)
        if idx < 0:
            raise ValueError("parameter must be positive, not %s" % idx)
        self.write_POSR_and_move(idx)
        return self.status()

    def move_to_position(self, pos):
        """Moves the servo to the specified position in mm."""
        if not isinstance(pos, (int, long, float)):
            raise ValueError("parameter must be numeric, not %s" % idx)
        if self.lower_lim is not None and pos < self.lower_lim:
            raise ValueError("parameter is less than lower limit (%s)" %
                             self.lower_lim)
        if self.upper_lim is not None and pos > self.upper_lim:
            raise ValueError("parameter is greater than upper limit (%s)" %
                             self.upper_lim)
        self.write_PCMD_and_move(int(pos * 100))
        return self.status()


class RotaryServoController(Controller):
    def __init__(self, client, ctrl_id,
                 lower_lim=0, upper_lim=200):
        super(RotaryServoController, self).__init__(client, ctrl_id)
        self.lower_lim = lower_lim
        self.upper_lim = upper_lim

    def move_to_preset(self, idx):
        """Moves the servo to one of the positions
        predefined in the position table."""
        if not isinstance(idx, (int, long)):
            raise ValueError("parameter must be an integer, not %s" % idx)
        if idx < 0:
            raise ValueError("parameter must be positive, not %s" % idx)
        self.write_POSR_and_move(idx)
        return self.status()

    def move_to_position(self, pos):
        """Moves the servo to the specified position in degrees."""
        if not isinstance(pos, (int, long, float)):
            raise ValueError("parameter must be numeric, not %s" % idx)
        if self.lower_lim is not None and pos < self.lower_lim:
            raise ValueError("parameter is less than lower limit (%s)" %
                             self.lower_lim)
        if self.upper_lim is not None and pos > self.upper_lim:
            raise ValueError("parameter is greater than upper limit (%s)" %
                             self.upper_lim)
        self.write_PCMD_and_move(int(pos * 100))
        return self.status()


