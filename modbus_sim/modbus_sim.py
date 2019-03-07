#!/usr/bin/env python
"""
A Modbus slave device used to simulate various RTU/PLC serial devices for testing.

Sets up a simmple device construct with 10,000 of each reg_type of register.
Increments values on the input registers every 5 seconds.

.. todo::
   Create several device templates as examples.

The file template uses metadata lines identified as comments starting with /*.
Each line in the file ends in a line feed ``\n``.
Use ``;`` to separate metadata tags.

* ``/**DEVICE_DESC;`` can be defined with:

   * ``VendorName=<string>``
   * ``ProductCode=<string>``
   * ``VendorUrl=<string>``
   * ``ProductName=<string>``
   * ``ModelName=<string>``
   * ``MajorMinorRevision=<string>``
   * ``sparse=<Boolean>`` True if polling of undefined registers should return an error

* ``/**PORT_SIM;`` can be defined with: port=<port>;mode=<mode>[;baudRate=<number>;parity=<parity>]`` where:

   * ``port=<str>`` e.g. tcp:502, udp:5020, COM35, /dev/ttyUSB0
   * ``mode=<str>`` tcp, rtu or ascii
   * ``baudRate=<int>``
   * ``parity=<str>`` none, even or odd

* ``deviceId=<number>;networkId=<number>[;plcBaseAddress=<plc>]`` where:

   * **deviceId** is a unique ID / arbitrary
   * **networkId** is the Modbus Slave ID
   * **plcBaseAddress** (default 0/False) can be set 1

* ``/*REGISTER`` is defined with:

   * ``paramId=<number>`` a uniqe parameter ID
   * ``name=<string>``
   * ``default=<value>`` the default value to configure in the register(s)

* ``paramId=<number>;deviceId=<number>;registerType=<reg>;address=<number>;encoding=<enc>[;length=<number>]`` where:

   * **paramId** is a unique identifier that may span multiple registers
   * **deviceId** maps to the unique device ID
   * **registerType** is one of:

      * **input** discrete input
      * **coil** discrete output
      * **analog** input register
      * **holding**  holding register

   * **address** is the Modbus or PLC address
   * **encoding** is one of:

      * **int8**, **int16**, **int32**, **uint8**, **uint16**, **uint32**
      * **float32**
      * **boolean**
      * **string**

   * **length** is required for **string** encodings to specify how many registers are used

"""

__version__ = "0.1.0"

import sys
import os
import argparse
import serial
import glob

import headless
from headless import RepeatingTimer
from simulators import sim_weather_lufft
import threading

from pymodbus import __version__ as pymodbus_version
from pymodbus.server.async import StartTcpServer
from pymodbus.server.async import StartUdpServer
from pymodbus.server.async import StartSerialServer

from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer, ModbusSocketFramer

from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.payload import BinaryPayloadDecoder

import httplib

PORT_DEFAULT = 'tcp:502'

active = True

# --------------------------------------------------------------------------- #
# Service logging configuration
# --------------------------------------------------------------------------- #
log = headless.get_wrapping_logger(debug=True)
server_log = headless.get_wrapping_logger(name="pymodbus.server", debug=True)

# --------------------------------------------------------------------------- #
# Modbus function code reference
# --------------------------------------------------------------------------- #
REGISTER_TYPES = ['hr', 'ir', 'di', 'co']
READ_CO = 0x01
READ_DI = 0x02
READ_HR = 0x03
READ_IR = 0x04
WRITE_SINGLE_CO = 0x05
WRITE_SINGLE_HR = 0x06
WRITE_MULTI_CO = 0x0f
WRITE_MULTI_HR = 0x10
READ_EXCEPTION_STATUS = 0x07
READ_DIAGNOSTICS = 0x08

REGISTER_ENCODING_TYPES = ['uint8', 'int8', 'uint16', 'int16', 'boolean', 'float32', 'ascii']

# --------------------------------------------------------------------------- #
# File parser configuration
# --------------------------------------------------------------------------- #
TEMPLATE_PARSER_DESC = "/**DEVICE_DESC"
TEMPLATE_PARSER_SIM_PORT = "/**SIM_PORT"
TEMPLATE_PARSER_PORT = "port"
TEMPLATE_PARSER_NETWORK = "deviceId"
TEMPLATE_PARSER_REG_DESC = "/*REGISTER"
TEMPLATE_PARSER_REG = "paramId"
TEMPLATE_PARSER_SEPARATOR = ";"

DEFAULT_TEMPLATE = "/**DEVICE_DESC;VendorName=PyModbus;ProductCode=PM;VendorUrl=http://github.com/bashwork/pymodbus;" \
                   "ProductName=PyModbus;ModelName=AsyncServer;MajorMinorRevision=1.0.0;sparse\n" \
                   "/**SIM_PORT;port=tcp:502;mode=tcp\n" \
                   "deviceId=1;networkId=1;plcBaseAddress=0;byteOrder=msb;wordOrder=msw\n" \
                   "/*REGISTER;paramId=1;Name=Pressure;Default=100;min=5;max=200\n" \
                   "paramId=1;deviceId=1;registerType=analog;address=1;encoding=int16\n" \
                   "/*REGISTER;paramId=2;Name=HighPressure;Default=150\n" \
                   "paramId=2;deviceId=1;registerType=holding;address=1;encoding=int16\n" \
                   "/*REGISTER;paramId=3;Name=Indicator;Default=0\n" \
                   "paramId=3;deviceId=1;registerType=input;address=1;encoding=boolean\n" \
                   "/*REGISTER;paramId=4;Name=Valve;Default=0\n" \
                   "paramId=4;deviceId=1;registerType=coil;address=2;encoding=boolean\n" \
                   "/*REGISTER;paramId=5;Name=Temperature;Default=22.5\n" \
                   "paramId=5;deviceId=1;registerType=analog;address=30;encoding=float32\n" \


class DeviceEndpoint(object):
    """
    Data model for an endpoint device such as sensor or actuator feeding into the simulated RTU/PLC

    .. todo::
       Not fully implemented
    """
    # From: https://github.com/SimplyAutomationized/modbus-tcp-tutorials/blob/master/tempSensors.py
    '''
    class Endpoint(object):
        def __init__(self, path, devicetype, callback):
            self.path = path
            self.devicetype = devicetype
            self.callback = callback
            self.value = 0

        def update(self):
            self.value = self.callback(self.path)


    def sensor_reading(sensor_path='some_path'):
        return 0


    def read_device_map():
        rootpath = ''
        devices = {
            0x0001: Endpoint(rootpath + 'unique_id', 'devicetype', sensor_reading)
        }
        return devices


    class CallbackDataBlock(ModbusSparseDataBlock):
        def __init__(self, devices):
            self.devices = devices
            self.devices[0xbeef] = len(self.devices)
            # self.get_device_values()
            self.values = {k: 0 for k in self.devices.iterkeys()}
            super(CallbackDataBlock, self).__init__(self.values)

        def get_device_value(self):
            devices = []
            devices_registers = filter(lambda d: d != 0xbeef, self.devices)
            for r in devices_registers:
                if self.devices[r].devicetype == 'my_device':
                    self.devices[r].register = r
                    devices.append(self.devices[r])
            t = threads.deferToThread(self.get_devices_values, devices)
            t.addCallback(self.update_devices_values)

        def get_devices_values(self, devices):
            values = {}
            for d in devices:
                d.update()
                values[d.register] = d.value
            return values

        def update_device_values(self, values):
            for register in values:
                self.values[register] = values[register]
            self.get_device_value()
    '''
    def __init__(self, parent, name, device_type, register_type, register_address, refresh_interval=None):
        self.parent = parent
        self.name = name
        self.device_types = ['sensor', 'actuator']
        if device_type in self.device_types:
            self.type = device_type
        else:
            raise EnvironmentError("Invalid device reg_type {type}".format(type=device_type))
        self.register_type = register_type   # 'di', 'co', 'ir', 'hr'
        self.register_address = register_address
        self.data_points = []
        self.config_params = []
        self.refresh_interval = refresh_interval

    def configure(self, **parameters):
        success = False
        return success

    def calibrate(self, **parameters):
        # TODO: model for calibration of values, return True if success else False
        success = False
        return success

    def read(self):
        # TODO: read line/simulator and update value; accommodate error cases
        return self.value

    def write(self):
        success = False
        if self.type == 'sensor':
            log.warning("Attempted to write to sensor")
        else:
            # TODO: set value of actuator
            pass
        return success

    def _update_register(self, value):
        success = False
        for reg in self.parent.registers:
            if reg.reg_type == self.register_type and reg.address == self.register_address:
                pass
        return success

    class DataPoint(object):
        def __init__(self, name, encoding, value, range_min, range_max, callback):
            self.name = name
            self.value = value
            self.range = range(range_min, range_max)
            self.encoding = encoding
            self.callback = callback

    class ConfigParameter(object):
        def __init__(self, name, value, operation):
            self.name = name
            self.value = value
            self.operation = operation


def get_simulator(vendor, model):
    """
    Simulators are expected to implement at least the following APIs, where register_type is from
    the list ['hr', 'ir', 'di', 'co]:

       * ``run`` a function that executes simulation
       * ``run_params`` optional kwargs passed into the run function
       * ``read`` a function to read a raw Modbus register based on register_type and address
       * ``write`` a function to write a raw Modbus register based on register_type and address

    :param vendor:
    :param model:
    :return:
    """
    sim = None
    if sim_weather_lufft.SIM_ID == (vendor, model):
        sim = {
            'module': sim_weather_lufft,
            'run': sim_weather_lufft.simulate,
            'run_params': {
                'log': log
            },
            'read': sim_weather_lufft.read_register,
            'write': sim_weather_lufft.write_register
        }
    return sim


class Slave(object):
    """
    Data model for a Modbus Slave (e.g. RTU, PLC)
    """
    def __init__(self, user_options):
        """
        :param user_options: object returned by the command line argument parser
        """
        self.template = user_options.template
        log.info("Simulating based on template: {}".format(user_options.template))
        self.identity = ModbusDeviceIdentification()
        self.port = user_options.port
        self.ser = None
        self.baudrate = user_options.baudrate
        self.bytesize = 8
        self.parity = 'none'
        self.stopbits = 1
        self.mode = user_options.mode
        self.slave_id = None
        self.zero_mode = True
        self.registers = []
        self.devices = []
        self.context = None
        self.sparse = False
        self.byteorder = Endian.Big
        self.wordorder = Endian.Big
        self.simulator = None
        self._parse_template()

    def _parse_template(self):
        """Parsing rules from template file to create Slave device"""
        if self.template == 'DEFAULT':
            template = DEFAULT_TEMPLATE
            lines = template.splitlines()
        else:
            if valid_path(self.template):
                with open(self.template) as f:
                    lines = f.readlines()
            else:
                raise ImportError("File name {filename} not found.".format(filename=self.template))
        for line in lines:
            if line[0:len(TEMPLATE_PARSER_DESC)] == TEMPLATE_PARSER_DESC:
                modbus_id = line.replace("/**", "").replace("/*", "").replace("*/", "").split(TEMPLATE_PARSER_SEPARATOR)
                for i in modbus_id:
                    if i[0:len('VendorName')] == 'VendorName':
                        self.identity.VendorName = i[len('VendorName')+1:].strip()
                    elif i[0:len('ProductCode')] == 'ProductCode':
                        self.identity.ProductCode = i[len('ProductCode')+1:].strip()
                    elif i[0:len('VendorUrl')] == 'VendorUrl':
                        self.identity.VendorUrl = i[len('VendorUrl')+1:].strip()
                    elif i[0:len('ProductName')] == 'ProductName':
                        self.identity.ProductName = i[len('ProductName')+1:].strip()
                    elif i[0:len('ModelName')] == 'ModelName':
                        self.identity.ModelName = i[len('ModelName')+1:].strip()
                    elif i[0:len('MajorMinorRevision')] == 'MajorMinorRevision':
                        self.identity.MajorMinorRevision = i[len('MajorMinorRevision')+1:].strip()
                    elif i[0:len('sparse')].lower() == 'sparse':
                        self.sparse = True
                if self.template == 'DEFAULT':
                    self.identity.MajorMinorRevision = pymodbus_version
                self.simulator = get_simulator(self.identity.VendorName, self.identity.ModelName)

            elif line[0:len(TEMPLATE_PARSER_NETWORK)] == TEMPLATE_PARSER_NETWORK:
                net_info = line.replace("/**", "").replace("/*", "").replace("*/", "").split(TEMPLATE_PARSER_SEPARATOR)
                for i in net_info:
                    if i[0:len('networkId')] == 'networkId':
                        net_id = int(i[len('networkId')+1:].strip())
                        if net_id in range(1, 254):
                            self.slave_id = int(i[len('networkId')+1:])
                        else:
                            log.error("Invalid Modbus Slave ID {id}".format(id=net_id))
                    elif i[0:len('plcBaseAddress')] == 'plcBaseAddress':
                        plc = int(i[len('plcBaseAddress')+1:].strip())
                        self.zero_mode = False if plc == 1 else True
                    elif i[0:len('byteOrder')] == 'byteOrder':
                        self.byteorder = Endian.Big if i[len('byteOrder')+1:].strip() == 'msb' else Endian.Little
                    elif i[0:len('wordOrder')] == 'wordOrder':
                        self.wordorder = Endian.Big if i[len('wordOrder')+1:].strip() == 'msw' else Endian.Little
                    # TODO: timeouts from template

            elif line[0:len(TEMPLATE_PARSER_SIM_PORT)] == TEMPLATE_PARSER_SIM_PORT:
                port_info = line.replace("/**", "").replace("/*", "").replace("*/", "").split(TEMPLATE_PARSER_SEPARATOR)
                for i in port_info:
                    if i[0:len('port')] == 'port':
                        port = i[len('port')+1:].strip()
                        log.info("port={port}".format(port=port))
                        if 'tcp' in port or 'udp' in port:
                            if port != self.port:
                                log.warning("Port mismatch: CLI={} but {}={}".format(self.port, self.template, port))
                            self.port = port
                        elif port in SerialPort.list_serial_ports():
                            self.port = port
                            self.ser = SerialPort(name=port)
                        else:
                            raise EnvironmentError("Undefined port {} read from {template}"
                                                   .format(port, template=self.template))
                    elif i[0:len('mode')] == 'mode':
                        self.mode = i[len('mode')+1:].strip()
                        log.info("mode={mode}".format(mode=self.mode))
                        if self.mode not in ['rtu', 'ascii', 'tcp']:
                            log.error("Undefined mode parsed from {template}".format(template=self.template))
                            raise EnvironmentError("Undefined Modbus mode read from {template}"
                                                   .format(template=self.template))
                    elif i[0:len('baudRate')] == 'baudRate':
                        baudrate = int(i[len('baudRate')+1:].strip())
                        log.info("baudRate={baudRate}".format(baudRate=baudrate))
                        if baudrate in [9600, 115200]:
                            if self.ser is None:
                                self.ser = SerialPort(name=self.port)
                            self.ser.baudrate = baudrate
                        else:
                            log.error("Unsupported baud rate {baud}".format(baud=str(baudrate)))
                    elif i[0:len('dataBits')] == 'dataBits':
                        bytesize = int(i[len('dataBits')+1:].strip())
                        log.info("dataBits={}".format(bytesize))
                        if bytesize in [7, 8]:
                            if self.ser is None:
                                self.ser = SerialPort(name=self.port)
                            self.ser.bytesize = bytesize
                        else:
                            log.error("Undefined dataBits {}".format(bytesize))
                    elif i[0:len('parity')] == 'parity':
                        parity = i[len('parity')+1:].strip()
                        log.info("parity={parity}".format(parity=parity))
                        if parity.lower() in ['none', 'even', 'odd']:
                            if parity.lower() == 'even':
                                parity = serial.PARITY_EVEN
                            elif parity.lower() == 'odd':
                                parity = serial.PARITY_ODD
                            else:
                                parity = serial.PARITY_NONE
                            if self.ser is None:
                                self.ser = SerialPort(name=self.port)
                            self.ser.parity = parity
                        else:
                            log.error("Undefined parity {parity}".format(parity=parity))
                    elif i[0:len('stopBits')] == 'stopBits':
                        stopbits = int(i[len('stopBits')+1:].strip())
                        log.info("stopBits={}".format(stopbits))
                        if stopbits in [1, 2]:
                            if self.ser is None:
                                self.ser = SerialPort(name=self.port)
                            self.ser.stopbits = stopbits
                        else:
                            log.error("Undefined stopBits {}".format(stopbits))

            elif line[0:len(TEMPLATE_PARSER_REG_DESC)] == TEMPLATE_PARSER_REG_DESC:
                reg = self.Register(context=self)
                reg_desc = line.replace("/**", "").replace("/*", "").replace("*/", "").split(TEMPLATE_PARSER_SEPARATOR)
                for i in reg_desc:
                    if i[0:len('paramId')] == 'paramId':
                        reg.paramId = int(i[len('paramId')+1:].strip())
                    elif i[0:len('address')].lower() == 'address':
                        addr = int(i[len('address') + 1:].strip())
                        if addr in range(0, 99999):
                            reg.address = addr
                        else:
                            log.error("Invalid Modbus address {num}".format(num=addr))
                    elif i[0:len('name')].lower() == 'name':
                        reg.name = i[len('name') + 1:].strip()
                    elif i[0:len('min')].lower() == 'min':
                        reg.min = i[len('min') + 1:].strip()
                    elif i[0:len('max')].lower() == 'max':
                        reg.max = i[len('max') + 1:].strip()
                    elif i[0:len('default')].lower() == 'default':
                        reg.default = i[len('default') + 1:].strip()
                self.registers.append(reg)

            elif line[0:len(TEMPLATE_PARSER_REG)] == TEMPLATE_PARSER_REG:
                # TODO: assign to proper objects, sort/group addresses by reg_type and min/max
                reg_config = line.replace("/**", "").replace("/*", "").replace("*/", "").split(TEMPLATE_PARSER_SEPARATOR)
                reg_exists = False
                this_reg = None
                for c in reg_config:
                    if c[0:len('paramId')] == 'paramId':
                        paramId = int(c[len('paramId')+1:].strip())
                        for reg in self.registers:
                            if paramId == reg.paramId:
                                reg_exists = True
                        if not reg_exists:
                            reg = self.Register(context=self.context, paramId=paramId)
                            self.registers.append(reg)
                            reg_exists = True
                        this_reg = paramId
                    elif c[0:len('address')] == 'address':
                        addr = int(c[len('address')+1:].strip())
                        if addr in range(0, 99999):
                            for reg in self.registers:
                                if reg.paramId == this_reg:
                                    reg.address = addr
                            if not reg_exists:
                                reg = self.Register(context=self.context, address=addr)
                                self.registers.append(reg)
                                reg_exists = True
                        else:
                            log.error("Invalid Modbus address {num}".format(num=addr))
                    elif c[0:len('registerType')] == 'registerType':
                        reg_type = c[len('registerType')+1:].strip()
                        if reg_type in ['analog']:
                            for reg in self.registers:
                                if reg.paramId == this_reg:
                                    reg.reg_type = 'ir'
                        elif reg_type in ['holding']:
                            for reg in self.registers:
                                if reg.paramId == this_reg:
                                    reg.reg_type = 'hr'
                        elif reg_type in ['input']:
                            for reg in self.registers:
                                if reg.paramId == this_reg:
                                    reg.reg_type = 'di'
                        elif reg_type in ['coil']:
                            for reg in self.registers:
                                if reg.paramId == this_reg:
                                    reg.reg_type = 'co'
                        else:
                            log.error("Unsupported registerType {type}".format(type=reg_type))
                    elif c[0:len('encoding')] == 'encoding':
                        enc = c[len('encoding')+1:].strip()
                        if enc in ['int16', 'int8', 'boolean']:
                            for reg in self.registers:
                                if reg.paramId == this_reg:
                                    reg.encoding = enc
                                    reg.default = int(reg.default)
                                    reg.min = int(reg.min) if reg.min is not None else None
                                    reg.max = int(reg.max) if reg.max is not None else None
                        elif enc in ['float32', 'int32']:
                            for reg in self.registers:
                                if reg.paramId == this_reg:
                                    reg.encoding = enc
                                    reg.length = 2
                                    reg.default = float(reg.default) if enc == 'float32' else int(reg.default)
                                    if reg.min is not None:
                                        reg.min = float(reg.min) if enc == 'float32' else int(reg.min)
                                    if reg.max is not None:
                                        reg.max = float(reg.max) if enc == 'float32' else int(reg.max)
                        else:
                            log.error("Unsupported encoding {type}".format(type=enc))
        hr_sequential = []
        hr_sparse_block = {}
        ir_sequential = []
        ir_sparse_block = {}
        di_sequential = []
        di_sparse_block = {}
        co_sequential = []
        co_sparse_block = {}
        for reg in self.registers:
            if reg.min is None:
                reg.min = reg.get_range()[0]
            if reg.max is None:
                reg.max = reg.get_range()[1]
            reg.default = reg.get_default()
            if reg.reg_type == 'hr' and reg.address is not None:
                if self.sparse:
                    hr_sparse_block[reg.address] = 0
                else:
                    hr_sequential.append(reg.address)
            elif reg.reg_type == 'ir' and reg.address is not None:
                if self.sparse:
                    ir_sparse_block[reg.address] = 0
                else:
                    ir_sequential.append(reg.address)
            elif reg.reg_type == 'di' and reg.address is not None:
                if self.sparse:
                    di_sparse_block[reg.address] = 0
                else:
                    di_sequential.append(reg.address)
            elif reg.reg_type == 'co' and reg.address is not None:
                if self.sparse:
                    co_sparse_block[reg.address] = 0
                else:
                    co_sequential.append(reg.address)
            else:
                log.error("Unhandled exception register {reg} addr={addr}".format(reg=reg.name, addr=str(reg.address)))
        if self.sparse:
            hr_block = ModbusSparseDataBlock(hr_sparse_block) if len(hr_sparse_block) > 0 else None
            ir_block = ModbusSparseDataBlock(ir_sparse_block) if len(ir_sparse_block) > 0 else None
            di_block = ModbusSparseDataBlock(di_sparse_block) if len(di_sparse_block) > 0 else None
            co_block = ModbusSparseDataBlock(co_sparse_block) if len(co_sparse_block) > 0 else None
        else:
            if len(hr_sequential) > 0:
                hr_sequential.sort()
                hr_block = ModbusSequentialDataBlock(0, [0 for x in range(hr_sequential[0],
                                                                          hr_sequential[len(hr_sequential)-1])])
            else:
                hr_block = None
            if len(ir_sequential) > 0:
                ir_sequential.sort()
                ir_block = ModbusSequentialDataBlock(0, [0 for x in range(ir_sequential[0],
                                                                          ir_sequential[len(ir_sequential)-1])])
            else:
                ir_block = None
            if len(di_sequential) > 0:
                di_sequential.sort()
                di_block = ModbusSequentialDataBlock(0, [0 for x in range(di_sequential[0],
                                                                          di_sequential[len(di_sequential)-1])])
            else:
                di_block = None
            if len(co_sequential) > 0:
                co_sequential.sort()
                co_block = ModbusSequentialDataBlock(0, [0 for x in range(co_sequential[0],
                                                                          co_sequential[len(co_sequential)-1])])
            else:
                co_block = None
        self.context = ModbusSlaveContext(hr=hr_block, ir=ir_block, di=di_block, co=co_block, zero_mode=self.zero_mode)
        # initialize default values
        for reg in self.registers:
            reg.context = self.context
            reg.set_value(reg.default)

    class Register(object):
        """
        An abstraction layer with metadata for the Modbus registers in the Slave context

        .. todo::
           docstring
        """
        def __init__(self, context, paramId=None, address=None, length=1, name=None, reg_type=None, encoding=None,
                     default=0, min=None, max=None, byteorder=Endian.Big, wordorder=Endian.Big):
            """
            Initializes a new Register object

            :param pymodbus.ModbusSlaveContext context: the slave context
            :param int paramId: the parameter ID derived from the Inmarsat config.dat file
            :param int address: the Modbus register address
            :param int length: the number of registers if multi-register (e.g. 2 for float32)
            :param str name: the abstract parameter name derived from the Inmarsat config.dat file
            :param str reg_type: the register type from ['hr', 'ir', 'di', 'co']
            :param str encoding: the data encoding e.g. ['uint16', 'boolean']
            :param default: the default value
            :param min: the minimum valid value
            :param max: the maximum valid value
            :param byteorder: the byte order from [Endian.Big, Endian.Little]
            :param wordorder: the word order from [Endian.Big, Endian.Little]
            """
            self.context = context
            self.paramId = paramId
            self.address = address
            self.length = length
            self.name = name
            self.reg_type = reg_type
            self.encoding = encoding
            self.default = default
            self.min = min if min is not None else self.get_range()[0]
            self.max = max if max is not None else self.get_range()[1]
            self.byteorder = byteorder
            self.wordorder = wordorder
            self.value = None

        def get_range(self):
            """Gets nominal max/min ranges based on encoding reg_type"""
            min = None
            max = None
            if self.encoding is not None:
                if 'uint' in self.encoding[0:len('uint')]:
                    min = 0
                    bits = int(self.encoding[len('uint'):])
                    max = 2**bits - 1
                elif 'int' in self.encoding[0:len('int')]:
                    bits = int(self.encoding[len('int'):])
                    min = -(2**bits/2)
                    max = (2**bits/2) - 1
                elif self.encoding == 'boolean':
                    min = 0
                    max = 1
            return min, max

        def get_default(self):
            """Returns the default value with the correct encoding"""
            default = self.default
            if self.encoding is not None:
                if 'int' in self.encoding[0:len('uint')]:
                    default = int(self.default) if self.default is not None else 0
                elif 'float' in self.encoding[0:len('float')]:
                    default = float(self.default) if self.default is not None else 0.0
                elif 'bool' in self.encoding[0:len('bool')]:
                    default = bool(self.default) if self.default is not None else False
                elif 'ascii' in self.encoding[0:len('ascii')]:
                    default = str(self.default) if self.default is not None else ""
            return default

        def get_function_code(self, read=True):
            """
            Gets the Modbus function code for the register reg_type and read/write operation.

            :param read: read/write
            :return: Modbus function code
            """
            if self.reg_type in ['hr', 'co']:
                if read:
                    return READ_HR if self.reg_type == 'hr' else READ_CO
                else:
                    return WRITE_SINGLE_HR if self.reg_type == 'hr' else WRITE_SINGLE_CO
            elif self.reg_type in ['ir', 'di']:
                if read:
                    return READ_IR if self.reg_type == 'ir' else READ_DI
                else:
                    raise EnvironmentError("Illegal operation cannot write to {type}".format(type=self.reg_type))

        def get_value(self):
            """
            :return: the value of the register
            """
            values = self.context.getValues(self.get_function_code(), self.address, self.length)
            decoder = BinaryPayloadDecoder.fromRegisters(registers=values,
                                                         byteorder=self.byteorder, wordorder=self.wordorder)
            if self.encoding in ['int8', 'uint8', 'boolean']:
                decoded = decoder.decode_8bit_int() if self.encoding == 'int8' else decoder.decode_8bit_uint()
            elif self.encoding in ['int16', 'uint16']:
                decoded = decoder.decode_16bit_int() if self.encoding == 'int16' else decoder.decode_16bit_uint()
            elif self.encoding in ['int32', 'uint32']:
                decoded = decoder.decode_32bit_int() if self.encoding == 'int32' else decoder.decode_32bit_uint()
            elif self.encoding in ['float32', 'float64']:
                decoded = decoder.decode_32bit_float() if self.encoding == 'float32' else decoder.decode_64bit_float()
            elif self.encoding in ['int64', 'uint64']:
                decoded = decoder.decode_64bit_int() if self.encoding == 'int64' else decoder.decode_64bit_uint()
            # Not using bits for boolean due to padding by codec operation
            elif self.encoding == 'bits':
                decoded = decoder.decode_bits()
            elif self.encoding in ['string', 'ascii']:
                decoded = decoder.decode_string()
            else:
                log.error("Unhandled encoding exception {enc}".format(enc=self.encoding))
                decoded = None
            # log.debug("Read {reg_type} {addr} as {enc} {val} from {list}".format(reg_type=self.reg_type, addr=self.address,
            #                                                                  enc=self.encoding, val=decoded,
            #                                                                  list=str(values)))
            self.value = decoded
            return decoded

        def set_value(self, value):
            """
            Set the value of the register

            :param value:
            """
            if value is not None:
                builder = BinaryPayloadBuilder(byteorder=self.byteorder, wordorder=self.wordorder)
                if self.encoding in ['int8', 'uint8', 'boolean']:
                    builder.add_8bit_int(value) if self.encoding == 'int8' else builder.add_8bit_uint(value)
                elif self.encoding in ['int16', 'uint16']:
                    builder.add_16bit_int(value) if self.encoding == 'int16' else builder.add_16bit_uint(value)
                elif self.encoding in ['int32', 'uint32']:
                    builder.add_32bit_int(value) if self.encoding == 'int32' else builder.add_32bit_uint(value)
                elif self.encoding in ['float32', 'float64']:
                    builder.add_32bit_float(value) if self.encoding == 'float32' else builder.add_64bit_float(value)
                elif self.encoding in ['int64', 'uint64']:
                    builder.add_64bit_int(value) if self.encoding == 'int64' else builder.add_64bit_uint(value)
                # Not using bits for boolean due to padding by codec operation
                elif self.encoding == 'bits':
                    builder.add_bits([value])
                elif self.encoding in ['string', 'ascii']:
                    builder.add_string(value)
                else:
                    log.error("Unhandled encoding exception {enc}".format(enc=self.encoding))
                payload = builder.to_registers()
                # log.debug("Setting {reg_type} {addr} to {enc} {val} as {list}".format(reg_type=self.reg_type, addr=self.address,
                #                                                                   enc=self.encoding, val=value,
                #                                                                   list=str(payload)))
                self.context.setValues(self.get_function_code(), self.address, payload)
                self.value = value
            else:
                log.warning("Attempt to set {type} {addr} to None (default={default})".format(type=self.reg_type,
                                                                                              addr=self.address,
                                                                                              default=self.default))


def valid_path(filename):
    """
    Validates a file path on local os or URL-based

    :param filename: (string) to be validated
    :return: {Boolean} result
    """
    if 'http://' in filename or 'https://' in filename:
        if 'http://' in filename:
            c = httplib.HTTPConnection(filename)
        else:
            c = httplib.HTTPSConnection(filename)
        c.request('HEAD', '')
        if c.getresponse().status == 200:
            return True
        else:
            return False
    if filename[0:3] == "../":
        filename = filename[3:]
    if os.path.exists(filename)or os.path.exists(os.path.join(os.path.dirname(__file__), filename)):
        return True
    else:
        return False


def update_values(server_context, slaves):
    """
    Updates the configured register values in the Modbus context.
    Increments or toggles values

    .. todo::

       * Proper striping of large values across 16-bit registers.

    :param pymodbus.ModbusServerContext server_context: (unused) a server context object
    :param Slave slaves: a list of ``Slave`` objects
    """
    # context = server_context
    for slave in slaves:
        if slave.simulator is None:
            for reg in slave.registers:
                old_value = reg.get_value()
                if reg.reg_type in ['hr', 'ir']:
                    if reg.max is None or old_value < reg.max:
                        new_value = old_value + 1
                    else:
                        new_value = reg.min
                else:
                    new_value = 0 if old_value == 1 else 1
                reg.set_value(new_value)
                log.debug("Reg={} Name={} Old={} New={} Max={} Min={}"
                          .format(reg.address, reg.name, old_value, new_value, reg.max, reg.min))
        else:
            for reg in slave.registers:
                old_value = reg.get_value()
                new_value = slave.simulator['read'](reg_type=reg.reg_type, address=reg.address)
                if new_value != old_value:
                    reg.set_value(new_value)
                    log.debug("New simulation value for {} old={} new={}".format(reg.name, old_value, new_value))


class SerialPort(object):
    """
    Setup and metadata for a serial port used for Modbus
    """
    def __init__(self, name='/dev/ttyUSB0', baudrate=9600, bytesize=8, parity=serial.PARITY_EVEN, stopbits=1):
        if name in self.list_serial_ports():
            self.name = name
            self.baudrate = baudrate if baudrate in self.supported_baudrates() else 9600
            self.bytesize = bytesize
            self.parity = parity
            self.stopbits = stopbits
            self.timeout = None
            self.write_timeout = None
            self.inter_byte_timeout = None
            self.xonxoff = False
            self.rtscts = False
            self.dsrdtr = False
        else:
            raise EnvironmentError("Unable to find specified serial name {port}".format(port=name))

    def set_bps(self, shorthand):
        """
        Set the baud, parity and stop-bits

        :param str shorthand: format e.g. 9600-8N1
        :return:
        """
        baudrate = int(shorthand.split("-")[0])
        framing = shorthand.split("-")[1].upper()
        if baudrate in self.supported_baudrates():
            self.baudrate = baudrate
        else:
            log.error("Attempt to configure unsupported baud rate {num}".format(num=baudrate))
        if framing == '8N1':
            self.bytesize = 8
            self.parity = 'none'
            self.stopbits = 1
        else:
            log.error("Attempt to configure unsupported framing {config}".format(config=framing))

    @staticmethod
    def supported_baudrates():
        """Returns the supported baud rates"""
        return [2400, 4800, 9600, 19200, 38400, 57600, 115200]

    @staticmethod
    def list_serial_ports():
        """
        Lists serial port names.
        :raises EnvironmentError: On unsupported or unknown platforms
        :returns: A list of the serial ports available on the system
        """
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError("Unsupported OS/platform")
        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result


def get_parser():
    """
    Parses the command line arguments.

    :returns: a parser with command line arguments
    :rtype: argparse.ArgumentParser
    """
    parser = argparse.ArgumentParser(description="Modbus Slave Endpoint.")

    port_choices = SerialPort.list_serial_ports() + ['tcp:502', 'udp:5020']

    parser.add_argument('-t', '--template', dest='template', default='DEFAULT',
                        help="the template file to use")

    parser.add_argument('-p', '--port', dest='port', default=PORT_DEFAULT,
                        choices=port_choices,
                        help="tcp:502, udp:5020, or a USB/serial port name")

    parser.add_argument('-b', '--baud', dest='baudrate', default=9600, type=int,
                        choices=[2400, 4800, 9600, 19200, 38400, 57600, 115200],
                        help="baud rate (``int`` default 9600)", metavar="{2400..115200}")

    # parser.add_argument('-f', '--framing', dest='framing', default='8N1',
    #                     choices=['8N1'],
    #                     help="serial port framing (byte size, parity, stop bits)")
    #
    parser.add_argument('-m', '--mode', dest='mode', default=None,
                        choices=['rtu', 'ascii', 'tcp'],
                        help="Modbus framing mode RTU, ASCII or TCP")

    parser.add_argument('--logfile', default=None,
                        help="the log file name with optional extension (default extension .log)")

    parser.add_argument('--logsize', type=int, default=5,
                        help="the maximum log file size, in MB (default 5 MB)")

    parser.add_argument('--debug', action='store_true',
                        help="enable tick_log debug logging (default OFF)")

    return parser


def run_async_server(update_interval=10):
    """
    Runs the Modbus asynchronous server

    :param int update_interval: the refresh interval for simulated data, in seconds
    """
    global active
    slave_updater = None
    sim_thread = None
    try:
        parser = get_parser()
        user_options = parser.parse_args()

        slave = Slave(user_options)
        slaves = {
            slave.slave_id: slave.context
        }
        context = ModbusServerContext(slaves=slaves, single=False)

        if slave.simulator is not None:
            log.info("Simulating {} {}".format(slave.identity.VendorName, slave.identity.ModelName))
            sim_thread = threading.Thread(target=slave.simulator['run'], name="rtu_simulator",
                                          kwargs=slave.simulator['run_params'])
            sim_thread.setDaemon(True)
            sim_thread.start()

        # Set up looping call to update values
        slave_list = [slave]
        updater_args = {'server_context': context, 'slaves': slave_list}
        slave_updater = RepeatingTimer(seconds=update_interval, name='slave_updater', defer=False,
                                       callback=update_values, **updater_args)
        slave_updater.start_timer()

        if slave.mode == 'tcp':
            framer = ModbusSocketFramer
        elif slave.mode == 'ascii':
            framer = ModbusAsciiFramer
        else:
            framer = ModbusRtuFramer

        # TODO: trap master connect/disconnect as INFO logs rather than DEBUG (default of pyModbus)
        if 'tcp' in slave.port:
            if len(slave.port.split(':')) > 1 and int(slave.port.split(':')[1]) in range(0, 65535+1):
                tcp_port = int(slave.port.split(':')[1])
            else:
                tcp_port = 502
            StartTcpServer(context, identity=slave.identity, address=("localhost", tcp_port), framer=framer,
                           defer_reactor_run=False)
        elif 'udp' in slave.port:
            if len(slave.port.split(':')) > 1 and int(slave.port.split(':')[1]) in range(0, 65535+1):
                udp_port = int(slave.port.split(':')[1])
            else:
                udp_port = 5020
            StartUdpServer(context, identity=slave.identity, address=("127.0.0.1", udp_port), framer=framer,
                           defer_reactor_run=False)
        else:
            log.debug("serial settings: {}".format(vars(slave.ser)))
            StartSerialServer(context, identity=slave.identity, framer=framer,
                              port=slave.ser.name,
                              baudrate=slave.ser.baudrate,
                              bytesize=slave.ser.bytesize,
                              parity=slave.ser.parity,
                              stopbits=slave.ser.stopbits,
                              defer_reactor_run=False)

    except KeyboardInterrupt, e:
        log.warning("Execution stopped by keyboard interrupt: {}".format(e))

    except Exception, e:
        log.error("{}".format(e))

    finally:
        print("********************** EXCEPTION OCCURRED *********************")
        print("Attempting to stop async server")
        if slave_updater is not None:
            print("slave_updater terminating")
            slave_updater.stop_timer()
            slave_updater.terminate()
            slave_updater.join()
        elif sim_thread is not None:
            sim_thread.join()
        sys.exit(0)


if __name__ == "__main__":
    run_async_server()
