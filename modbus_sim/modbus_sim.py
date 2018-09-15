#!/usr/bin/env python
"""
A Modbus slave device used to simulate various RTU/PLC serial devices for testing.

.. todo::
   Import mbs slave definitions built by `Modbus Slave <http://www.modbustools.com>`_ tool.

"""

__version__ = "0.1.0"

import sys
import logging
import argparse
import serial
import glob

import headless
from headless import RepeatingTimer

from pymodbus import __version__ as py_ver
from pymodbus.server.async import StartTcpServer
from pymodbus.server.async import StartUdpServer
from pymodbus.server.async import StartSerialServer

from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer, ModbusSocketFramer

# Serial Port Defaults
# serial_port = '/dev/ttyUSB0'
# serial_baud = 9600
# serial_bytesize = serial.EIGHTBITS
# serial_parity = serial.PARITY_NONE
# serial_stopbits = serial.STOPBITS_ONE

# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
# FORMAT = ('%(asctime)-15s %(threadName)-15s'
#           ' %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
# logging.basicConfig(format=FORMAT)
# log = logging.getLogger()
# log.setLevel(logging.DEBUG)
log = headless.get_wrapping_log(debug=True)

# Modbus function code reference:
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


class Device(object):
    def __init__(self):
        pass

    def something(self):
        pass


class Slave(object):
    """
    Data model for a Modbus Slave (e.g. RTU, PLC)

    .. todo::
       NOT IMPLEMENTED

    :param identity: ``pymodbus.device.ModbusDeviceIdentification`` object
    :param slave_id: (number) Network ID of the slave
    :param register_map: ``pymodbus.datastore.ModbusSlaveContext`` object with registers and default values

       - ``ModbusSequentialDataBlock`` or ``ModbusSparseDataBlock``
       - assigned to ``di`` (Digital Inputs), ``co`` (Coils), ``ir`` (Input Registers), ``hr`` (Holding Registers)

    :param devices: List of ``modbusslave.Device`` objects

    """
    def __init__(self, identity=None, slave_id=1, zero_mode=False, register_map=None, devices=None):
        """
        See Slave docstring

        :param identity:
        :param slave_id:
        :param zero_mode:
        :param register_map:
        :param devices:
        """
        if identity is None:
            identity = ModbusDeviceIdentification()
            identity.VendorName = 'Pymodbus'
            identity.ProductCode = 'PM'
            identity.VendorUrl = 'http://github.com/bashwork/pymodbus/'
            identity.ProductName = 'Pymodbus Server'
            identity.ModelName = 'Pymodbus Server'
            identity.MajorMinorRevision = '1.5'
        self.identity = identity
        self.slave_id = slave_id
        self.zero_mode = zero_mode
        if register_map is None:
            register_map = ModbusSlaveContext(co=ModbusSequentialDataBlock.create(),
                                              di=ModbusSequentialDataBlock.create(),
                                              ir=ModbusSequentialDataBlock.create(),
                                              hr=ModbusSequentialDataBlock.create(),
                                              zero_mode=zero_mode)
        self.register_map = register_map
        if devices is None:
            devices = [Device()]
        self.devices = devices


# From: https://github.com/SimplyAutomationized/modbus-tcp-tutorials/blob/master/tempSensors.py
'''
class Device(object):
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
        0x0001: Device(rootpath + 'unique_id', 'devicetype', sensor_reading)
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


def get_slave_context(device='Default'):
    log.debug("Getting slave context")
    supported_devices = ['Default']
    default_hr_value = 3
    default_ir_value = 2
    default_di_value = 1
    default_co_value = 0
    if device in supported_devices:
        # TODO: Create reference/test devices
        hr_block = ModbusSequentialDataBlock(0, [default_hr_value for x in range(10000)])
        ir_block = ModbusSequentialDataBlock(0, [default_ir_value for x in range(10000)])
        di_block = ModbusSequentialDataBlock(0, [default_di_value for x in range(10000)])
        co_block = ModbusSequentialDataBlock(0, [default_co_value for x in range(10000)])
        return ModbusSlaveContext(hr=hr_block, ir=ir_block, di=di_block, co=co_block)
    else:
        raise NotImplementedError('Target Modbus device not supported.')


def update_values(server_contexts):
    """
    Updates the configured register values in the Modbus context

    .. todo::
       INCOMPLETE, scale up and support device templates

    :param server_contexts: a ``ModbusServerContext``

    """
    context = server_contexts[0]
    function_code = 0x03
    slave_id = 0x01
    address = 0x00
    count = 1
    get_values = context[slave_id].getValues(function_code, address, count=count)
    set_values = [v + 1 for v in get_values]
    log.debug("Modbus slave updating. New values:" + str(set_values))
    context[slave_id].setValues(function_code, address, set_values)   # the index here confuses me!?


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
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


class SerialPort(object):
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, bytesize=8, parity='none', stopbits=1):
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits


def get_parser():
    """
    Parses the command line arguments.

    :returns: An argparse.ArgumentParser

    """
    parser = argparse.ArgumentParser(description="Modbus Slave Device.")

    # TODO: determine environment to set defaults e.g. /dev/ttyUSB0 for RPi, TCP for PC
    # PORT_DEFAULT = '/dev/ttyUSB0'   # Raspberry Pi
    PORT_DEFAULT = 'tcp'
    port_choices = list_serial_ports() + ['tcp', 'udp']

    parser.add_argument('-p', '--port', dest='port', default=PORT_DEFAULT,
                        choices=port_choices,
                        help="tcp:502, udp:5020, or a USB/serial port name")

    parser.add_argument('-b', '--baud', default=9600, type=int,
                        choices=[2400, 4800, 9600, 19200, 38400, 57600, 115200],
                        help="baud rate (``int`` default 9600)", metavar="{2400..115200}")

    parser.add_argument('-f', '--framing', dest='framing', default='8N1',
                        choices=['8N1'],
                        help="serial port framing (byte size, parity, stop bits)")

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


def run_async_server():
    """
    Runs the Modbus asynchronous server
    """
    parser = get_parser()
    user_options = parser.parse_args()
    TCP_PORT = 502
    UDP_PORT = 5020
    # ----------------------------------------------------------------------- #
    # initialize your data store
    # ----------------------------------------------------------------------- #
    # The datastores only respond to the addresses that they are initialized to
    # Therefore, if you initialize a DataBlock to addresses from 0x00 to 0xFF,
    # a request to 0x100 will respond with an invalid address exception.
    # This is because many devices exhibit this kind of behavior (but not all)
    #
    #     block = ModbusSequentialDataBlock(0x00, [0]*0xff)
    #
    # Continuing, you can choose to use a sequential or a sparse DataBlock in
    # your data context.  The difference is that the sequential has no gaps in
    # the data while the sparse can. Once again, there are devices that exhibit
    # both forms of behavior::
    #
    #     block = ModbusSparseDataBlock({0x00: 0, 0x05: 1})
    #     block = ModbusSequentialDataBlock(0x00, [0]*5)
    #
    # Alternately, you can use the factory methods to initialize the DataBlocks
    # or simply do not pass them to have them initialized to 0x00 on the full
    # address range::
    #
    #     store = ModbusSlaveContext(di = ModbusSequentialDataBlock.create())
    #     store = ModbusSlaveContext()
    #
    # Finally, you are allowed to use the same DataBlock reference for every
    # table or you you may use a separate DataBlock for each table.
    # This depends if you would like functions to be able to access and modify
    # the same data or not::
    #
    #     block = ModbusSequentialDataBlock(0x00, [0]*0xff)
    #     store = ModbusSlaveContext(di=block, co=block, hr=block, ir=block)
    #
    # The server then makes use of a server context that allows the server to
    # respond with different slave contexts for different unit ids. By default
    # it will return the same context for every unit id supplied (broadcast
    # mode).
    # However, this can be overloaded by setting the single flag to False
    # and then supplying a dictionary of unit id to context mapping::
    #
    #     slaves  = {
    #         0x01: ModbusSlaveContext(...),
    #         0x02: ModbusSlaveContext(...),
    #         0x03: ModbusSlaveContext(...),
    #     }
    #     context = ModbusServerContext(slaves=slaves, single=False)
    #
    # The slave context can also be initialized in zero_mode which means that a
    # request to address(0-7) will map to the address (0-7). The default is
    # False which is based on section 4.4 of the specification, so address(0-7)
    # will map to (1-8)::
    #
    #     store = ModbusSlaveContext(..., zero_mode=True)
    # ----------------------------------------------------------------------- #

    slaves = {
        0x01: get_slave_context(),
    }
    context = ModbusServerContext(slaves=slaves, single=False)
    # slaves = get_slave_context()
    # context = ModbusServerContext(slaves=slaves, single=True)

    # initialize the server information (else defaulted to empty strings)
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'PyModbus'
    identity.ProductCode = 'PM'
    identity.VendorUrl = 'http://github.com/bashwork/pymodbus/'
    identity.ProductName = 'Pymodbus Server'
    identity.ModelName = 'Pymodbus Server'
    identity.MajorMinorRevision = py_ver

    # Set up looping call to update values
    UPDATE_INTERVAL = 5
    slave_updater = RepeatingTimer(seconds=UPDATE_INTERVAL, name='slave_updater',
                                   callback=update_values, args=(context,), defer=False)
    slave_updater.start_timer()

    if user_options.mode == 'tcp' or user_options.mode is None and user_options.port.lower() == 'tcp':
        framer = ModbusSocketFramer
    elif user_options.mode == 'ascii':
        framer = ModbusAsciiFramer
    else:
        framer = ModbusRtuFramer

    if user_options.port.lower() == 'tcp':
        StartTcpServer(context, identity=identity, address=("localhost", TCP_PORT), framer=framer)
    elif user_options.port.lower() == 'udp':
        StartUdpServer(context, identity=identity, address=("127.0.0.1", UDP_PORT), framer=framer)
    else:
        if user_options.port in list_serial_ports():
            ser = SerialPort()
            StartSerialServer(context, identity=identity, framer=framer,
                              port=ser.port,
                              baudrate=ser.baudrate,
                              bytesize=ser.bytesize,
                              parity=ser.parity,
                              stopbits=ser.stopbits,)  # need timeout=X?
        else:
            raise EnvironmentError('Unable to find specified serial port ' + user_options.port)
    # '''


if __name__ == "__main__":
    run_async_server()
