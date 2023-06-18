from enum import Enum
import time
import serial
import struct
import re


class CvCmdHandler:
    # misc constants
    DATA_PACKAGE_SIZE = 15
    DATA_PAYLOAD_INDEX = 3
    MIN_TX_SEPARATION_SEC = 0  # reserved for future, currently control board is fast enough

    class eMsgType(Enum):
        MSG_MODE_CONTROL = b'\x10'
        MSG_CV_CMD = b'\x20'
        MSG_ACK = b'\x40'

    class eSepChar(Enum):  # start and ending hexes, acknowledgement bit
        CHAR_HEADER = b'>>'
        ACK_ASCII = b'ACK'
        CHAR_UNUSED = b'\xFF'

    class eRxState(Enum):
        RX_STATE_INIT = 0
        RX_STATE_WAIT_FOR_ETX = 1
        RX_STATE_SEND_ACK = 2

    class eModeControlBits(Enum):
        MODE_AUTO_AIM_BIT = 0b00000001
        MODE_AUTO_MOVE_BIT = 0b00000010
        MODE_ENEMY_DETECTED_BIT = 0b00000100

    def __init__(self):
        self.Rx_State = self.eRxState.RX_STATE_INIT
        self.AutoAimSwitch = False
        self.AutoMoveSwitch = False
        self.EnemySwitch = False
        self.rxSwitchBuffer = 0
        self.prevTxTime = 0

        # self.ser = serial.Serial(port='/dev/ttyTHS2', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
        # Manual test on Windows
        self.ser = serial.Serial(port='COM8', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)

        self.txCvCmdMsg = bytearray(self.eSepChar.CHAR_HEADER.value + self.eMsgType.MSG_CV_CMD.value + self.eSepChar.CHAR_UNUSED.value*12)
        # txAckMsg is always the same, so use the immutable bytes object
        self.txAckMsg = b''.join([self.eSepChar.CHAR_HEADER.value, self.eMsgType.MSG_ACK.value, self.eSepChar.ACK_ASCII.value, self.eSepChar.CHAR_UNUSED.value*9])
        assert (len(self.txCvCmdMsg) == self.DATA_PACKAGE_SIZE)
        assert (len(self.txAckMsg) == self.DATA_PACKAGE_SIZE)

    def CvCmd_Reset(self):
        self.Rx_State = self.eRxState.RX_STATE_INIT

    # @brief main API function
    # @param[in] gimbal_coordinate_x and gimbal_coordinate_y: type is int; will be converted to int16_t
    # @param[in] chassis_speed_x and chassis_speed_y: type is float; can be positive/negative; will be converted to float (32 bits)
    def CvCmd_Heartbeat(self, gimbal_coordinate_x, gimbal_coordinate_y, chassis_speed_x, chassis_speed_y):
        # CV positive directions: +x is to the right, +y is upwards
        # Remote controller positive directions: +x is upwards, +y is to the left
        gimbal_coordinate_x, gimbal_coordinate_y = gimbal_coordinate_y, gimbal_coordinate_x
        chassis_speed_x, chassis_speed_y = chassis_speed_y, chassis_speed_x
        chassis_speed_y = -chassis_speed_y

        # Tx
        if (self.AutoAimSwitch or self.AutoMoveSwitch) and (time.time() - self.prevTxTime > self.MIN_TX_SEPARATION_SEC):
            self.txCvCmdMsg[self.DATA_PAYLOAD_INDEX:self.DATA_PAYLOAD_INDEX+12] = b''.join([gimbal_coordinate_x.to_bytes(2, 'little'), gimbal_coordinate_y.to_bytes(2, 'little'), struct.pack('<f', chassis_speed_x), struct.pack('<f', chassis_speed_y)])
            self.ser.write(self.txCvCmdMsg)
            self.prevTxTime = time.time()

        # Rx
        fHeartbeatFinished = False
        while fHeartbeatFinished == False:
            fHeartbeatFinished = self.CvCmd_RxHeartbeat()

        return (self.AutoAimSwitch, self.AutoMoveSwitch, self.EnemySwitch)

    def CvCmd_RxHeartbeat(self):
        fHeartbeatFinished = True

        if self.Rx_State == self.eRxState.RX_STATE_INIT:
            self.AutoAimSwitch = False
            self.AutoMoveSwitch = False
            self.EnemySwitch = False

            if not self.ser.is_open:
                self.ser.open()
            # control board sends many garbage data when it restarts, so clean buffer here
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            # print("Reactor online. Sensors online. Weapons online. All systems nominal.\n")
            self.Rx_State = self.eRxState.RX_STATE_WAIT_FOR_ETX
            fHeartbeatFinished = True

        elif self.Rx_State == self.eRxState.RX_STATE_WAIT_FOR_ETX:
            # polling for control msg, if any msg received, ACK back
            if self.ser.in_waiting >= self.DATA_PACKAGE_SIZE:
                bytesRead = self.ser.read(self.ser.in_waiting)
                dataPackets = re.findall(self.eSepChar.CHAR_HEADER.value + self.eMsgType.MSG_MODE_CONTROL.value + b"." + self.eSepChar.CHAR_UNUSED.value + b"{11}", bytesRead)
                if dataPackets:
                    # read the mode of the last packet, because it's the latest
                    self.rxSwitchBuffer = dataPackets[-1][self.DATA_PAYLOAD_INDEX]
                    self.AutoAimSwitch = bool(self.rxSwitchBuffer & self.eModeControlBits.MODE_AUTO_AIM_BIT.value)
                    self.AutoMoveSwitch = bool(self.rxSwitchBuffer & self.eModeControlBits.MODE_AUTO_MOVE_BIT.value)
                    self.EnemySwitch = bool(self.rxSwitchBuffer & self.eModeControlBits.MODE_ENEMY_DETECTED_BIT.value)
                    self.Rx_State = self.eRxState.RX_STATE_SEND_ACK
                    fHeartbeatFinished = False
                else:
                    self.ser.reset_input_buffer()
                    fHeartbeatFinished = True

        elif self.Rx_State == self.eRxState.RX_STATE_SEND_ACK:
            if time.time() - self.prevTxTime > self.MIN_TX_SEPARATION_SEC:
                self.ser.write(self.txAckMsg)
                self.prevTxTime = time.time()
                self.Rx_State = self.eRxState.RX_STATE_WAIT_FOR_ETX
            fHeartbeatFinished = True

        return fHeartbeatFinished

## Example usage
# CvCmder = CvCmdHandler()
# oldflags = (False, False, False)
# while True:
#     flags = CvCmder.CvCmd_Heartbeat(0, 0, 0, 0)  # gimbal_coordinate_x, gimbal_coordinate_y, chassis_speed_x, chassis_speed_y
#     if flags != oldflags:
#         oldflags = flags
#         print(flags)
