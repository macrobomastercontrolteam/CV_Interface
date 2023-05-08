from enum import Enum
import time
import serial
import struct


class CvCmdHandler:
    # misc constants
    DATA_PACKAGE_SIZE = 15
    DATA_PAYLOAD_INDEX = 2

    class eMsgType(Enum):
        MSG_MODE_CONTROL = b'\x10'
        MSG_CV_CMD = b'\x20'
        MSG_ACK = b'\x40'

    class eSepChar(Enum):  # start and ending hexes, acknowledgement bit
        CHAR_STX = b'\x02'
        CHAR_ETX = b'\x03'
        ACK_ASCII = b'ACK'
        CHAR_UNUSED = b'\xFF'

    class eRxState(Enum):
        RX_STATE_INIT = 0
        RX_STATE_WAIT_FOR_STX = 1
        RX_STATE_READ_PAYLOAD = 2

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

        self.ser = serial.Serial(port='/dev/ttyTHS2', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
        ## Manual test on Windows
        # self.ser = serial.Serial(port='COM9', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)

        self.txCvCmdMsg = bytearray(self.eSepChar.CHAR_STX.value + self.eMsgType.MSG_CV_CMD.value + self.eSepChar.CHAR_UNUSED.value*12 + self.eSepChar.CHAR_ETX.value)
        # txAckMsg is always the same, so use the immutable bytes object
        self.txAckMsg = b''.join([self.eSepChar.CHAR_STX.value, self.eMsgType.MSG_ACK.value, self.eSepChar.ACK_ASCII.value, self.eSepChar.CHAR_UNUSED.value*9, self.eSepChar.CHAR_ETX.value])
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
        chassis_speed_y = -chassis_speed_x

        # Tx
        if self.AutoAimSwitch or self.AutoMoveSwitch:
            self.txCvCmdMsg[self.DATA_PAYLOAD_INDEX:self.DATA_PAYLOAD_INDEX+12] = b''.join([gimbal_coordinate_x.to_bytes(2, 'little'), gimbal_coordinate_y.to_bytes(2, 'little'), struct.pack('<f', chassis_speed_x), struct.pack('<f', chassis_speed_y)])
            self.ser.write(self.txCvCmdMsg)

        # Rx
        self.CvCmd_RxHeartbeat()
        return (self.AutoAimSwitch, self.AutoMoveSwitch, self.EnemySwitch)

    def CvCmd_RxHeartbeat(self):
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
            self.Rx_State = self.eRxState.RX_STATE_WAIT_FOR_STX

        elif self.Rx_State == self.eRxState.RX_STATE_WAIT_FOR_STX:
            # polling for control msg, if any msg received, ACK back
            if self.ser.in_waiting >= self.DATA_PACKAGE_SIZE:
                # read_until returns b'' or b'\x...\x02' or '\x02'. The point is it contains bytes up to b'\x02'
                bytesUpToStx = self.ser.read_until(self.eSepChar.CHAR_STX.value)
                if bytesUpToStx and (bytesUpToStx[-1] == int.from_bytes(self.eSepChar.CHAR_STX.value, 'little')):
                    self.Rx_State = self.eRxState.RX_STATE_READ_PAYLOAD
                else:
                    self.ser.reset_input_buffer()

        elif self.Rx_State == self.eRxState.RX_STATE_READ_PAYLOAD:
            # CHAR_STX may be the last char in the buffer when switching from RX_STATE_WAIT_FOR_STX, so need to take care of in_waiting size here
            if self.ser.in_waiting >= self.DATA_PACKAGE_SIZE-1:
                byteRead = self.ser.read(1)
                fInvalid = True
                if byteRead == self.eMsgType.MSG_MODE_CONTROL.value:
                    self.rxSwitchBuffer = int.from_bytes(self.ser.read(1), 'little')

                    # check remaining payload
                    bytesUpToEtx = self.ser.read_until(self.eSepChar.CHAR_ETX.value)
                    if bytesUpToEtx and (bytesUpToEtx == self.eSepChar.CHAR_UNUSED.value*11 + self.eSepChar.CHAR_ETX.value):
                        self.AutoAimSwitch = bool(self.rxSwitchBuffer & self.eModeControlBits.MODE_AUTO_AIM_BIT.value)
                        self.AutoMoveSwitch = bool(self.rxSwitchBuffer & self.eModeControlBits.MODE_AUTO_MOVE_BIT.value)
                        self.EnemySwitch = bool(self.rxSwitchBuffer & self.eModeControlBits.MODE_ENEMY_DETECTED_BIT.value)
                        self.ser.write(self.txAckMsg)
                        self.Rx_State = self.eRxState.RX_STATE_WAIT_FOR_STX
                        fInvalid = False

                if fInvalid:
                    # maybe reader cursor derailed; immediately look for STX again to save looping time
                    bytesUpToStx = self.ser.read_until(self.eSepChar.CHAR_STX.value)
                    if bytesUpToStx and (bytesUpToStx[-1] == int.from_bytes(self.eSepChar.CHAR_STX.value, 'little')):
                        pass  # stay in this state; read payload of next msg
                    else:
                        self.Rx_State = self.eRxState.RX_STATE_WAIT_FOR_STX
