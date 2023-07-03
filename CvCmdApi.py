from enum import Enum
import time
import serial
import struct
import re
import math

class CvCmdHandler:
    # misc constants
    DATA_PACKAGE_SIZE = 19  # 2 bytes header, 1 byte msg type, 16 bytes payload
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
        RX_STATE_WAIT_FOR_PKG = 1
        RX_STATE_SEND_ACK = 2

    class eModeControlBits(Enum):
        MODE_AUTO_AIM_BIT = 0b00000001
        MODE_AUTO_MOVE_BIT = 0b00000010
        MODE_ENEMY_DETECTED_BIT = 0b00000100

    # @param[in] max_gimbal_coordinate_x, max_gimbal_coordinate_y: type is int; unit is pixel; starts from 1
    # @param[in] max_gimbal_angle_yaw: field of view angle; type is float; unit is radian
    def __init__(self, serial_port, max_gimbal_coordinate_x, max_gimbal_coordinate_y, max_gimbal_angle_yaw, focal_length_in_mm=None, optical_format=None, camera_to_axis_distance=None):
        # Hardware parameters
        self.camera_to_axis_distance = camera_to_axis_distance
        self.half_gimbal_coordinate_x = max_gimbal_coordinate_x/2
        self.half_gimbal_coordinate_y = max_gimbal_coordinate_y/2
        self.focal_length_in_pixel = self.half_gimbal_coordinate_x / math.tan(max_gimbal_angle_yaw*math.pi/180/2)
        # Because the manufacturer may truncate usable pixels, the following calculation may not be accurate
        # if focal_length_in_mm != None:
        #     # TODO: extend support to different optical_format
        #     # optical_format = 1 for 1/4''; 2 for 1/2''; 4 for 1''; 8 for 2''
        #     if optical_format == 1:
        #         assert (abs(focal_length_in_mm - self.focal_length_in_pixel * (3.6/max_gimbal_coordinate_x)) <= 0.1)
        #     else:
        #         raise NotImplementedError

        self.CvCmd_Reset()

        # self.ser = serial.Serial(port='/dev/ttyTHS2', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
        # Manual test on Windows
        self.ser = serial.Serial(port=serial_port, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)

        self.txCvCmdMsg = bytearray(self.eSepChar.CHAR_HEADER.value + self.eMsgType.MSG_CV_CMD.value + self.eSepChar.CHAR_UNUSED.value*16)
        # txAckMsg is always the same, so use the immutable bytes object
        self.txAckMsg = b''.join([self.eSepChar.CHAR_HEADER.value, self.eMsgType.MSG_ACK.value, self.eSepChar.ACK_ASCII.value, self.eSepChar.CHAR_UNUSED.value*13])

        assert (len(self.txCvCmdMsg) == self.DATA_PACKAGE_SIZE)
        assert (len(self.txAckMsg) == self.DATA_PACKAGE_SIZE)
        assert (len(self.txSetModeMsg) == self.DATA_PACKAGE_SIZE)

    def CvCmd_Reset(self):
        self.prevTxTime = 0
        self.AutoAimSwitch = False
        self.AutoMoveSwitch = False
        self.EnemySwitch = False
        self.PrevShootSwitch = False
        self.ShootSwitch = False
        self.gimbal_cmd_delta_yaw = 0
        self.gimbal_cmd_delta_pitch = 0
        self.chassis_cmd_speed_x = 0
        self.chassis_cmd_speed_y = 0
        self.gimbal_coordinate_x = 0
        self.gimbal_coordinate_y = 0
        self.chassis_speed_x = 0
        self.chassis_speed_y = 0
        self.target_depth = None
        self.Rx_State = self.eRxState.RX_STATE_INIT
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except:
            pass

    # @brief main API function
    # @param[in] gimbal_coordinate_x and gimbal_coordinate_y: type is int; will be converted to int16_t
    # @param[in] chassis_speed_x and chassis_speed_y: type is float; can be positive/negative; will be converted to float (32 bits)
    def CvCmd_Heartbeat(self, gimbal_coordinate_x, gimbal_coordinate_y, chassis_speed_x, chassis_speed_y, target_depth=None):
        self.gimbal_coordinate_x = gimbal_coordinate_x
        self.gimbal_coordinate_y = gimbal_coordinate_y
        self.chassis_speed_x = chassis_speed_x
        self.chassis_speed_y = chassis_speed_y
        self.target_depth = target_depth

        # Condition signals
        self.CvCmd_ConditionSignals()
        # Tx
        self.CvCmd_TxHeartbeat()
        # Rx
        fHeartbeatFinished = False
        while fHeartbeatFinished == False:
            fHeartbeatFinished = self.CvCmd_RxHeartbeat()

        return (self.AutoAimSwitch, self.AutoMoveSwitch, self.EnemySwitch)

    def CvCmd_ConditionSignals(self):
        # Gimbal: pixel to angle conversion
        # TODO: Use parabolic instead of linear trajectory
        # CV positive directions: +x is to the right, +y is downwards
        # angle unit is radian
        camera_angle_x = math.atan((self.gimbal_coordinate_x - self.half_gimbal_coordinate_x)/self.focal_length_in_pixel)
        camera_angle_y = math.atan((self.half_gimbal_coordinate_y - self.gimbal_coordinate_y)/self.focal_length_in_pixel)
        if (self.target_depth == None) or (self.camera_to_axis_distance == None):
            # Approximation is valid if self.target_depth >> self.camera_to_axis_distance
            self.gimbal_cmd_delta_yaw = camera_angle_x
            self.gimbal_cmd_delta_pitch = camera_angle_y
        else:
            self.gimbal_cmd_delta_yaw = math.atan(self.target_depth*math.sin(camera_angle_x)/(self.camera_to_axis_distance+math.sin(camera_angle_x)))
            self.gimbal_cmd_delta_pitch = math.atan(self.target_depth*math.sin(camera_angle_y)/(self.camera_to_axis_distance+math.sin(camera_angle_y)))

        # Chassis: speed to speed conversion
        # CV positive directions: +x is to the right, +y is upwards
        # Remote controller positive directions: +x is upwards, +y is to the left
        self.chassis_cmd_speed_x = self.chassis_speed_y
        self.chassis_cmd_speed_y = -self.chassis_speed_x

    def CvCmd_RxHeartbeat(self):
        fHeartbeatFinished = True

        if self.Rx_State == self.eRxState.RX_STATE_INIT:
            if not self.ser.is_open:
                self.ser.open()
            self.CvCmd_Reset()
            self.Rx_State = self.eRxState.RX_STATE_WAIT_FOR_PKG
            fHeartbeatFinished = True

        elif self.Rx_State == self.eRxState.RX_STATE_WAIT_FOR_PKG:
            # polling for control msg, if any msg received, ACK back
            if self.ser.in_waiting >= self.DATA_PACKAGE_SIZE:
                bytesRead = self.ser.read(self.ser.in_waiting)
                dataPackets = re.findall(self.eSepChar.CHAR_HEADER.value + self.eMsgType.MSG_MODE_CONTROL.value + b"." + self.eSepChar.CHAR_UNUSED.value + b"{15}", bytesRead)
                if dataPackets:
                    # read the mode of the last packet, because it's the latest
                    rxSwitchBuffer = dataPackets[-1][self.DATA_PAYLOAD_INDEX]
                    self.AutoAimSwitch = bool(rxSwitchBuffer & self.eModeControlBits.MODE_AUTO_AIM_BIT.value)
                    self.AutoMoveSwitch = bool(rxSwitchBuffer & self.eModeControlBits.MODE_AUTO_MOVE_BIT.value)
                    self.EnemySwitch = bool(rxSwitchBuffer & self.eModeControlBits.MODE_ENEMY_DETECTED_BIT.value)
                    self.Rx_State = self.eRxState.RX_STATE_SEND_ACK
                    fHeartbeatFinished = False
                else:
                    self.ser.reset_input_buffer()
                    fHeartbeatFinished = True

        elif self.Rx_State == self.eRxState.RX_STATE_SEND_ACK:
            if time.time() - self.prevTxTime > self.MIN_TX_SEPARATION_SEC:
                self.ser.write(self.txAckMsg)
                self.prevTxTime = time.time()
                self.Rx_State = self.eRxState.RX_STATE_WAIT_FOR_PKG
            fHeartbeatFinished = True

        return fHeartbeatFinished

    def CvCmd_TxHeartbeat(self):
        # Tx: keeping sending cmd to keep control board alive (watchdog timer logic)
        if (self.AutoAimSwitch or self.AutoMoveSwitch) and (time.time() - self.prevTxTime > self.MIN_TX_SEPARATION_SEC):
            self.txCvCmdMsg[self.DATA_PAYLOAD_INDEX:self.DATA_PAYLOAD_INDEX+16] = struct.pack('<ffff', self.gimbal_cmd_delta_yaw, self.gimbal_cmd_delta_pitch, self.chassis_cmd_speed_x, self.chassis_cmd_speed_y)
            # print("Delta yaw: ", self.gimbal_cmd_delta_yaw)
            # print("Delta pitch: ", self.gimbal_cmd_delta_pitch)
            self.ser.write(self.txCvCmdMsg)
            self.prevTxTime = time.time()
