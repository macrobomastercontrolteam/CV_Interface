from enum import Enum
import time
import serial
import struct
import re
import math


class CvCmdHandler:
    # misc constants
    DATA_TIMESTAMP_INDEX = 2
    DATA_PACKAGE_SIZE = 21  # 2 bytes header, 1 byte msg type, 16 bytes payload
    DATA_PAYLOAD_INDEX = 5
    MIN_TX_SEPARATION_SEC = 0  # reserved for future, currently control board is fast enough
    MIN_INFO_REQ_SEPARATION_SEC = 1
    SHOOT_TIMEOUT_SEC = 2
    DEBUG_CV = True

    class eMsgType(Enum):
        MSG_MODE_CONTROL = b'\x10'
        MSG_CV_CMD = b'\x20'
        MSG_ACK = b'\x40'
        MSG_INFO_REQUEST = b'\x50'
        MSG_INFO_FEEDBACK = b'\x51'

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
        MODE_SHOOT_BIT = 0b00001000

    class eInfoBits(Enum):
        MODE_TRAN_DELTA_BIT = 0b00000001
        MODE_CV_SYNC_TIME_BIT = 0b00000010

    # @param[in] max_camera_coordinate_x, max_camera_coordinate_y: type is int; unit is pixel; starts from 1
    # @param[in] max_camera_angle_yaw: field of view angle; type is float; unit is radian
    def __init__(self, serial_port, max_camera_coordinate_x, max_camera_coordinate_y, max_camera_angle_yaw, focal_length_in_mm=None, optical_format=None, camera_to_axis_distance=None):
        # Hardware parameters
        self.camera_to_axis_distance = camera_to_axis_distance
        self.half_camera_coordinate_x = max_camera_coordinate_x/2
        self.half_camera_coordinate_y = max_camera_coordinate_y/2
        self.focal_length_in_pixel = self.half_camera_coordinate_x / math.tan(max_camera_angle_yaw*math.pi/180/2)
        # Because the manufacturer may truncate usable pixels, the following calculation may not be accurate
        # if focal_length_in_mm != None:
        #     # TODO: extend support to different optical_format
        #     # optical_format = 1 for 1/4''; 2 for 1/2''; 4 for 1''; 8 for 2''
        #     if optical_format == 1:
        #         assert (abs(focal_length_in_mm - self.focal_length_in_pixel * (3.6/max_camera_coordinate_x)) <= 0.1)
        #     else:
        #         raise NotImplementedError

        self.ackMsgInfo = {"reqCtrlTimestamp": -1, "reqRxTimestamp": -1}
        self.CvSyncTime = 0
        self.CvCmd_Reset()

        # self.ser = serial.Serial(port='/dev/ttyTHS2', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
        # Manual test on Windows
        self.ser = serial.Serial(port=serial_port, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)

        self.txCvCmdMsg = self.CvCmd_InitTxMsg(self.eMsgType.MSG_CV_CMD.value)
        self.txInfoRequestMsg = self.CvCmd_InitTxMsg(self.eMsgType.MSG_INFO_REQUEST.value)
        self.txSetModeMsg = self.CvCmd_InitTxMsg(self.eMsgType.MSG_MODE_CONTROL.value)
        self.txAckMsg = self.CvCmd_InitTxMsg(self.eMsgType.MSG_ACK.value + self.eSepChar.ACK_ASCII.value)

        # @TODO: reserve data placeholders within msg frames for each type of msg beforehand
        self.txAckMsgPayloadIndex = self.DATA_PAYLOAD_INDEX + len(self.eSepChar.ACK_ASCII.value)

    def CvCmd_InitTxMsg(self, payload):
        txMsg = bytearray(self.eSepChar.CHAR_HEADER.value)
        txMsg += b'\x00\x00'  # placeholder for timestamp
        txMsg += payload
        txMsg += self.eSepChar.CHAR_UNUSED.value*(self.DATA_PACKAGE_SIZE-len(txMsg))
        return txMsg

    def CvCmd_GetUint16Time(self):
        return int(time.time()*1000) & ((1 << 16)-1)

    def CvCmd_GetUint16Delta(self, minuendTime, subtrahendTime):
        signedDelta = minuendTime - subtrahendTime
        unsignedDelta = signedDelta if (signedDelta >= 0) else (signedDelta + (1 << 16))
        return unsignedDelta

    def CvCmd_BuildSendTxMsg(self, txMsg):
        txMsg[self.DATA_TIMESTAMP_INDEX:self.DATA_TIMESTAMP_INDEX+2] = struct.pack('<H', self.CvCmd_GetUint16Delta(self.CvCmd_GetUint16Time(), self.CvSyncTime))
        self.ser.write(txMsg)
        self.prevTxTime = time.time()
        if self.DEBUG_CV:
            assert (len(txMsg) == self.DATA_PACKAGE_SIZE)
            print("Sent: ", txMsg)
            if txMsg is self.txAckMsg:
                print("ACK sent")
            elif txMsg is self.txInfoRequestMsg:
                print("Info request sent")
            elif txMsg is self.txCvCmdMsg:
                print("Gimbal cmd sent")
            elif txMsg is self.txSetModeMsg:
                print("Shoot req sent")

    def CvCmd_Reset(self):
        # reqCtrlTimestamp: control native timestamp contained within the request sent by control board
        # reqRxTimestamp: receive timestamp of the request sent by control board
        self.ackMsgInfo["reqCtrlTimestamp"] = -1
        self.ackMsgInfo["reqRxTimestamp"] = -1
        self.infoRequestPending = 0
        self.prevTxTime = 0
        self.prevInfoReqTime = 0
        self.cvCmdCount = 0
        self.tranDelta = None  # Transmission delay time in ms
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
        self.prev_Rx_State = self.Rx_State
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except:
            pass

    # @brief main API function
    # @param[in] gimbal_coordinate_x and gimbal_coordinate_y: type is int; will be converted to int16_t
    # @param[in] chassis_speed_x and chassis_speed_y: type is float; can be positive/negative; will be converted to float (32 bits)
    def CvCmd_Heartbeat(self, gimbal_coordinate_x, gimbal_coordinate_y, chassis_speed_x, chassis_speed_y, target_depth=None):
        self.gimbal_coordinate_x = int(gimbal_coordinate_x)
        self.gimbal_coordinate_y = int(gimbal_coordinate_y)
        self.chassis_speed_x = float(chassis_speed_x)
        self.chassis_speed_y = float(chassis_speed_y)
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
        camera_angle_x = math.atan((self.gimbal_coordinate_x - self.half_camera_coordinate_x)/self.focal_length_in_pixel)
        camera_angle_y = math.atan((self.half_camera_coordinate_y - self.gimbal_coordinate_y)/self.focal_length_in_pixel)
        if (self.target_depth == None) or (self.camera_to_axis_distance == None):
            # Approximation is valid if self.target_depth >> self.camera_to_axis_distance
            self.gimbal_cmd_delta_yaw = camera_angle_x
            self.gimbal_cmd_delta_pitch = camera_angle_y
        else:
            self.gimbal_cmd_delta_yaw = math.atan(self.target_depth*math.sin(camera_angle_x)/(self.camera_to_axis_distance+math.sin(camera_angle_x)))
            self.gimbal_cmd_delta_pitch = math.atan(self.target_depth*math.sin(camera_angle_y)/(self.camera_to_axis_distance+math.sin(camera_angle_y)))
        if self.DEBUG_CV:
            print("gimbal_cmd_delta_yaw: ", self.gimbal_cmd_delta_yaw, "gimbal_cmd_delta_pitch: ", self.gimbal_cmd_delta_pitch)

        # Chassis: speed to speed conversion
        # CV positive directions: +x is to the right, +y is upwards
        # Remote controller positive directions: +x is upwards, +y is to the left
        self.chassis_cmd_speed_x = self.chassis_speed_y
        self.chassis_cmd_speed_y = -self.chassis_speed_x

    def CvCmd_RxHeartbeat(self):
        fHeartbeatFinished = True

        if self.DEBUG_CV:
            if self.ser.is_open and self.ser.in_waiting > 0:
                print("Input buffer size: ", self.ser.in_waiting)
            if self.prev_Rx_State != self.Rx_State:
                print("Rx_State: ", self.Rx_State)
                self.prev_Rx_State = self.Rx_State

        if self.Rx_State == self.eRxState.RX_STATE_INIT:
            if not self.ser.is_open:
                self.ser.open()
            self.CvCmd_Reset()
            self.Rx_State = self.eRxState.RX_STATE_WAIT_FOR_PKG
            fHeartbeatFinished = True

        elif self.Rx_State == self.eRxState.RX_STATE_WAIT_FOR_PKG:
            if self.ser.in_waiting >= self.DATA_PACKAGE_SIZE:
                bytesRead = self.ser.read(self.ser.in_waiting)
                # @TODO: use regex to search msg by msg instead of processing only the last msg. For now, control board don't have much to send, so it's fine.
                setModeRequestPackets = re.findall(self.eSepChar.CHAR_HEADER.value + b".." + self.eMsgType.MSG_MODE_CONTROL.value + b"." + self.eSepChar.CHAR_UNUSED.value + b"{15}", bytesRead)
                infoFeedbackPackets = re.findall(self.eSepChar.CHAR_HEADER.value + b".." + self.eMsgType.MSG_INFO_FEEDBACK.value + b"." + b".." + self.eSepChar.CHAR_UNUSED.value + b"{13}", bytesRead)
                if self.DEBUG_CV:
                    print("bytesRead: ", bytesRead)
                    print("setModeRequestPackets: ", setModeRequestPackets)
                    print("infoFeedbackPackets: ", infoFeedbackPackets)

                if setModeRequestPackets:
                    # read the mode of the last packet, because it's the latest
                    rxSwitchBuffer = setModeRequestPackets[-1][self.DATA_PAYLOAD_INDEX]
                    self.AutoAimSwitch = bool(rxSwitchBuffer & self.eModeControlBits.MODE_AUTO_AIM_BIT.value)
                    self.AutoMoveSwitch = bool(rxSwitchBuffer & self.eModeControlBits.MODE_AUTO_MOVE_BIT.value)
                    self.EnemySwitch = bool(rxSwitchBuffer & self.eModeControlBits.MODE_ENEMY_DETECTED_BIT.value)
                    # Shoot switch is controlled by CV
                    # self.ShootSwitch = bool(rxSwitchBuffer & self.eModeControlBits.MODE_SHOOT_BIT.value)
                    self.ackMsgInfo["reqCtrlTimestamp"] = struct.unpack('<H', setModeRequestPackets[-1][self.DATA_TIMESTAMP_INDEX:self.DATA_TIMESTAMP_INDEX+2])[0]
                    self.ackMsgInfo["reqRxTimestamp"] = self.CvCmd_GetUint16Time()
                    self.cvCmdCount = 0
                    self.Rx_State = self.eRxState.RX_STATE_SEND_ACK
                    fHeartbeatFinished = False

                if infoFeedbackPackets:
                    for packet in infoFeedbackPackets:
                        rxInfoType = packet[self.DATA_PAYLOAD_INDEX]
                        rxInfoData = packet[self.DATA_PAYLOAD_INDEX+1:self.DATA_PAYLOAD_INDEX+3]
                        if rxInfoType == self.eInfoBits.MODE_TRAN_DELTA_BIT.value:
                            # Warning: tranDelta is signed
                            self.tranDelta = struct.unpack('<h', rxInfoData)[0]
                            self.infoRequestPending &= ~rxInfoType
                            if self.DEBUG_CV:
                                print("tranDelta: ", self.tranDelta)
                        elif rxInfoType == self.eInfoBits.MODE_CV_SYNC_TIME_BIT.value:
                            self.CvSyncTime = struct.unpack('<H', rxInfoData)[0]
                            self.infoRequestPending &= ~rxInfoType
                            if self.DEBUG_CV:
                                print("CvSyncTime: ", self.CvSyncTime)
                    # Do not change Rx_State or fHeartbeatFinished

                # No valid msg received, retry connection
                if not (setModeRequestPackets or infoFeedbackPackets):
                    self.ser.reset_input_buffer()
                    fHeartbeatFinished = True

        elif self.Rx_State == self.eRxState.RX_STATE_SEND_ACK:
            if time.time() - self.prevTxTime > self.MIN_TX_SEPARATION_SEC:
                self.CvSyncTime = self.CvCmd_GetUint16Time()
                uiExecDelta = self.CvCmd_GetUint16Delta(self.CvSyncTime, self.ackMsgInfo["reqRxTimestamp"])
                self.txAckMsg[self.txAckMsgPayloadIndex:self.txAckMsgPayloadIndex+6] = struct.pack('<HHH', self.ackMsgInfo["reqCtrlTimestamp"], uiExecDelta, self.CvSyncTime)
                self.CvCmd_BuildSendTxMsg(self.txAckMsg)
                # Trigger info request in TxHeartbeat
                self.infoRequestPending |= self.eInfoBits.MODE_TRAN_DELTA_BIT.value | self.eInfoBits.MODE_CV_SYNC_TIME_BIT.value
                self.Rx_State = self.eRxState.RX_STATE_WAIT_FOR_PKG
            fHeartbeatFinished = True

        return fHeartbeatFinished

    def CvCmd_TxHeartbeat(self):
        # Tx: keeping sending cmd to keep control board alive (watchdog timer logic)
        if (time.time() - self.prevTxTime > self.MIN_TX_SEPARATION_SEC):
            if self.infoRequestPending and (time.time() - self.prevInfoReqTime > self.MIN_INFO_REQ_SEPARATION_SEC):
                self.txInfoRequestMsg[self.DATA_PAYLOAD_INDEX] = self.infoRequestPending
                self.CvCmd_BuildSendTxMsg(self.txInfoRequestMsg)
                self.prevInfoReqTime = time.time()
            elif ((self.AutoAimSwitch or self.AutoMoveSwitch) and (self.tranDelta != None)):
                self.txCvCmdMsg[self.DATA_PAYLOAD_INDEX:self.DATA_PAYLOAD_INDEX+16] = struct.pack('<ffff', self.gimbal_cmd_delta_yaw, self.gimbal_cmd_delta_pitch, self.chassis_cmd_speed_x, self.chassis_cmd_speed_y)
                self.CvCmd_BuildSendTxMsg(self.txCvCmdMsg)
                self.cvCmdCount += 1
                if (self.cvCmdCount % 10 == 0):
                    self.infoRequestPending |= self.eInfoBits.MODE_TRAN_DELTA_BIT.value

        # Latching shoot switch logic
        if self.ShootSwitch:
            if self.PrevShootSwitch == False:
                if time.time() - self.prevTxTime > self.MIN_TX_SEPARATION_SEC:
                    self.txSetModeMsg[self.DATA_PAYLOAD_INDEX] = self.eModeControlBits.MODE_SHOOT_BIT.value
                    self.CvCmd_BuildSendTxMsg(self.txSetModeMsg)
                    self.shootStartTime = time.time()
                    self.PrevShootSwitch = True
            elif time.time() - self.shootStartTime > self.SHOOT_TIMEOUT_SEC:
                # control should automatically disable itself as well
                self.shootStartTime = time.time()
                self.ShootSwitch = False
                self.PrevShootSwitch = False

    def CvCmd_Shoot(self):
        # ShootSwitch will be automatically disabled after SHOOT_TIMEOUT_SEC
        self.ShootSwitch = True
