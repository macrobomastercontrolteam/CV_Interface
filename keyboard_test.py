import CvCmdApi

# Example usage
CvCmder = CvCmdApi.CvCmdHandler('COM27')
# oldflags = (False, False, False)
# while True:
#     flags = CvCmder.CvCmd_Heartbeat(0, 0, 0, 0)  # gimbal_coordinate_x, gimbal_coordinate_y, chassis_speed_x, chassis_speed_y
#     if flags != oldflags:
#         oldflags = flags
#         print(flags)

import keyboard
import time
oldflags = (False, False, False)
counter = 0
loop_delay = 0.05
while True:
    chassis_vy = 0
    chassis_vx = 0
    gimbal_pitch = 0
    gimbal_yaw = 0

    if keyboard.is_pressed('s'):
        chassis_vy = -0.3
    elif keyboard.is_pressed('w'):
        chassis_vy = 0.3
    elif keyboard.is_pressed('a'):
        chassis_vx = -0.3
    elif keyboard.is_pressed('d'):
        chassis_vx = 0.3

    # percentage
    if keyboard.is_pressed('k'):
        gimbal_pitch = -0.2
    elif keyboard.is_pressed('i'):
        gimbal_pitch = 0.2
    elif keyboard.is_pressed('j'):
        gimbal_yaw = -0.2
    elif keyboard.is_pressed('l'):
        gimbal_yaw = 0.2

    if keyboard.is_pressed('q'):
        break
    
    # if counter == int(5/loop_delay):
    #     CvCmder.CvCmd_Shoot()

    flags = CvCmder.CvCmd_Heartbeat(gimbal_pitch_target=gimbal_pitch, gimbal_yaw_target=gimbal_yaw, chassis_speed_x=chassis_vx, chassis_speed_y=chassis_vy)
    # flags = CvCmder.CvCmd_Heartbeat(gimbal_coordinate_x=1280/2*(1+gimbal_yaw), gimbal_coordinate_y=720/2*(1+gimbal_pitch),chassis_speed_x=chassis_vx,chassis_speed_y=chassis_vy)
    if flags != oldflags:
        oldflags = flags
        print(flags)
    # print(chassis_vx, chassis_vy)
    time.sleep(loop_delay)
    counter += 1