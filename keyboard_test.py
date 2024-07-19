import CvCmdApi
CvCmder = CvCmdApi.CvCmdHandler('COM16')

import keyboard
import time
loop_delay = 0
gimbal_pitch = 0
gimbal_yaw = 0
while True:
    chassis_vy = 0
    chassis_vx = 0

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
        gimbal_pitch = 0.5
    elif keyboard.is_pressed('i'):
        gimbal_pitch = -0.5
    elif keyboard.is_pressed('j'):
        gimbal_yaw = -0.3
    elif keyboard.is_pressed('l'):
        gimbal_yaw = 0.3

    if keyboard.is_pressed('q'):
        break
    
    if keyboard.is_pressed('g'):
        CvCmder.CvCmd_Shoot()
        
    if keyboard.is_pressed('c'):
        CvCmder.CvCmd_Chassis_Spinning(True)
    
    if keyboard.is_pressed('v'):
        CvCmder.CvCmd_Chassis_Spinning(False)

    CvCmder.CvCmd_Heartbeat(gimbal_pitch_target=gimbal_pitch, gimbal_yaw_target=gimbal_yaw, chassis_speed_x=chassis_vx, chassis_speed_y=chassis_vy)
    # Warning: for python version less than 3.11 running on Windows, the min achieveable sleep time is around 15ms. Test yourself before using time.sleep()
    time.sleep(loop_delay)
