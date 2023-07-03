import CvCmdApi

# Example usage
CvCmder = CvCmdApi.CvCmdHandler('COM19', 1280, 720, 100)
# oldflags = (False, False, False)
# while True:
#     flags = CvCmder.CvCmd_Heartbeat(0, 0, 0, 0)  # gimbal_coordinate_x, gimbal_coordinate_y, chassis_speed_x, chassis_speed_y
#     if flags != oldflags:
#         oldflags = flags
#         print(flags)

import keyboard
import time
oldflags = (False, False, False)
chassis_speed_y_Slider = 0
chassis_speed_x_Slider = 0
# gimbal_coordinate_y_Slider_arr = [100, 0]
gimbal_coordinate_y_Slider = 0
gimbal_coordinate_y_Slider_index = 0
counter = 0
loop_delay = 0.05
while True:
    if keyboard.is_pressed('s'):
        chassis_speed_y_Slider += -0.02
    elif keyboard.is_pressed('w'):
        chassis_speed_y_Slider += 0.02
    elif keyboard.is_pressed('a'):
        chassis_speed_x_Slider += -0.02
    elif keyboard.is_pressed('d'):
        chassis_speed_x_Slider += 0.02
    if keyboard.is_pressed('q'):
        break
    
    if counter == int(5/loop_delay):
        CvCmder.CvCmd_Shoot()

    flags = CvCmder.CvCmd_Heartbeat(gimbal_coordinate_x=0,gimbal_coordinate_y=gimbal_coordinate_y_Slider,chassis_speed_x=chassis_speed_x_Slider,chassis_speed_y=chassis_speed_y_Slider)
    if flags != oldflags:
        oldflags = flags
        print(flags)
    print(chassis_speed_x_Slider, chassis_speed_y_Slider)
    time.sleep(loop_delay)
    counter += 1