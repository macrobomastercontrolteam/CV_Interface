# CV_Interface
Communication layer API for computer vision (CV) to connect with control board

CV requirement for python version: 3.6.9

## Signal Conditioners
### Camera pixel to angle conversion
Reference: https://www.cs.cmu.edu/~16385/s17/Slides/11.1_Camera_matrix.pdf
### Camera frame to Control board IMU frame conversion
Alternative IMU solutions:
- **(Current setup)** Camera must have its own IMU to measure its orientation. The orientation of camera frame is different from the orientation of control board IMU frame. Therefore, the target 3D point in camera frame must be converted into control board IMU frame. We are free to do it on CV side before sending or control side after sending. For now, it's easier to do it on control side, because it's easier for Ctrl to find the transformation matrix during calibration process.
- CV and Ctrl can share the same IMU. In this case, the orientation relationship with Camera and IMU should be hardcoded into CV, and target position should be converted into IMU frame before sending to Ctrl. Finally, Ctrl would not need to do any conversion.

Control board should use its current IMU location, target location sent by CV and initial projectile speed setting to calculate the projectile direction as the setpoint for gimbal motors.

Typically, we fix control board to the same orientation as the cannon, so omit the difference in cannon frame and control board IMU frame. 

@TODO: To have good precision, control would better have a cascaded feed-forward & feedback control strategy to ensure the initial projectile speed. One of the feedbacks should be Referee system's projectile speed measurement. Its actuator would better be CAN-bus controlled M3508 without reduction gears. We must wait until friction wheels to reach certain speed before loading projectiles to shoot every time. And relation between this wheel speed and the actually measured initial speed should be estimated and feed-forwarded for future shots.

(WIP)

## Data Packages
### Control to CV
(WIP)
### CV to Control
(WIP)

## Sequence Diagrams
### Normal CV command process
``` mermaid
sequenceDiagram
    autonumber
    loop while True
        Note left of Control: Communication establishment: time synchronization
        loop every 1.5 seconds until ACK
            Control-->Control:Time1 = Timestamp(Tx)
            Control->>CV :Set-mode request: Auto-aim or Auto-move on
        end
        activate CV
        CV-->CV :SyncTime = Timestamp(Tx)<br/>ExecDelta = Timestamp(Tx) - Timestamp(Rx)
        deactivate CV
        CV->>Control :ACK (with ExecDelta)
        Control-->Control: TranDelta = (Timestamp(Rx) - Time1 - ExecDelta)/2<br/>SyncTime = Timstamp(Rx) - TranDelta
        CV->>Control :Request Info:TranDelta
        Control->>CV :Info:TranDelta
        Note left of Control: Cv Command State
        loop every MIN_TX_SEPARATION_SEC<br/>until CV msg interval > 1 sec,<br/>for which Ctrl would deem CV being offline
            CV->>Control :CvCmdMsg<br/>(Enemy 3D location in Camera frame)<br/>(chassis 2D target location in Camera frame)
            opt Every 5 seconds
                CV->>Control :Request info:TranDelta
                Control->>CV :Info:TranDelta
                CV-->CV :Update TranDelta in Kalman filter
            end
        end
    end
```
### Gimbal frame calibration process
``` mermaid
sequenceDiagram
    Control-->Control:Calibration process started by remote controller
    Control->>CV :Set-mode request (Auto-aim & Auto-move off)
    CV->>Control :ACK
    loop At every step of gimbal calibration,<br/>i.e. when gimbal is aiming at <br/>bottom, top, left, right directions
        Control-->Control:After Ctrl board IMU signal is stable for 0.5 seconds
        Control->>CV :Request Info: Camera IMU data
        CV->>Control :Info: Camera IMU data
    end
    Control-->Control: Solve for and save transformation matrix
```

## Circuit Diagram
``` mermaid
flowchart LR
    subgraph CtrlBoardC["Control Board"]
        subgraph CtrltoCv-UART["UART for CV"]
            Ctrl-CtrltoCv-VCC["5V"]
            Ctrl-CtrltoCv-Rx["Rx"]
            Ctrl-CtrltoCv-Tx["Tx"]
            Ctrl-CtrltoCv-Gnd["Gnd"]
        end
    end

    subgraph CV["Computer Vision Board"]
        subgraph CV-CtrltoCv-USBtoTTL["USB-to-TTL (5V level)"]
            CV-CtrltoCv-VCC["5V"]
            CV-CtrltoCv-Rx["Rx"]
            CV-CtrltoCv-Tx["Tx"]
            CV-CtrltoCv-Gnd["Gnd"]
        end
    end

    Ctrl-CtrltoCv-VCC~~~|"#quot;5V#quot; may be different,\nso do not connect"|CV-CtrltoCv-VCC
    Ctrl-CtrltoCv-Rx---CV-CtrltoCv-Tx
    Ctrl-CtrltoCv-Tx---CV-CtrltoCv-Rx
    Ctrl-CtrltoCv-Gnd---CV-CtrltoCv-Gnd
```

## Jetson TX2 Special Circuit Connection (Obsolete)
This picture is obsolte, please use USB-to-TTL dongle instead. Because the UART Tx port on Jetson TX2 is not working properly. Its current output capacity is too low that it must use a MOSFET circuit to drive the control board's Rx pin.
- Serial Port : /dev/ttyTHS2
- Resistors below converts control board's 5V level Tx signal down to 3.3V via voltage divider.
![6634a59717a0defd36d039292c8d643](https://user-images.githubusercontent.com/56321690/236954118-339b6e05-28cb-4140-a3d6-6123518752b6.jpg)
