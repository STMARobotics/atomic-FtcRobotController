package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoSubsystemForAuto {
    private Servo servo;

    public ServoSubsystemForAuto(DcMotorEx slide, Servo servo) {
        this.servo = servo;
    }

    public ServoSubsystemForAuto(HardwareMap hardwareMap) {
    }


    public void setServoPosition(double position) {
        position = Math.max(-90, Math.min(90, position));

        double mappedPosition = (position + 90) / 180.0;

        servo.setPosition(mappedPosition);
    }

}

