package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoSubsystemForAuto {
    private final Servo servo;

    public ServoSubsystemForAuto(Servo servo) {
        this.servo = servo;
    }


    public void setServoPosition(double position) {
        position = Math.max(-90, Math.min(90, position));

        double mappedPosition = (position + 90) / 180.0;

        servo.setPosition(mappedPosition);
    }

}

