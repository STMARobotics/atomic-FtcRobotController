package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmControl {
    private final MotorEx arm;
    private final PIDFController armPID;
    private double armTargetPosition = 0.0;

    // PID constants

    public ArmControl(HardwareMap hardwareMap) {
        arm = new MotorEx(hardwareMap, "arm");
        armPID = new PIDFController(kP, kI, kD, kF);
    }

        double armPower = armPID.calculate(arm.getCurrentPosition(), armTargetPosition);
        arm.set(armPower);
    }

    public double getArmTargetPosition() {
        return armTargetPosition;
    }

    public double getArmPower() {
        return armPID.calculate(arm.getCurrentPosition(), armTargetPosition);
    }
}