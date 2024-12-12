package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmControl {
    private final MotorEx arm;
    private final PIDFController armPID;
    private double armTargetPosition = 0.0;

    private static final double TICKS_PER_REVOLUTION = 28;

    private static final double kP = 0.008;
    private static final double kI = 0;
    private static final double kD = 0.001;
    private static final double kF = 0;
    private static final double kG = 0;

    private double previousPosition = 0.0;
    private double previousTime = 0.0;


    public ArmControl(HardwareMap hardwareMap) {
        arm = new MotorEx(hardwareMap, "arm");
        armPID = new PIDFController(kP, kI, kD, kF);
    }

    public void setJoystickInput(double joystickInput) {
        armTargetPosition += joystickInput;
    }

    public void update() {
        double currentPosition = arm.getCurrentPosition();

        double gravityCompensation = kG * Math.cos(Math.toRadians(currentPosition));

        double armPower = armPID.calculate(currentPosition, armTargetPosition) + gravityCompensation;

        arm.set(armPower);
    }

    public void setPosition(double position) {
        armTargetPosition = position;
    }

    public double getArmTargetPosition() {
        return armTargetPosition;
    }

    public double getArmPower() {
        return armPID.calculate(arm.getCurrentPosition(), armTargetPosition);
    }

    public void resetZero() {
        arm.stopAndResetEncoder();
    }

    public double getArmPosition() {
        return arm.getCurrentPosition();
    }

    public void setArmPower(double power) {
        arm.set(power);
    }
}
