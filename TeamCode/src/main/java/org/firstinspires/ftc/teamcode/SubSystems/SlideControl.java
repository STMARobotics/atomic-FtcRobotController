package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SlideControl {

    private DcMotorEx slide;
    private Servo servo;
    private double targetPosition;
    private double currentPosition;
    private ElapsedTime runtime = new ElapsedTime();

    private static final double TICKS_PER_REVOLUTION = 28;

    private double kP = 0.0075, kI = 0, kD = 0.0;
    private double previousError = 0, integral = 0;

    public SlideControl(DcMotorEx slide, Servo servo) {
        this.slide = slide;
        this.servo = servo;
        this.targetPosition = 0;
        this.currentPosition = 0;
    }

    public SlideControl(HardwareMap hardwareMap) {
    }

    public void resetEncoder() {
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        currentPosition = slide.getCurrentPosition();
        targetPosition = currentPosition;
    }

    public void updatePIDControl() {
        double error = targetPosition - currentPosition;
        integral += error * runtime.seconds();
        double derivative = (error - previousError) / runtime.seconds();

        double power = (kP * error) + (kI * integral) + (kD * derivative);

        slide.setPower(power);
        previousError = error;
    }

    public void setTargetPosition(double target) {
        targetPosition = target;
    }

    public double getCurrentPosition() {
        return slide.getCurrentPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setSlidePower(double power) {
        slide.setPower(power);
    }

    public void setServoPosition(double position) {
        position = Math.max(-90, Math.min(90, position));

        double mappedPosition = (position + 90) / 180.0;

        servo.setPosition(mappedPosition);
    }

    public void update() {
        currentPosition = slide.getCurrentPosition();
        updatePIDControl();
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Current Motor Position", currentPosition);
        telemetry.addData("Target Motor Position", targetPosition);
        telemetry.addData("Servo Position", servo.getPosition());
        telemetry.addData("PID Error", targetPosition - currentPosition);
        telemetry.update();
    }
}
