package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class SlideControl {

    private DcMotorEx slideMotor;  // Motor for the slide
    private Servo servo;  // GoBilda dual mode servo
    private double targetPosition;  // Target position in degrees
    private double currentPosition;  // Current position in degrees
    private ElapsedTime runtime = new ElapsedTime();

    // PID control variables
    private double kP = 0.05, kI = 0.0, kD = 0.0;  // PID constants
    private double previousError = 0, integral = 0;

    public SlideControl(DcMotorEx slideMotor, Servo servo) {
        this.slideMotor = slideMotor;
        this.servo = servo;
        this.targetPosition = 0;
        this.currentPosition = 0;
    }

    // Function to reset the encoder and set the current position as zero
    public void resetEncoder() {
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentPosition = slideMotor.getCurrentPosition();
        targetPosition = currentPosition;  // Set target as current position
    }

    // Function to update PID control
    public void updatePIDControl() {
        double error = targetPosition - currentPosition;
        integral += error * runtime.seconds();
        double derivative = (error - previousError) / runtime.seconds();

        double power = (kP * error) + (kI * integral) + (kD * derivative);

        slideMotor.setPower(power);
        previousError = error;
    }

    // Set a target position for the slide motor
    public void setTargetPosition(double target) {
        targetPosition = target;
    }

    // Get current position of the motor
    public double getCurrentPosition() {
        return slideMotor.getCurrentPosition();
    }

    // Get current target position
    public double getTargetPosition() {
        return targetPosition;
    }

    // Control the GoBilda dual mode servo
    public void setServoPosition(double position) {
        servo.setPosition(position);
    }

    // Update function to run every loop
    public void update(Telemetry telemetry) {
        currentPosition = slideMotor.getCurrentPosition();
        updatePIDControl();

        // Telemetry updates for monitoring
        telemetry.addData("Current Motor Position", currentPosition);
        telemetry.addData("Target Motor Position", targetPosition);
        telemetry.addData("Servo Position", servo.getPosition());
        telemetry.addData("PID Error", targetPosition - currentPosition);
        telemetry.update();
    }
}
