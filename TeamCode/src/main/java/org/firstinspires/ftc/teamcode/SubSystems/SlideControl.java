package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SlideControl {

    private DcMotorEx slide;  // Motor for the slide
    private Servo servo;  // GoBilda dual mode servo
    private double targetPosition;  // Target position in degrees
    private double currentPosition;  // Current position in degrees
    private ElapsedTime runtime = new ElapsedTime();

    // PID control variables
    private double kP = 0.0075, kI = 0, kD = 0.0;  // PID constants
    private double previousError = 0, integral = 0;

    public SlideControl(DcMotorEx slide, Servo servo) {
        this.slide = slide;
        this.servo = servo;
        this.targetPosition = 0;
        this.currentPosition = 0;
    }

    // Function to reset the encoder and set the current position as zero
    public void resetEncoder() {
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        currentPosition = slide.getCurrentPosition();
        targetPosition = currentPosition;  // Set target as current position
    }

    // Function to update PID control
    public void updatePIDControl() {
        double error = targetPosition - currentPosition;
        integral += error * runtime.seconds();
        double derivative = (error - previousError) / runtime.seconds();

        double power = (kP * error) + (kI * integral) + (kD * derivative);

        slide.setPower(power);
        previousError = error;
    }

    // Set a target position for the slide motor
    public void setTargetPosition(double target) {
        targetPosition = target;
    }

    // Get current position of the motor
    public double getCurrentPosition() {
        return slide.getCurrentPosition();
    }

    // Get current target position
    public double getTargetPosition() {
        return targetPosition;
    }

    // Control the GoBilda dual mode servo
    public void setServoPosition(double position) {
        // Ensure the position stays within the range of -90 to 90 degrees
        position = Math.max(-90, Math.min(90, position));

        // Map -90 to 90 degrees to the range of -1 to 1 for the servo
        double mappedPosition = (position + 90) / 180.0;

        servo.setPosition(mappedPosition);
    }

    // Update function to run every loop
    public void update() {
        currentPosition = slide.getCurrentPosition();
        updatePIDControl();
    }

//    public void servoPosition(double currentPosition) {
//
//    }

    // Telemetry update function
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Current Motor Position", currentPosition);
        telemetry.addData("Target Motor Position", targetPosition);
        telemetry.addData("Servo Position", servo.getPosition());
        telemetry.addData("PID Error", targetPosition - currentPosition);
        telemetry.update();
    }
}
