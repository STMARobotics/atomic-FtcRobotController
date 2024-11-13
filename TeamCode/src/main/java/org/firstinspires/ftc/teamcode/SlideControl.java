package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SlideControl {
    private DcMotorEx slideMotor;

    // Encoder and movement constants
    private static final int ENCODER_TICKS_PER_ROTATION = 1440; // Change based on motor specs
    private static final int DEGREES_PER_ROTATION = 360;

    // PID constants
    private double kP = 0.1, kI = 0.0, kD = 0.0;
    private double integralSum = 0, lastError = 0;

    // Target positions for different modes in degrees
    private static final double HOME_POSITION_DEGREES = 0;
    private static final double PICKUP_POSITION_DEGREES = 360;
    private static final double DROPOFF_POSITION_DEGREES = 720;

    // Current target position in degrees
    private double targetPositionDegrees = HOME_POSITION_DEGREES;

    // Timer for delay functions
    private ElapsedTime timer = new ElapsedTime();

    public SlideControl(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Converts degrees to encoder ticks
    private double degreesToTicks(double degrees) {
        return (degrees / DEGREES_PER_ROTATION) * ENCODER_TICKS_PER_ROTATION;
    }

    // Converts encoder ticks to degrees
    private double ticksToDegrees(int ticks) {
        return (ticks * DEGREES_PER_ROTATION) / (double) ENCODER_TICKS_PER_ROTATION;
    }

    // Reset the zero position (current position as home)
    public void resetZeroPosition() {
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Setters for predefined target positions
    public void setHomePosition() {
        setTargetPositionDegrees(HOME_POSITION_DEGREES);
    }

    public void setPickupPosition() {
        setTargetPositionDegrees(PICKUP_POSITION_DEGREES);
    }

    public void setDropoffPosition() {
        setTargetPositionDegrees(DROPOFF_POSITION_DEGREES);
    }

    // Set custom position in degrees
    public void setTargetPositionDegrees(double targetDegrees) {
        this.targetPositionDegrees = targetDegrees;
    }

    // Get current position in degrees
    public double getCurrentPositionDegrees() {
        return ticksToDegrees(slideMotor.getCurrentPosition());
    }

    // Get the target position in degrees
    public double getTargetPositionDegrees() {
        return targetPositionDegrees;
    }

    // Hold the home position by maintaining motor at HOME_POSITION_DEGREES for 2 seconds
    public void holdHomePosition() {
        setHomePosition();
        timer.reset();
        while (timer.seconds() < 2) {
            updatePID();
        }
    }

    // Main PID update loop
    public void updatePID() {
        // Convert target position in degrees to ticks
        double targetPositionTicks = degreesToTicks(targetPositionDegrees);
        double currentPositionTicks = slideMotor.getCurrentPosition();
        double error = targetPositionTicks - currentPositionTicks;

        // PID calculations
        integralSum += error;
        double derivative = error - lastError;
        lastError = error;

        double pidOutput = (kP * error) + (kI * integralSum) + (kD * derivative);
        slideMotor.setPower(pidOutput);
    }

    // Manually control slide motor power (for testing or manual adjustments)
    public void setMotorPower(double power) {
        slideMotor.setPower(power);
    }

    // Set PID coefficients dynamically
    public void setPIDCoefficients(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    // Check if the slide motor is at the target position within a given tolerance
    public boolean isAtTargetPosition(double toleranceDegrees) {
        double errorDegrees = Math.abs(targetPositionDegrees - getCurrentPositionDegrees());
        return errorDegrees <= toleranceDegrees;
    }

    // Stop the motor
    public void stopMotor() {
        slideMotor.setPower(0);
    }

    // Toggle between home, pickup, and dropoff positions using bumpers
    public void togglePositionWithBumpers(boolean leftBumper, boolean rightBumper) {
        if (leftBumper) {
            setPickupPosition();
        } else if (rightBumper) {
            setDropoffPosition();
        }
    }
}
