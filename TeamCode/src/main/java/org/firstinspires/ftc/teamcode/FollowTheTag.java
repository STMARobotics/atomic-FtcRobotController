package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Limelight3A Distance Control", group = "Sensor")
public class FollowTheTag extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor frontRight, rearRight, rearLeft, frontLeft;

    // Target distance (1 meter) in meters
    private static final double TARGET_DISTANCE = 1.0;

    // Control constants
    private static final double ALIGNMENT_KP = 0.03; // Proportional gain for alignment
    private static final double DISTANCE_KP = 0.05;  // Proportional gain for distance
    private static final double MIN_POWER = 0.1;    // Minimum power to avoid stalling
    private static final double MAX_POWER = 0.5;    // Limit maximum speed

    // Known constants for Limelight camera
    private static final double LIMELIGHT_HEIGHT = 0.5;  // Height of the Limelight camera in meters
    private static final double LIMELIGHT_ANGLE = 20.0;  // Camera pitch angle in degrees

    @Override
    public void runOpMode() {
        // Initialize hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        // Configure motors
        setMotorDirections();

        // Start Limelight polling
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Fetch the horizontal offset (tx) from the Limelight
                double tx = result.getTx();  // Horizontal alignment in degrees

                // Convert tx to radians for trigonometric calculations
                double txRadians = Math.toRadians(tx);

                // Calculate distance using trigonometry (simple 2D calculation)
                double distance = LIMELIGHT_HEIGHT / Math.tan(txRadians);

                // Calculate the alignment and distance correction
                double alignmentCorrection = tx * ALIGNMENT_KP;
                double distanceCorrection = (distance - TARGET_DISTANCE) * DISTANCE_KP;

                // Apply power limits
                alignmentCorrection = clamp(alignmentCorrection);
                distanceCorrection = clamp(distanceCorrection);

                // Calculate motor powers for mecanum drive
                double frontLeftPower = distanceCorrection + alignmentCorrection;
                double frontRightPower = distanceCorrection - alignmentCorrection;
                double rearLeftPower = distanceCorrection + alignmentCorrection;
                double rearRightPower = distanceCorrection - alignmentCorrection;

                // Set motor powers to move the robot
                setMotorPowers(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);

                // Telemetry for debugging
                telemetry.addData("Calculated Distance", "%.2f meters", distance);
                telemetry.addData("Alignment (tx)", "%.2f", tx);
                telemetry.addData("FL Power", frontLeftPower);
                telemetry.addData("FR Power", frontRightPower);
                telemetry.addData("RL Power", rearLeftPower);
                telemetry.addData("RR Power", rearRightPower);
            } else {
                telemetry.addData("Limelight", "No valid target detected");
                stopMotors();
            }

            telemetry.update();
        }

        limelight.stop();
    }

    /**
     * Sets the directions for mecanum motors.
     */
    private void setMotorDirections() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Sets power to all four mecanum motors.
     */
    private void setMotorPowers(double fl, double fr, double rl, double rr) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        rearLeft.setPower(rl);
        rearRight.setPower(rr);
    }

    /**
     * Stops all motors.
     */
    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    /**
     * Clamps a value between a minimum and maximum value.
     */
    private double clamp(double value) {
        return Math.max(MIN_POWER, Math.min(MAX_POWER, value));
    }
}
