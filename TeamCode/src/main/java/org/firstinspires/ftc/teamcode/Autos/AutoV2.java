package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;
import org.firstinspires.ftc.teamcode.SubSystems.AutoSubsystem;

@Autonomous
public class AutoV2 extends LinearOpMode {

    private AutoSubsystem autoSubsystem;
    private ArmControl armControl;
    private SlideControl slideControl;
    private IMU imu;
    private double fieldOffset = 0;
    double targetSlidePosition;
    double targetServoPosition;
    double emptyVariable;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        Servo slideServo = hardwareMap.get(Servo.class, "servo");
//        rearRight.isBusy();

        // Initialize subsystems
        armControl = new ArmControl(hardwareMap);
        slideControl = new SlideControl(slideMotor, slideServo);

        slideControl.resetEncoder();
        armControl.resetZero();

        // Reverse motor directions
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Wait for start
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // Record the initial heading
        fieldOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        //set servo to home
        slideControl.setServoPosition(65);

        // Autonomous sequence
        // Move forward
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, -0.3, 1600);

        // Rotate to -45 degrees
        rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, -45);

        // Go back
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, 0.3, 1450);
        stopDrivetrain(frontLeft, rearLeft, rearRight, frontRight);

//        // Move to high basket
//        targetSlidePosition = -3550;
//        slideControl.setTargetPosition(targetSlidePosition);
//        waitForSlideToReachTarget(slideControl, targetSlidePosition);
//
//        // Dump the block
//        slideControl.setServoPosition(-80);
//        slideControl.update();
//        sleep(1750);
//
//        // Set bucket to home position
//        targetServoPosition = -10; // Normalized value for 10 degrees
//        slideControl.setServoPosition(targetServoPosition);
//        sleep(250);
//
//        // Put slide down
//        targetSlidePosition = 0;
//        slideControl.setTargetPosition(targetSlidePosition);
//        waitForSlideToReachTarget(slideControl, targetSlidePosition);
//        sleep(500); // Allow time for slide to return to base

        //beta test of the AutoSubsystem
        autoSubsystem.dumpSampleHigh();

        rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, 0);

        // Stop all motors
        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);

        // Telemetry to indicate completion
        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
        slideControl.update();
    }

    private void moveDrivetrain(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double power, int duration) throws InterruptedException {
        frontLeft.setPower(power);
        rearLeft.setPower(power);
        frontRight.setPower(power);
        rearRight.setPower(power);
        sleep(duration);
    }

    private void stopDrivetrain(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight) {
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }

    private void rotateToAngle(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double targetAngle) throws InterruptedException {
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - fieldOffset;
        double turnPower;
        double error;

        while (opModeIsActive()) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - fieldOffset;

            // Calculate angle difference (normalized to [-180, 180])
            error = targetAngle - currentAngle;
            error = ((error + 180) % 360 + 360) % 360 - 180; // Normalize to [-180, 180]

            if (Math.abs(error) <= 2) { // Stop when within tolerance of 5 degrees
                break;
            }

            // Gradual slowdown: Adjust turn power based on error magnitude
            turnPower = 0.65 * (Math.abs(error) / 45.0); // Scale power (0.3 at 45° error, lower as it approaches target)
            turnPower = Math.max(0.1, turnPower); // Ensure a minimum power to overcome inertia

            if (error > 0) {
                // Rotate clockwise
                frontLeft.setPower(turnPower);
                rearLeft.setPower(turnPower);
                frontRight.setPower(-turnPower);
                rearRight.setPower(-turnPower);
            } else {
                // Rotate counter-clockwise
                frontLeft.setPower(-turnPower);
                rearLeft.setPower(-turnPower);
                frontRight.setPower(turnPower);
                rearRight.setPower(turnPower);
            }

            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        // Stop all motors once target is reached
        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);
    }

    public void waitForSlideToReachTarget(SlideControl slideControl, double targetPosition) throws InterruptedException {
        while (opModeIsActive() && Math.abs(slideControl.getCurrentPosition() - targetPosition) > 1) {
            slideControl.update();
            telemetry.addData("Current Slide Position", slideControl.getCurrentPosition());
            telemetry.addData("Target Slide Position", targetPosition);
            telemetry.addData("Slide Error", Math.abs(slideControl.getCurrentPosition() - targetPosition));
            telemetry.update();
            sleep(50); // Small delay to avoid overloading the system
        }
    }
}
