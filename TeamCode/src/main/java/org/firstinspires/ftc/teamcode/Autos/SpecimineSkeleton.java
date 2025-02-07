package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

@Autonomous
public class SpecimineSkeleton extends LinearOpMode {

    private IMU imu;
    private double fieldOffset = 0;
    private double RIGHT;
    private double LEFT;

    @Override
    public void runOpMode() {
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        Servo slideServo = hardwareMap.get(Servo.class, "servo");
        final CRServo intake = hardwareMap.get(CRServo.class, "intake");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmControl armControl = new ArmControl(hardwareMap);
        SlideControl slideControl = new SlideControl(slideMotor, slideServo);

        slideControl.resetEncoder();
        armControl.resetZero();

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        fieldOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // move forward so arm goes out
        moveDrivetrainNoStop(frontLeft, rearLeft, frontRight, rearRight, -0.1, 750);

        stopDrivetrain(frontLeft, rearLeft, rearRight, frontRight);

        // slide up
        slideControl.autoSlideMover(-2000);

        // arm out
        armControl.autoArmMover(3000);

        // slide down
        slideControl.autoSlideMover(0);

        // Stop
        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);

        intake.setPower(0.3);

        // hang spec
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, -0.4, 1500);

        // arm a bit down for a sec
        armControl.autoArmMover(3150);

        // back up
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, 0.4, 1500);

        // strafe
        strafeDrivetrain(frontLeft, rearLeft, rearRight, frontRight, 0.4, 1500);

        intake.setPower(0);

        // do yuh stuff idk
        armControl.autoArmMover(4950);

        rotateToAngle(frontLeft, rearLeft, rearRight, frontRight, 0);

//        // If* strafe works
//        strafeDrivetrain(frontLeft, rearLeft, rearRight, frontRight, 0.3, 1000);
//
//        // Move to first
//        moveDrivetrain(frontLeft, rearLeft, rearRight, frontRight, -0.3, 1000);
//
//        // Strafe to first
//        strafeDrivetrain(frontLeft, rearLeft, rearRight, frontRight, 0.3, 500);
//
//        // Put first in area
//        moveDrivetrain(frontLeft, rearLeft, rearRight, frontRight, 0.3, 1000);
//
//        // Go forward
//        moveDrivetrain(frontLeft, rearLeft, rearRight, frontRight, -0.3, 1000);
//
//        // Strafe to 2nd
//        strafeDrivetrain(frontLeft, rearLeft, rearRight, frontRight, 0.3, 500);
//
//        // Put 2nd in area
//        moveDrivetrain(frontLeft, rearLeft, rearRight, frontRight, 0.3, 1000);
//
//        // Go forward
//        moveDrivetrain(frontLeft, rearLeft, rearRight, frontRight, -0.3, 1000);
//
//        // Strafe to 3rd
//        strafeDrivetrain(frontLeft, rearLeft, rearRight, frontRight, 0.3, 500);
//
//        // Put 3rd in area
//        moveDrivetrain(frontLeft, rearLeft, rearRight, frontRight, -0.3, 1000);
//
//        // Go forward
//        moveDrivetrain(frontLeft, rearLeft, rearRight, frontRight, 0.3, 1000);
//

        strafeDrivetrain(frontLeft, rearLeft, rearRight, frontRight, 0.1, 10000);
        rotateToAngle(frontLeft, rearLeft, rearRight, frontRight, 90);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
        slideControl.update();
    }

    private void moveDrivetrain(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double power, int duration) {
        frontLeft.setPower(power);
        rearLeft.setPower(power);
        frontRight.setPower(power);
        rearRight.setPower(power);
        sleep(duration);
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }

    private void moveDrivetrainNoStop(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double power, int duration) {
        frontLeft.setPower(power);
        rearLeft.setPower(power);
        frontRight.setPower(power);
        rearRight.setPower(power);
        sleep(duration);
    }

    private void strafeDrivetrain(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double power1, int duration) {
        frontLeft.setPower(-power1);
        rearLeft.setPower(power1);
        frontRight.setPower(-power1);
        rearRight.setPower(power1);
        sleep(duration);
    }

    private void stopDrivetrain(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight) {
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }

    private void rotateToAngle(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double targetAngle) {
        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double currentAngle;
        double turnPower;
        double error;

        while (opModeIsActive()) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - fieldOffset;

            error = targetAngle - currentAngle;
            error = ((error + 180) % 360 + 360) % 360 - 180;

            if (Math.abs(error) <= 2) {
                break;
            }

            turnPower = 0.65 * (Math.abs(error) / 45.0);
            turnPower = Math.max(0.1, turnPower);

            if (error > 0) {
                frontLeft.setPower(turnPower);
                rearLeft.setPower(turnPower);
                frontRight.setPower(-turnPower);
                rearRight.setPower(-turnPower);
            } else {
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

        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);
    }

}
