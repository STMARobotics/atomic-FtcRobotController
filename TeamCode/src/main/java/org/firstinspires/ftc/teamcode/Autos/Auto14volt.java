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
public class Auto14volt extends LinearOpMode {

    private IMU imu;
    private double fieldOffset = 0;

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

        // Start auto
        fieldOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        slideControl.setServoPosition(0);

        // drive from wall
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, -0.6, 400); //update

        rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, -45);

        // drive backwards to basket
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, 0.6, 625); //update
        stopDrivetrain(frontLeft, rearLeft, rearRight, frontRight);

        //drop 1st sample in basket
        slideControl.autoSlideMover(-3550);
        slideControl.setServoPosition(-80);
        sleep(850);
        slideControl.setServoPosition(-10);
        armControl.autoArmMover(4950);
        slideControl.autoSlideMover(-10);

        //rotate to pickup 2nd
        rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, -11);

        //grab second
        intake.setPower(1);

        //drive forward; grab the 2nd block (first on field)
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, -0.3, 750); //update
        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);
        intake.setPower(0);
        armControl.autoArmMover(1540);
        intake.setPower(-1);
        sleep(300);
        intake.setPower(0);
        armControl.autoArmMover(4950);
        slideControl.setServoPosition(0);

        //drive backwards to basket; drop 2nd reverse
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, 0.3, 500); //update

        rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, -45);

        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);

        //drop 2nd
        slideControl.autoSlideMover(-3550);
        slideControl.setServoPosition(-80);
        sleep(850);
        slideControl.setServoPosition(0);
        slideControl.autoSlideMover(-30); //here

        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);

        rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, 9);

        //grab 3rd and drop 3rd
        armControl.autoArmMover(4950);
        intake.setPower(1);

        //drive forward to grab 3rd (2nd on field)
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, -0.3, 650); //update
        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);
        intake.setPower(0);
        armControl.autoArmMover(1540);
        intake.setPower(-1);
        sleep(300);
        intake.setPower(0);
        armControl.autoArmMover(4950);

        // drive backwards to basket 3rd
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, 0.3, 500); //update
        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);
        rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, -45);
        slideControl.autoSlideMover(-3550);
        slideControl.setServoPosition(-80);
        sleep(850);
        slideControl.setServoPosition(0);
        slideControl.autoSlideMover(-30);

        rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, 0);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
        slideControl.update();
    }
//random
    private void moveDrivetrain(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double power, int duration) {
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
//            telemetry.addData("Slide Position", );
            telemetry.update();
        }

        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);
    }

}
