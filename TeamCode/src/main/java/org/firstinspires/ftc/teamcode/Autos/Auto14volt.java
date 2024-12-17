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
import org.firstinspires.ftc.teamcode.SubSystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

@Autonomous
public class Auto14volt extends LinearOpMode {

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
        armControl = new ArmControl(hardwareMap);
        slideControl = new SlideControl(slideMotor, slideServo);

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

        slideControl.setServoPosition(0);

        // move to basket
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, -0.6, 400);

        rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, -45);

        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, 0.6, 625);
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
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, -0.3, 750);
        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);
        intake.setPower(0);
        armControl.autoArmMover(1540);
        intake.setPower(-1);
        sleep(300);
        intake.setPower(0);
        armControl.autoArmMover(4950);
        slideControl.setServoPosition(0);

        //move to drop 2nd
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, 0.3, 500);

        rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, -45);

        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);

        //drop 2nd
        slideControl.autoSlideMover(-3550);
        slideControl.setServoPosition(-80);
        sleep(850);
        slideControl.setServoPosition(0);
        slideControl.autoSlideMover(-10);

        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);

        rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, 12);

        //grab 3rd and drop 3rd
        armControl.autoArmMover(4950);
        intake.setPower(1);
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, -0.3, 650);
        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);
        intake.setPower(0);
        armControl.autoArmMover(1540);
        intake.setPower(-1);
        sleep(300);
        intake.setPower(0);
        armControl.autoArmMover(4950);
        moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, 0.3, 500);
        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);
        rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, -45);
        slideControl.autoSlideMover(-3550);
        slideControl.setServoPosition(-80);
        sleep(850);
        slideControl.setServoPosition(0);
        slideControl.autoSlideMover(0);

        rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, 0);

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

    public void dontWaitForSlide(){
        slideControl.setTargetPosition(0);

    }

    public void waitForSlideToReachTarget(SlideControl slideControl, double targetPosition) throws InterruptedException {
        while (opModeIsActive() && Math.abs(slideControl.getCurrentPosition() - targetPosition) > 1) {
            slideControl.update();
            telemetry.addData("Current Slide Position", slideControl.getCurrentPosition());
            telemetry.addData("Target Slide Position", targetPosition);
            telemetry.addData("Slide Error", Math.abs(slideControl.getCurrentPosition() - targetPosition));
            telemetry.update();
            sleep(50);
        }
    }
}
