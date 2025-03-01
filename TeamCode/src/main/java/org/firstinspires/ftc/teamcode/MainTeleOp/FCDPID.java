package org.firstinspires.ftc.teamcode.MainTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

@TeleOp
public class FCDPID extends LinearOpMode {
    private boolean halfSpeed = false;
    private boolean lastButtonState = false;
    private double fieldOffset = 0;
    double targetSlidePosition;
    private double targetArmPosition = 0;
    double targetServoPosition = 65;
    private DcMotor rearRight;
    private DcMotor rearLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private IMU imu;
    private int presetCycle = 0;
    private double lastPresetCycle = 0;
    private boolean lastLeftBumperState = false;
    private boolean lastRightBumperState = false;
    private boolean settingSlideTargetHigher;
    private boolean turningOffSlide;


    @Override
    public void runOpMode() {
        final DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        final DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        final DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        final DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        final CRServo intake = hardwareMap.get(CRServo.class, "intake");
        final DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        final Servo slideServo = hardwareMap.get(Servo.class, "servo");
        SlideControl slideControl = new SlideControl(slideMotor, slideServo);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        final IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        ArmControl armControl = new ArmControl(hardwareMap);
        armControl.resetZero();

        waitForStart();
        if (isStopRequested()) return;

        slideControl.resetEncoder();
        telemetry.addData("init", true);
        telemetry.update();

        while (opModeIsActive()) {
            boolean currentButtonState = gamepad1.right_stick_button;
            if (currentButtonState && !lastButtonState) {
                halfSpeed = !halfSpeed;
            }

            lastButtonState = currentButtonState;

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double iF = gamepad2.right_trigger;
            double iR = gamepad2.left_trigger * -1;

            if (halfSpeed) {
                y *= 0.3;
                x *= 0.3;
            }

            if (gamepad1.y) {
                fieldOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - fieldOffset;
            double headingRad = Math.toRadians(botHeading);

            {
                double temp = y * Math.cos(headingRad) + x * Math.sin(headingRad);
                x = -y * Math.sin(headingRad) + x * Math.cos(headingRad);
                y = temp;
            }

            double turningSpeed = 0.3;

            if (Math.abs(rx) > 0.1) {
                rx = turningSpeed * Math.signum(rx);
            } else {
                rx = 0;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontRightPower = (y + x + rx) / denominator;
            double rearRightPower = (y - x + rx) / denominator;
            double rearLeftPower = (y + x - rx) / denominator;
            double frontLeftPower = (y - x - rx) / denominator;
            double intakePower = (iF + iR);


            if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad2.right_bumper && gamepad2.left_bumper) {
                frontRightPower = 999;
                rearRightPower = 999;
                rearLeftPower = -999;
                frontLeftPower = -999;
            }
            if (gamepad1.back) {
                frontRightPower = 0;
                rearRightPower = 0;
                rearLeftPower = 0;
                frontLeftPower = 0;
            }

            slideControl.getCurrentPosition();

            if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
                targetSlidePosition += gamepad2.left_stick_y * 30;
                slideControl.setTargetPosition(targetSlidePosition);
            }

            if (gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1) {
                targetArmPosition += gamepad2.right_stick_y * 70;
                armControl.setPosition(targetArmPosition);
            }

            boolean isArmBeingControlled = (Math.abs(gamepad2.right_stick_y) > 0.1);

            if (gamepad2.dpad_up) {
                targetServoPosition = -10;
                slideControl.setServoPosition(targetServoPosition);
            }

            if (gamepad2.dpad_right) {
                targetServoPosition = 65;
                slideControl.setServoPosition(targetServoPosition);
            }

            if (gamepad2.dpad_down) {
                targetServoPosition = -80;
                slideControl.setServoPosition(targetServoPosition);
            }

            if (gamepad2.y) {
                targetSlidePosition = -3550;
                slideControl.setTargetPosition(targetSlidePosition);
            }

            if (gamepad2.a) {
                targetSlidePosition = -15;
                slideControl.setTargetPosition(targetSlidePosition);
            }

            if (gamepad2.b) {
                targetSlidePosition = -1820;
                slideControl.setTargetPosition(targetSlidePosition);
            }

            if (gamepad2.right_bumper && !lastRightBumperState) {
                presetCycle -= 1;
            }

            if (gamepad2.left_bumper && !lastLeftBumperState) {
                presetCycle += 1;
            }

            lastLeftBumperState = gamepad2.left_bumper;
            lastRightBumperState = gamepad2.right_bumper;


            if (presetCycle < 1) {
                presetCycle = 1;
            }

            if (presetCycle > 4) {
                presetCycle = 4;
            }

            if (presetCycle != lastPresetCycle && !isArmBeingControlled) {
                switch (presetCycle) {
                    case 1:
                        targetArmPosition = 0;
                        break;
                    case 2:
                        targetArmPosition = -775;
                        break;
                    case 3:
                        targetArmPosition = -1815;
                        break;
                    case 4:
                        targetArmPosition = -3350;
                        break;
                    default:
                        telemetry.addData("preset", "none selected");
                        telemetry.update();
                        break;
                }
                armControl.setPosition(targetArmPosition);
                lastPresetCycle = presetCycle; // Update the lastPresetCycle after acting on it.
            }


            if (targetSlidePosition > -1) {
                targetSlidePosition = -1;
                slideControl.setTargetPosition(targetSlidePosition);
            }

            if (targetSlidePosition < -3800) {
                targetSlidePosition = -3800;
                slideControl.setTargetPosition(targetSlidePosition);
            }

            if (targetSlidePosition > -1) {
                slideControl.setSlidePower(0);
            }

            if (slideControl.getSlideAmps() > 4) {
                targetSlidePosition += -5;
                slideControl.setTargetPosition(targetSlidePosition);
                boolean settingSlideTargetHigher = true;
            } else {
                settingSlideTargetHigher = false;
            }

            if (slideControl.getSlideAmps() > 5) {
                slideControl.setSlidePower(0);
                boolean turningOffSlide = true;
            } else {
                turningOffSlide = false;
            }


            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);
            rearLeft.setPower(rearLeftPower);
            frontLeft.setPower(frontLeftPower);
            intake.setPower(intakePower);

//            slideControl.setTargetPosition(targetSlidePosition);
//            slideControl.setServoPosition(targetServoPosition);
            armControl.setPosition(targetArmPosition);
            armControl.update();
            slideControl.update();

            String emptyVariable = " ";

            telemetry.addData("Half-Speed Mode", halfSpeed ? "ON" : "OFF");
            telemetry.addData("", emptyVariable);
            telemetry.addData("Arm Target Position", armControl.getArmTargetPosition());
            telemetry.addData("Arm Position", armControl.getArmPosition());
            telemetry.addData("", emptyVariable);
            telemetry.addData("Slide Target Position", slideControl.getTargetPosition());
            telemetry.addData("Slide Position", slideControl.getCurrentPosition());
            telemetry.addData("Servo Position", targetServoPosition);
            telemetry.addData("Slide Amp Draw", slideControl.getSlideAmps());
            telemetry.addData("", emptyVariable);
            telemetry.addData("Minor slide fix", settingSlideTargetHigher);
            telemetry.addData("turning off slide (warning)", turningOffSlide);
            telemetry.addData("", emptyVariable);
            telemetry.addData("Heading", botHeading);
            telemetry.addData("", emptyVariable);
            telemetry.update();
        }
    }
}
