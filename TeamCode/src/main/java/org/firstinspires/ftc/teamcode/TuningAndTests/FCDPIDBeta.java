package org.firstinspires.ftc.teamcode.TuningAndTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

class IntakeControl {
    private DcMotorEx intakeMotor;
    private double normalPower = 0.45; // Your normal max power
    private double minPower = 0.1;    // Minimum power when under strain
    private double currentThreshold = 7; // Amps - adjust based on your motor
    private double recoveryTime = 1000; // Time in ms before returning to normal power
    private long lastStrainTime = 0;

    public IntakeControl(DcMotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    public double getAdjustedIntakePower(double requestedPower) {
        double currentDraw = intakeMotor.getCurrent(CurrentUnit.AMPS);
        long currentTime = System.currentTimeMillis();

        if (currentDraw > currentThreshold) {
            lastStrainTime = currentTime;
            double scaleFactor = minPower / normalPower;
            return requestedPower * scaleFactor;
        }
        else if (currentTime - lastStrainTime < recoveryTime) {
            double timeScale = (currentTime - lastStrainTime) / recoveryTime;
            double scaleFactor = minPower + (normalPower - minPower) * timeScale;
            return requestedPower * (scaleFactor / normalPower);
        }

        return requestedPower;
    }

    public double getCurrentDraw() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }
}

@TeleOp
public class FCDPIDBeta extends LinearOpMode {
    private boolean halfSpeed = false;
    private boolean lastButtonState = false;
    private double fieldOffset = 0;
    private ArmControl armControl;
    private SlideControl slideControl;
    private IntakeControl intakeControl;
    double targetSlidePosition;
    private double targetArmPosition = 0;
    double targetServoPosition = 65;

    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        final DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        final DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        final DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        final DcMotorEx intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        final DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        final Servo slideServo = hardwareMap.get(Servo.class, "servo");

        // Initialize IntakeControl
        intakeControl = new IntakeControl(intake);
        slideControl = new SlideControl(slideMotor, slideServo);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        final IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        armControl = new ArmControl(hardwareMap);
        armControl.resetZero();

        waitForStart();
        if (isStopRequested()) return;

        slideControl.resetEncoder();

        while (opModeIsActive()) {
            boolean currentButtonState = gamepad1.right_stick_button;
            if (currentButtonState && !lastButtonState) {
                halfSpeed = !halfSpeed;
            }
            lastButtonState = currentButtonState;

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double iF = gamepad2.right_trigger * -0.45;
            double iR = gamepad2.left_trigger * 0.45;

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

            // Use IntakeControl to adjust power based on strain
            double adjustedIntakePower = intakeControl.getAdjustedIntakePower(intakePower);

            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);
            rearLeft.setPower(rearLeftPower);
            frontLeft.setPower(frontLeftPower);
            intake.setPower(adjustedIntakePower);

            double currentSlidePosition = slideControl.getCurrentPosition();
            if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
                targetSlidePosition += gamepad2.left_stick_y * 30;
            }

            if (gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1) {
                targetArmPosition += gamepad2.right_stick_y * 30;
            }

            if (gamepad2.dpad_up) {
                targetServoPosition = -10;
            }

            if (gamepad2.dpad_right) {
                targetServoPosition = 65;
            }

            if (gamepad2.dpad_down) {
                targetServoPosition = -80;
            }

            if (gamepad2.y) {
                targetSlidePosition = -3550;
            }

            if (gamepad2.a) {
                targetSlidePosition = -5;
            }

            if (gamepad2.b) {
                targetSlidePosition = -1820;
            }

            if (targetSlidePosition > 10) {
                targetSlidePosition = 0;
            }

            if (targetSlidePosition < -3800) {
                targetSlidePosition = -3800;
            }

            slideControl.setTargetPosition(targetSlidePosition);
            slideControl.setServoPosition(targetServoPosition);
            armControl.setPosition(targetArmPosition);
            armControl.update();
            slideControl.update();

            String emptyVariable = " ";
            
            telemetry.addData("Half-Speed Mode", halfSpeed ? "ON" : "OFF");
            telemetry.addData("", emptyVariable);
            telemetry.addData("Arm Target Position", armControl.getArmTargetPosition());
            telemetry.addData("Arm Position", armControl.getArmPosition());
            telemetry.addData("Arm Power", armControl.getArmPower());
            telemetry.addData("", emptyVariable);
            telemetry.addData("Slide Target Position", slideControl.getTargetPosition());
            telemetry.addData("Slide Position", slideControl.getCurrentPosition());
            telemetry.addData("Servo Position", targetServoPosition);
            telemetry.addData("", emptyVariable);
            telemetry.addData("Heading", botHeading);
            telemetry.addData("", emptyVariable);
            telemetry.addData("Intake Current Draw", intakeControl.getCurrentDraw());
            telemetry.addData("Adjusted Intake Power", adjustedIntakePower);
            telemetry.update();
        }
    }
}