package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;

@TeleOp
public class FCDPIDbeta extends LinearOpMode {
    private boolean halfSpeed = false;
    private boolean lastButtonState = false;
    private boolean rightBumperLocked = true;
    private double fieldOffset = 0;
    private ArmControl armControl;
    private double targetArmPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        final DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        final DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        final DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        final DcMotor intake = hardwareMap.dcMotor.get("intake");
        final DcMotor slide = hardwareMap.dcMotor.get("slide"); // Slide motor
        final Servo basket = hardwareMap.servo.get("basket"); // Basket servo

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        basket.setPosition(0); // Ensure servo is at position 0 initially

        final IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        armControl = new ArmControl(hardwareMap);
        armControl.resetZero();

        waitForStart();
        if (isStopRequested()) return;

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

            boolean isFacingForwardBackward = Math.abs(botHeading) < 45 || Math.abs(botHeading) > 315;

            if (isFacingForwardBackward) {
                double temp = y * Math.cos(headingRad) - x * Math.sin(headingRad);
                x = y * Math.sin(headingRad) + x * Math.cos(headingRad);
                y = temp;
            } else {
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

            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);
            rearLeft.setPower(rearLeftPower);
            frontLeft.setPower(frontLeftPower);
            intake.setPower(intakePower);

            // Arm control logic
            if (gamepad2.dpad_up) {
                targetArmPosition -= 2;
            } else if (gamepad2.dpad_down) {
                targetArmPosition += 2;
            } else if (gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1) {
                targetArmPosition += gamepad2.right_stick_y * 5;
            }

            armControl.setPosition(targetArmPosition);
            armControl.update();

            // Slide motor control
            if (gamepad2.left_bumper) {
                slide.setPower(-1.0); // Reverse motor
                sleep(3000); // Run for 3 seconds
                slide.setPower(0);
                rightBumperLocked = false;
            }

            if (!rightBumperLocked && gamepad2.right_bumper) {
                slide.setPower(1.0); // Forward motor
                sleep(3000); // Run for 3 seconds
                slide.setPower(0);
                rightBumperLocked = true;
            }

            // Basket servo control in locked mode
            if (rightBumperLocked && gamepad2.right_bumper) {
                if (basket.getPosition() == 0) {
                    basket.setPosition(1); // Move to 180 degrees
                } else {
                    basket.setPosition(0); // Move back to 0 degrees
                }
                sleep(500); // Small delay for servo to reach position
            }

            telemetry.addData("Half-Speed Mode", halfSpeed ? "ON" : "OFF");
            telemetry.addData("Arm Target Position", armControl.getArmTargetPosition());
            telemetry.addData("Arm Power", armControl.getArmPower());
            telemetry.addData("Arm Position", armControl.getArmPosition());
            telemetry.addData("Heading", botHeading);
            telemetry.addData("Battery Voltage", String.format("%.2f V", hardwareMap.voltageSensor.get("Control Hub").getVoltage()));
            telemetry.update();
        }
    }
}
