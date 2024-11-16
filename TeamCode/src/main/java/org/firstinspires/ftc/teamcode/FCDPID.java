package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;

@TeleOp
public class FCDPID extends LinearOpMode {
    private boolean halfSpeed = false;
    private boolean lastButtonState = false;
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

        while (opModeIsActive()) {
            boolean currentButtonState = gamepad1.right_stick_button;
            if (currentButtonState && !lastButtonState) {
                halfSpeed = !halfSpeed;
            }
            lastButtonState = currentButtonState;

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x; // Rotation input
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

            // Check if the bot is facing roughly forward/backward
            boolean isFacingForwardBackward = Math.abs(botHeading) < 45 || Math.abs(botHeading) > 315;

            if (isFacingForwardBackward) {
                // Normal movement controls
                double temp = y * Math.cos(headingRad) - x * Math.sin(headingRad);
                x = y * Math.sin(headingRad) + x * Math.cos(headingRad);
                y = temp;
            } else {
                // Reverse left/right controls
                double temp = y * Math.cos(headingRad) + x * Math.sin(headingRad);
                x = -y * Math.sin(headingRad) + x * Math.cos(headingRad);
                y = temp;
            }

            // Set a constant turning speed
            double turningSpeed = 0.3;
            if (Math.abs(rx) > 0.1) { // Only apply turning when there is input
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

            if (gamepad2.dpad_up) {
                targetArmPosition -= 10;
            } else if (gamepad2.dpad_down) {
                targetArmPosition += 10;
            } else if (gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1) {
                targetArmPosition += gamepad2.right_stick_y * 30;
            }


            armControl.setPosition(targetArmPosition);
            armControl.update();

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
