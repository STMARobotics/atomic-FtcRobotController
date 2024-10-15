package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    private boolean halfSpeed = false;
    private boolean lastButtonState = false;
    private double armSpeed = 0.0;
    private double fieldOffset = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve and initialize the IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

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
                rx *= 0.3;
            }

            // Reset field-centric forward position with gamepad1 Y button
            if (gamepad1.y) {
                fieldOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - fieldOffset;
            double headingRad = Math.toRadians(botHeading);
            double temp = y * Math.cos(headingRad) - x * Math.sin(headingRad);
            x = y * Math.sin(headingRad) + x * Math.cos(headingRad);
            y = temp;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontRightPower = (y + x + rx) / denominator;
            double rearRightPower = (y - x + rx) / denominator;
            double rearLeftPower = (y + x - rx) / denominator;
            double frontLeftPower = (y - x - rx) / denominator;
            double intakePower = (iF + iR) / denominator;

            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);
            rearLeft.setPower(rearLeftPower);
            frontLeft.setPower(frontLeftPower);
            intake.setPower(intakePower);

            // Adjust arm speed based on D-pad input
            if (gamepad2.dpad_up) {
                armSpeed += 0.005;
            } else if (gamepad2.dpad_down) {
                armSpeed -= 0.005;
            } else if (gamepad2.dpad_right) {
                armSpeed = 0.0;
            }
            armSpeed = Math.max(-1.0, Math.min(1.0, armSpeed));
            arm.setPower(armSpeed);

            telemetry.addData("Half-Speed Mode", halfSpeed ? "ON" : "OFF");
            telemetry.addData("Arm Speed", armSpeed);
            telemetry.addData("Heading", botHeading);
            telemetry.update();
        }
    }
}
