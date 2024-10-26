package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class PIDTuningTeleOp extends LinearOpMode {
    private boolean halfSpeed = false;
    private boolean lastButtonState = false;
    private double fieldOffset = 0;
    private boolean xHeld = false;

    private double kP = 0.01;
    private double kI = 0.001;
    private double kD = 0;
    private double kF = 0; // Initial gravity compensation factor

    private PIDFController armPID;
    private MotorEx arm;
    private double armTargetPosition = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        arm = new MotorEx(hardwareMap, "arm");
        armPID = new PIDFController(kP, kI, kD, kF);
        arm.stopAndResetEncoder();

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

            // Adjust PID constants using D-pad
            if (gamepad1.dpad_up) {
                kP += 0.0005;
                armPID.setP(kP);
            } else if (gamepad1.dpad_down) {
                kP -= 0.0005;
                armPID.setP(kP);
            }
            if (gamepad1.dpad_right) {
                kI += 0.005;
                armPID.setI(kI);
            } else if (gamepad1.dpad_left) {
                kI -= 0.005;
                armPID.setI(kI);
            }

            // Adjust gravity compensation using gamepad2 bumpers
            if (gamepad2.left_bumper) {
                kF += 0.0005;
                armPID.setF(kF);
            } else if (gamepad2.right_bumper) {
                kF -= 0.0005;
                armPID.setF(kF);
            }

            // Hold X to cut power and set new zero position
            if (gamepad2.x) {
                xHeld = true;
                arm.set(0);  // Cut power to the arm
            } else if (xHeld) {
                // When X is released, set current position as new zero
                armTargetPosition = arm.getCurrentPosition();
                armPID.reset();  // Reset PID controller
                xHeld = false;
            }

            // Update arm control with gravity compensation (unless X is held)
            if (!xHeld) {
                armTargetPosition += -gamepad2.left_stick_y * 2.5;
                double armPower = armPID.calculate(arm.getCurrentPosition(), armTargetPosition);
                arm.set(armPower);
            }

            telemetry.addData("Half-Speed Mode", halfSpeed ? "ON" : "OFF");
            telemetry.addData("Arm Target Position", armTargetPosition);
            telemetry.addData("Arm Power", arm.motor.getPower());
            telemetry.addData("P", kP);
            telemetry.addData("I", kI);
            telemetry.addData("F (Gravity Compensation)", kF);
            telemetry.addData("Heading", botHeading);
            telemetry.update();
        }
    }
}