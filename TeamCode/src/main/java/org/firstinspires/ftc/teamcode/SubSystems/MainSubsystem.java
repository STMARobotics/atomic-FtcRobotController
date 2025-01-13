package org.firstinspires.ftc.teamcode.SubSystems;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Date;

public class MainSubsystem {

    private ArmControl armControl;
    private SlideControl slideControl;
    private IMU imu;
    private VoltageSensor batterySensor;
    private HardwareMap hardwareMap;
    private boolean isCalibrated = false;
    private double armError;

    public MainSubsystem(HardwareMap hardwareMap) {
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.hardwareMap = hardwareMap;

        armControl = new ArmControl(hardwareMap);
        slideControl = new SlideControl(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        batterySensor = hardwareMap.voltageSensor.iterator().next();
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

    }

    public double getArmPosition() {
        return armControl.getArmPosition();
    }

    public double getArmTargetPosition() {
        return armControl.getArmTargetPosition();
    }

    public double getArmError() {
        // return armControl.getArmTargetPosition() - armControl.getArmPosition();
        return Math.abs(armControl.getArmTargetPosition()) - Math.abs(armControl.getArmPosition());
    }

    public double getSlidePosition() {
        return slideControl.getCurrentPosition();
    }

    public double getSlideTargetPosition() {
        return slideControl.getTargetPosition();
    }

    public double getSlideError() {
        // return slideControl.getTargetPosition() - slideControl.getCurrentPosition();
        return Math.abs(slideControl.getTargetPosition()) - Math.abs(slideControl.getCurrentPosition());
    }

    public void calibrate() {
        armControl.resetZero();
        slideControl.resetEncoder();
        isCalibrated = true;
    }

    public void moveDrivetrain(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double power, int duration) {
        frontLeft.setPower(power);
        rearLeft.setPower(power);
        frontRight.setPower(power);
        rearRight.setPower(power);
        safeSleep(duration);
    }

    public void stopDrivetrain(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight) {
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }

    private void safeSleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void rotateToAngle(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double targetAngle) {
        double fieldOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - fieldOffset;
        double turnPower;
        double error = targetAngle - currentAngle;

        while (error>3) {
            fieldOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
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

        }

        stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);
    }

    public void logData(Gamepad gamepad1, Gamepad gamepad2) {
        try (FileWriter writer = new FileWriter("logData.csv", true)) {
            String timestamp = new Date().toString();

            double imuX = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
            double imuY = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
            double imuZ = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            double batteryLevel = batterySensor.getVoltage();

            double armPosition = armControl.getArmPosition();
            double slidePosition = slideControl.getCurrentPosition();

            String gamepad1Buttons = getGamepadButtonStates(gamepad1);
            String gamepad2Buttons = getGamepadButtonStates(gamepad2);
            double joystick1X = gamepad1.left_stick_x;
            double joystick1Y = gamepad1.left_stick_y;
            double joystick2X = gamepad2.left_stick_x;
            double joystick2Y = gamepad2.left_stick_y;

            writer.write(String.format("%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%s,%.2f,%.2f,%.2f,%.2f\n",
                timestamp, imuX, imuY, imuZ, batteryLevel, armPosition, slidePosition,
                gamepad1Buttons, gamepad2Buttons, joystick1X, joystick1Y, joystick2X, joystick2Y));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private String getGamepadButtonStates(Gamepad gamepad) {
        StringBuilder sb = new StringBuilder();
        if (gamepad.a) sb.append("A,");
        if (gamepad.b) sb.append("B,");
        if (gamepad.x) sb.append("X,");
        if (gamepad.y) sb.append("Y,");
        if (gamepad.left_bumper) sb.append("LB,");
        if (gamepad.right_bumper) sb.append("RB,");
        if (gamepad.dpad_up) sb.append("DU,");
        if (gamepad.dpad_down) sb.append("DD,");
        if (gamepad.dpad_left) sb.append("DL,");
        if (gamepad.dpad_right) sb.append("DR,");
        return sb.toString();
    }

    public void fixArmError() {
        while (getArmError() > 10) {
            armControl.update();
        }
    }

    public void fixArmErrorBeta() {
        armError = getArmError();
        armControl.setPosition(getArmTargetPosition() - armError);
        armControl.update();
    }
}
