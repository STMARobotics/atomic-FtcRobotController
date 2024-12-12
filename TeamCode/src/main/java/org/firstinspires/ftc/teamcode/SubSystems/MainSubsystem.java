package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

    public MainSubsystem(HardwareMap hardwareMap) {
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
        return armControl.getArmTargetPosition() - armControl.getArmPosition();
    }

    public double getSlidePosition() {
        return slideControl.getCurrentPosition();
    }

    public double getSlideTargetPosition() {
        return slideControl.getTargetPosition();
    }

    public double getSlideError() {
        return slideControl.getTargetPosition() - slideControl.getCurrentPosition();
    }

    public double getIntakeServoPower() {
        // implement this later
        return 0;
    }

//    public double getBucketServoTargetPosition() {
//        //not working
//    }

    public void calibrate() {
        armControl.resetZero();
        slideControl.resetEncoder();
        isCalibrated = true;
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
        while (getArmError() > 5) {
            armControl.update();
        }
    }
}
