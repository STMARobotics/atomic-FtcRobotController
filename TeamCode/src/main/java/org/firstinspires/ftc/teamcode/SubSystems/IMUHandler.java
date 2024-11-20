package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUHandler {
    private final IMU imu;
    private double fieldOffset = 0;

    public IMUHandler(HardwareMap hardwareMap, String imuName) {
        imu = hardwareMap.get(IMU.class, imuName);
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }

    public void resetFieldOffset() {
        fieldOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getFieldAdjustedHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - fieldOffset;
    }

    public double[] calculateFieldCentric(double x, double y) {
        double botHeading = Math.toRadians(getFieldAdjustedHeading());
        double tempX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double tempY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        return new double[]{tempX, tempY};
    }
}
