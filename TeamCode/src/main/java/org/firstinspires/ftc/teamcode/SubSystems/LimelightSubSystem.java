package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightSubSystem {
    private Limelight3A limelight;
    private DcMotor frontRight, rearRight, rearLeft, frontLeft;
    private double targetPositionX, targetPositionY;
    private double actualPositionX, actualPositionY;

    private double kP = 0.0075, kI = 0, kD = 0.0;
    private double headingKP = 0.01;
    private double previousHeadingError = 0;
    private double integralHeading = 0;

    private double previousErrorX = 0, previousErrorY = 0;
    private double integralX = 0, integralY = 0;
    private final double integralLimit = 0.1;
    private IMU imu;

    public LimelightSubSystem(Limelight3A limelight, IMU imu, DcMotor frontRight, DcMotor rearRight, DcMotor rearLeft, DcMotor frontLeft) {
        this.limelight = limelight;
        this.imu = imu;
        this.frontRight = frontRight;
        this.rearRight = rearRight;
        this.rearLeft = rearLeft;
        this.frontLeft = frontLeft;

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void updateActualPosition() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                actualPositionX = botpose.getPosition().x;
                actualPositionY = botpose.getPosition().y;
            }
        } else {
            actualPositionX = 0;
            actualPositionY = 0;
        }
    }

    public double getActualPositionX() {
        return actualPositionX;
    }

    public double getActualPositionY() {
        return actualPositionY;
    }

    public double getErrorX() {
        return targetPositionX - actualPositionX;
    }

    public double getErrorY() {
        return targetPositionY - actualPositionY;
    }

    public boolean isAtPosition() {
        if (getErrorX() < 0.75 && getErrorY() < 0.5){
            return true;
        }else {
            return false;
        }
    }

    public void goToPosition(double targetX, double targetY, double targetHeading) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double actualX = botpose.getPosition().x;
                double actualY = botpose.getPosition().y;
                double actualHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

                double errorX = targetX - actualX;
                double errorY = targetY - actualY;
                double headingError = targetHeading - actualHeading;
                headingError = AngleUnit.normalizeDegrees(headingError);

                integralX += errorX;
                integralY += errorY;
                double powerX = kP * errorX + kI * integralX + kD * (errorX - previousErrorX);
                double powerY = kP * errorY + kI * integralY + kD * (errorY - previousErrorY);

                integralHeading += headingError;
                double turnPower = headingKP * headingError + kI * integralHeading + kD * (headingError - previousHeadingError);

                double fieldHeading = Math.toRadians(actualHeading);
                double tempX = powerX * Math.cos(fieldHeading) - powerY * Math.sin(fieldHeading);
                double tempY = powerX * Math.sin(fieldHeading) + powerY * Math.cos(fieldHeading);

                double frontRightPower = tempY - tempX - turnPower;
                double rearRightPower = tempY + tempX - turnPower;
                double rearLeftPower = tempY - tempX + turnPower;
                double frontLeftPower = tempY + tempX + turnPower;

                frontRight.setPower(frontRightPower);
                rearRight.setPower(rearRightPower);
                rearLeft.setPower(rearLeftPower);
                frontLeft.setPower(frontLeftPower);

                previousErrorX = errorX;
                previousErrorY = errorY;
                previousHeadingError = headingError;
            }
        }
    }

    public void stopMotors() {
        frontRight.setPower(0);
        rearRight.setPower(0);
        rearLeft.setPower(0);
        frontLeft.setPower(0);
    }
}
