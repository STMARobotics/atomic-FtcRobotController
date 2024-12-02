package org.firstinspires.ftc.teamcode.TuningAndTests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.SubSystems.LimelightSubSystem;

@TeleOp(name = "Automatic LimeLight PID Tuning", group = "Tuning")
public class LLPidAutoTuner extends LinearOpMode {
    private LimelightSubSystem limelightSubSystem;
    private DcMotorEx frontRight, rearRight, rearLeft, frontLeft;

    private double positionAX = 5, positionAY = 5;
    private double positionBX = 0, positionBY = 0;
    private double kP = 0.0075, kI = 0, kD = 0.0;
    private boolean atTarget = false;

    private double targetHeading = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");

        limelightSubSystem = new LimelightSubSystem(
                hardwareMap.get(Limelight3A.class, "limelight"),
                frontRight, rearRight, rearLeft, frontLeft
        );

        telemetry.addLine("Ready for Automatic Movement PID Tuning");
        telemetry.update();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        waitForStart();

        boolean moveToB = false;
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double targetX = moveToB ? positionBX : positionAX;
                double targetY = moveToB ? positionBY : positionAY;


                limelightSubSystem.goToPosition(targetX, targetY, targetHeading);
                double errorX = limelightSubSystem.getErrorX();
                double errorY = limelightSubSystem.getErrorY();

                //check if at the correct pos
                if (Math.abs(errorX) < 0.05 && Math.abs(errorY) < 0.05) {
                    atTarget = true;
                }

                if (atTarget) {
                    moveToB = !moveToB;
                    atTarget = false;
                }


                telemetry.addData("Target", "X: %.2f Y: %.2f", targetX, targetY);
                telemetry.addData("Position", "X: %.2f Y: %.2f", limelightSubSystem.getActualPositionX(), limelightSubSystem.getActualPositionY());
                telemetry.addData("PID", "kP: %.4f kI: %.4f kD: %.4f", kP, kI, kD);
                telemetry.update();
            }
        }

        telemetry.addLine("PID Tuning Complete");
        telemetry.update();
    }
}
