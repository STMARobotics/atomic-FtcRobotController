package org.firstinspires.ftc.teamcode.TuningAndTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class FixSlideNotZeroing extends LinearOpMode {
    private double ampDraw;

    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            ampDraw = slideMotor.getCurrent(CurrentUnit.AMPS);

            while (ampDraw < 5) {
                ampDraw = slideMotor.getCurrent(CurrentUnit.AMPS);
                slideMotor.setPower(1);
                telemetry.addData("amp draw", ampDraw);
                telemetry.update();
            }

            if (ampDraw > 5) {
                slideMotor.setPower(0);
                break;
            }
        }
    }
}