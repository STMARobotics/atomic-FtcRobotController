package org.firstinspires.ftc.teamcode.MainTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.VariableFactory;

@TeleOp
public class TestVariableFactory extends LinearOpMode {
    VariableFactory variableFactory;

    @Override
    public void runOpMode() {


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {



            String emptyVariable = " ";
            telemetry.addData("check variable factory old", variableFactory.getVariable("moveTo1stBasketDuration1"));
            telemetry.update();
        }
    }
}
