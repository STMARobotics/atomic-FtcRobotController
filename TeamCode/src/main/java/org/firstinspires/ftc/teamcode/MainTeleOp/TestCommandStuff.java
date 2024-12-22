package org.firstinspires.ftc.teamcode.MainTeleOp;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Commands.testServoMove;
import org.firstinspires.ftc.teamcode.Commands.testSlideUp;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;
import com.arcrobotics.ftclib.command.CommandScheduler;

@TeleOp
public class TestCommandStuff extends LinearOpMode {
    private testServoMove testServoMove;
    private testSlideUp testSlideUp;
    private ArmControl armControl;
    private SlideControl slideControl;
    private MainSubsystem mainSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize commands
        testServoMove = new testServoMove(armControl, slideControl, mainSubsystem);
        testSlideUp = new testSlideUp(armControl, slideControl, mainSubsystem);

        CommandScheduler scheduler = CommandScheduler.getInstance();

        ParallelCommandGroup parallelCommands = new ParallelCommandGroup(
                new testServoMove(testServoMove),
                new testSlideUp(testSlideUp)
        );

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.a) {
                scheduler.schedule(parallelCommands);
            }
        }
    }
}
