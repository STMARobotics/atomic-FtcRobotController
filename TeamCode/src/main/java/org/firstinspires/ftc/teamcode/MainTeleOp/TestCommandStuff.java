package org.firstinspires.ftc.teamcode.MainTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.TuningAndTests.testSlideUp;
import org.firstinspires.ftc.teamcode.TuningAndTests.testMoveCommand;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;
import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;

@TeleOp(name = "TestCommandStuff")
public class TestCommandStuff extends CommandOpMode {

    @Override
    public void initialize() {
        GamepadEx driverOp = new GamepadEx(gamepad1);

        SlideControl slideControl = new SlideControl();
        MainSubsystem mainSubsystem = new MainSubsystem(hardwareMap);

        testSlideUp command1 = new testSlideUp(slideControl, mainSubsystem);
        testMoveCommand command2 = new testMoveCommand();

        new GamepadButton(driverOp, GamepadKeys.Button.A)
                .whenPressed(
                        new ParallelCommandGroup(
                                command1,
                                command2
                        )
                );
    }
}
