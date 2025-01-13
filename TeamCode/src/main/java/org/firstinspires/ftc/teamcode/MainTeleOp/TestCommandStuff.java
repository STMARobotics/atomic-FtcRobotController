package org.firstinspires.ftc.teamcode.MainTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.TuningAndTests.testSlideUp;
import org.firstinspires.ftc.teamcode.TuningAndTests.testMoveCommand;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;
import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;

@TeleOp(name = "TestCommandStuff")
public class TestCommandStuff extends CommandOpMode {
    private GamepadEx driverOp;
    private testSlideUp command1;
    private testMoveCommand command2;

    private ArmControl armControl;
    private SlideControl slideControl;
    private MainSubsystem mainSubsystem;

    @Override
    public void initialize() {
        driverOp = new GamepadEx(gamepad1);

        armControl = new ArmControl(hardwareMap);
        slideControl = new SlideControl(hardwareMap);
        mainSubsystem = new MainSubsystem(hardwareMap);

        command1 = new testSlideUp(slideControl, mainSubsystem);
        command2 = new testMoveCommand();

        new GamepadButton(driverOp, GamepadKeys.Button.A)
                .whenPressed(
                        new ParallelCommandGroup(
                                command1,
                                command2
                        )
                );
    }
}
