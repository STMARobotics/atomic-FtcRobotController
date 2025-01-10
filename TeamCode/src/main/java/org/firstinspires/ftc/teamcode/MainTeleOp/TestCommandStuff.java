package org.firstinspires.ftc.teamcode.MainTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Commands.testSlideUp;
import org.firstinspires.ftc.teamcode.Commands.testMoveCommand;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;
import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;

@TeleOp(name = "My TeleOp")
public class TestCommandStuff extends CommandOpMode {
    private GamepadEx driverOp;
    private testSlideUp command1;
    private testMoveCommand command2;

    // Declare subsystems
    private ArmControl armControl;
    private SlideControl slideControl;
    private MainSubsystem mainSubsystem;

    @Override
    public void initialize() {
        // Initialize gamepad
        driverOp = new GamepadEx(gamepad1);

        // Initialize subsystems
        armControl = new ArmControl(hardwareMap);
        slideControl = new SlideControl(hardwareMap);
        mainSubsystem = new MainSubsystem(hardwareMap);

        // Initialize commands with subsystems
        command1 = new testSlideUp(armControl, slideControl, mainSubsystem);
        command2 = new testMoveCommand(slideControl, armControl, mainSubsystem);

        // Create button binding
        new GamepadButton(driverOp, GamepadKeys.Button.A)
                .whenPressed(
                        new ParallelCommandGroup(
                                command1,
                                command2
                        )
                );
    }
}