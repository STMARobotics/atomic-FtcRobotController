package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

public class testServoMove extends CommandBase {
    private ArmControl armControl;
    private SlideControl slideControl;
    private MainSubsystem mainSubsystem;

    public testServoMove(ArmControl armControl, SlideControl slideControl, MainSubsystem mainSubsystem) {
        this.armControl = armControl;
        this.slideControl = slideControl;
        this.mainSubsystem = mainSubsystem;

        addRequirements((Subsystem) armControl, (Subsystem) slideControl, (Subsystem) mainSubsystem);
    }

    public testServoMove(testServoMove testServoMove) {
    }

    @Override
    public void initialize() {
        slideControl.setServoPosition(-80);

        slideControl.setServoPosition(65);
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}