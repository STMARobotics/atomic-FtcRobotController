package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

public class testSlideUp extends CommandBase {
    private final SlideControl slideControl;
    private final MainSubsystem mainSubsystem;

    public testSlideUp(ArmControl armControl, SlideControl slideControl, MainSubsystem mainSubsystem) {
        this.slideControl = slideControl;
        this.mainSubsystem = mainSubsystem;

        addRequirements((Subsystem) slideControl, (Subsystem) mainSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        slideControl.autoSlideMover(-3550);
    }

    @Override
    public boolean isFinished() {
        if (mainSubsystem.getSlideError() < 10){
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }
}