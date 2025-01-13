package org.firstinspires.ftc.teamcode.TuningAndTests;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

public class testSlideUp extends CommandBase {
    private final SlideControl slideControl;
    private final MainSubsystem mainSubsystem;

    public testSlideUp(SlideControl slideControl, MainSubsystem mainSubsystem) {
        this.slideControl = slideControl;
        this.mainSubsystem = mainSubsystem;

        addRequirements((Subsystem) slideControl, (Subsystem) mainSubsystem);
    }

//    @Override
//    public void initialize() {
//
//    }

    @Override
    public void execute() {
        slideControl.autoSlideMover(-3550);
    }

    @Override
    public boolean isFinished() {
        return mainSubsystem.getSlideError() < 10;
    }

//    @Override
//    public void end(boolean interrupted) {
//    }
}