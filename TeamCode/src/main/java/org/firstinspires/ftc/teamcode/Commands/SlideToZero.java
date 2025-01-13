package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

public class SlideToZero extends CommandBase {
    private final SlideControl slideControl;

    public SlideToZero(SlideControl slideControl) {
        this.slideControl = slideControl;

        addRequirements((Subsystem) slideControl);
    }

//    @Override
//    public void initialize() {
//
//    }

    @Override
    public void execute() {
        slideControl.autoSlideMover(0);
    }

    @Override
    public boolean isFinished() {
        return slideControl.getSlideError() < 1.5;
    }

//    @Override
//    public void end(boolean interrupted) {
//    }
}