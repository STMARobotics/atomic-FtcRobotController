package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

public class testSlideUp extends CommandBase {
    private ArmControl armControl;
    private SlideControl slideControl;
    private MainSubsystem mainSubsystem;

    public testSlideUp(ArmControl armControl, SlideControl slideControl, MainSubsystem mainSubsystem) {
        this.armControl = armControl;
        this.slideControl = slideControl;
        this.mainSubsystem = mainSubsystem;

        addRequirements((Subsystem) armControl, (Subsystem) slideControl, (Subsystem) mainSubsystem);
    }

    public testSlideUp(testSlideUp testSlideUp) {
    }

    @Override
    public void initialize() {
        slideControl.autoSlideMover(-3550);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}