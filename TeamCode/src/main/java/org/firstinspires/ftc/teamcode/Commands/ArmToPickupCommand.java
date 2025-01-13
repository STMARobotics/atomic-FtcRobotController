package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;


public class ArmToPickupCommand extends CommandBase {
    private final ArmControl armControl;

    public ArmToPickupCommand(ArmControl armControl) {
        this.armControl = armControl;

        addRequirements((Subsystem) armControl);
    }

//    @Override
//    public void initialize() {
//
//    }

    @Override
    public void execute() {
        armControl.autoArmMover(4950);
    }

    @Override
    public boolean isFinished() {
        return armControl.getArmError() < 10;
    }

//    @Override
//    public void end(boolean interrupted) {
//    }
}