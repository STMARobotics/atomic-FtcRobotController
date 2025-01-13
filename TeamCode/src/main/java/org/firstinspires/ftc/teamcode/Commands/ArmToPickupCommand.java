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

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        armControl.autoArmMover(1000);
    }//this is a placeholder we need to actually find the pickup position for auto because its different then teleop

    @Override
    public boolean isFinished() {
        if (armControl.getArmError() < 10){
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }
}