package org.firstinspires.ftc.teamcode.Commands.Movement;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.VariableFactory;

public class Drop2nd extends CommandBase {
    private final MainSubsystem mainSubsystem;
    private final VariableFactory variableFactory;
    private boolean isMoving = false;
    private double duration;
    private final ElapsedTime timer;

    public Drop2nd(MainSubsystem mainSubsystem, VariableFactory variableFactory) {
        this.mainSubsystem = mainSubsystem;
        this.variableFactory = variableFactory;
        addRequirements(mainSubsystem);
        this.timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        timer.reset();
        duration = variableFactory.getVariable("moveToDrop2ndDuration");
        isMoving = true;
    }

    @Override
    public void execute() {
        if (isMoving) {
            mainSubsystem.moveDrivetrain(variableFactory.getVariable("moveToDrop2ndPower"), variableFactory.getVariable("moveToDrop2ndDuration"));
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.milliseconds() >= duration) {
            mainSubsystem.stopDrivetrain();
            isMoving = false;
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mainSubsystem.stopDrivetrain();
        isMoving = false;
    }

}