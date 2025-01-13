package org.firstinspires.ftc.teamcode.Commands.Movement;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;

public class RotateZero extends CommandBase {
    private final MainSubsystem mainSubsystem;
    private boolean isMoving = false;
    private double duration;
    private final ElapsedTime timer;

    public RotateZero(MainSubsystem mainSubsystem) {
        this.mainSubsystem = mainSubsystem;
        addRequirements(mainSubsystem);
        this.timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        timer.reset();
        duration = 1500;
        isMoving = true;
    }

    @Override
    public void execute() {
        if (isMoving) {
            mainSubsystem.rotateToAngle(0);
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