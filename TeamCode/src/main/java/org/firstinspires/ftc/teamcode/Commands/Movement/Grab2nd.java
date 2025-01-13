package org.firstinspires.ftc.teamcode.Commands.Movement;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.VariableFactory;
import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;

public class Grab2nd extends CommandBase {
    private MainSubsystem mainSubsystem;
    private VariableFactory variableFactory;
    private HardwareMap hardwareMap;
    private DcMotor frontRight;
    private DcMotor rearRight;
    private DcMotor rearLeft;
    private DcMotor frontLeft;
    private boolean isMoving = false;
    private double duration;
    private final ElapsedTime timer;

    public Grab2nd(MainSubsystem mainSubsystem) {
        this.mainSubsystem = mainSubsystem;
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        addRequirements((Subsystem) mainSubsystem);
        this.timer = new ElapsedTime();

    }

    @Override
    public void initialize() {
        timer.reset();
        duration = variableFactory.getVariable("moveToPickup2ndDuration");
        isMoving = true;
    }

    @Override
    public void execute() {
        if (isMoving) {
            mainSubsystem.moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, variableFactory.getVariable("moveToPickup2ndPower"), variableFactory.getVariable("moveToPickup2ndDuration"));
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.milliseconds() >= duration) {
            mainSubsystem.stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);
            isMoving = false;
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mainSubsystem.stopDrivetrain(frontLeft, rearLeft, frontRight, rearRight);
        isMoving = false;
    }
}