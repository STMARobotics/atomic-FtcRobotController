package org.firstinspires.ftc.teamcode.Commands.Movement;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.VariableFactory;
import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;

public class Drop3rd extends CommandBase {
    private final MainSubsystem mainSubsystem;
    private final VariableFactory variableFactory;
    private HardwareMap hardwareMap;
    private DcMotor frontRight;
    private DcMotor rearRight;
    private DcMotor rearLeft;
    private DcMotor frontLeft;
    private boolean isMoving = false;
    private double duration;
    private final ElapsedTime timer;

    public Drop3rd(MainSubsystem mainSubsystem, VariableFactory variableFactory) {
        this.mainSubsystem = mainSubsystem;
        this.variableFactory = variableFactory;
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
        duration = variableFactory.getVariable("moveToDrop3rdDuration");
        isMoving = true;
    }

    @Override
    public void execute() {
        if (isMoving) {
            mainSubsystem.moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, variableFactory.getVariable("moveToDrop3rdPower"), variableFactory.getVariable("moveToDrop3rdDuration"));
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

    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void setFrontRight(DcMotor frontRight) {
        this.frontRight = frontRight;
    }

    public void setRearRight(DcMotor rearRight) {
        this.rearRight = rearRight;
    }

    public void setRearLeft(DcMotor rearLeft) {
        this.rearLeft = rearLeft;
    }

    public void setFrontLeft(DcMotor frontLeft) {
        this.frontLeft = frontLeft;
    }
}