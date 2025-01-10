package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

public class testMoveCommand extends CommandBase {
    private final DcMotor frontRight;
    private final DcMotor rearRight;
    private final DcMotor rearLeft;
    private final DcMotor frontLeft;
    private final ElapsedTime timer;

    public testMoveCommand(SlideControl slideControl, ArmControl armControl, MainSubsystem mainSubsystem) {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        frontRight.setPower(0);
        rearRight.setPower(0);
        rearLeft.setPower(0);
        frontLeft.setPower(0);
        timer.reset();  // Start the timer
    }

    @Override
    public void execute() {
        frontRight.setPower(0.5);
        rearRight.setPower(0.5);
        rearLeft.setPower(0.5);
        frontLeft.setPower(0.5);
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= 5.0;  // Return true after 5 seconds
    }

    @Override
    public void end(boolean interrupted) {
        frontRight.setPower(0);
        rearRight.setPower(0);
        rearLeft.setPower(0);
        frontLeft.setPower(0);
    }
}