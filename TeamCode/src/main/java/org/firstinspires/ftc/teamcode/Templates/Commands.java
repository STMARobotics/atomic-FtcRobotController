//package org.firstinspires.ftc.teamcode.Commands.Movement;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.arcrobotics.ftclib.command.Subsystem;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.SubSystems.SubsystemName;
//
//public class CommandName extends CommandBase {
//    private SubsystemName subsytemName;
//    private VariableFactory variableFactory;
//    private HardwareMap hardwareMap;
//    private DcMotor frontRight;
//    private DcMotor rearRight;
//    private DcMotor rearLeft;
//    private DcMotor frontLeft;
//    etc...
//
//    public CommandName(MainSubsystem mainSubsystem) {
//        this.subsystemName = subsystemName;
//        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
//        DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
//        DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
//        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        addRequirements((Subsystem) subsystemName);
//    }
//
//    @Override
//    public void initialize() {
//      do initialization stuff
//    }
//
//    @Override
//    public void execute() {
//      do your main loop stuff here
//    }
//
//    @Override
//    public boolean isFinished() {
//        if (whatever) {
//            return true;
//        } else {
//        return false;
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//      do cleanup stuff
//    }
//}