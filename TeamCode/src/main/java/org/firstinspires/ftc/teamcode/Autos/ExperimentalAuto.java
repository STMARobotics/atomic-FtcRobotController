package org.firstinspires.ftc.teamcode.Autos;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;
import org.firstinspires.ftc.teamcode.SubSystems.ServoSubsystemForAuto;
import org.firstinspires.ftc.teamcode.SubSystems.VariableFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.ArmToPickupCommand;
import org.firstinspires.ftc.teamcode.Commands.DropHighCommand;
import org.firstinspires.ftc.teamcode.Commands.SlideToZero;
import org.firstinspires.ftc.teamcode.Commands.Movement.Drop2nd;
import org.firstinspires.ftc.teamcode.Commands.Movement.Drop3rd;
import org.firstinspires.ftc.teamcode.Commands.Movement.Grab2nd;
import org.firstinspires.ftc.teamcode.Commands.Movement.Grab3rd;

@Autonomous
public class ExperimentalAuto extends LinearOpMode {

    private ArmControl armControl;
    private SlideControl slideControl;
    private MainSubsystem mainSubsystem;
    private ServoSubsystemForAuto servoSubsystem;
    private VariableFactory variableFactory;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        Servo slideServo = hardwareMap.get(Servo.class, "servo");

        armControl = new ArmControl(hardwareMap);
        slideControl = new SlideControl(slideMotor, slideServo);
        mainSubsystem = new MainSubsystem(hardwareMap);
        variableFactory = new VariableFactory();

        slideControl.resetEncoder();
        armControl.resetZero();

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        mainSubsystem.moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, variableFactory.getVariable("moveTo1stBasketPower"), variableFactory.getVariable("moveTo1stBasketDuration1"));

        mainSubsystem.rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, variableFactory.getVariable("rotateToDrop1stAngle"));

        mainSubsystem.moveDrivetrain(frontLeft, rearLeft, frontRight, rearRight, variableFactory.getVariable("moveTo1stBasketPower2"), variableFactory.getVariable("moveTo1stBasketDuration2"));
        mainSubsystem.stopDrivetrain(frontLeft, rearLeft, rearRight, frontRight);

        slideControl.autoSlideMover(-3550);
        slideControl.setServoPosition(-80);
        sleep(850);
        slideControl.setServoPosition(-10);
        armControl.autoArmMover(4950);
        mainSubsystem.rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, variableFactory.getVariable("rotateToPickup2ndAngle"));

        new ParallelCommandGroup(
                new SlideToZero(slideControl),
                new Grab2nd(mainSubsystem)
        ).schedule();




        new SequentialCommandGroup(
                new Drop2nd(mainSubsystem)).schedule();

        mainSubsystem.rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, 0);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
