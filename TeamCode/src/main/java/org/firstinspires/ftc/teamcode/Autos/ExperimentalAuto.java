package org.firstinspires.ftc.teamcode.Autos;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Commands.Movement.RotateZero;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;
import org.firstinspires.ftc.teamcode.SubSystems.VariableFactory;
import org.firstinspires.ftc.teamcode.Commands.ArmToPickupCommand;
import org.firstinspires.ftc.teamcode.Commands.SlideToZero;
import org.firstinspires.ftc.teamcode.Commands.Movement.Drop2nd;
import org.firstinspires.ftc.teamcode.Commands.Movement.Drop3rd;
import org.firstinspires.ftc.teamcode.Commands.Movement.Grab2nd;
import org.firstinspires.ftc.teamcode.Commands.Movement.Grab3rd;

@Autonomous
public class ExperimentalAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        Servo slideServo = hardwareMap.get(Servo.class, "servo");

        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        ArmControl armControl = new ArmControl(hardwareMap);
        SlideControl slideControl = new SlideControl(slideMotor, slideServo);
        MainSubsystem mainSubsystem = new MainSubsystem(hardwareMap);
        VariableFactory variableFactory = new VariableFactory();

        slideControl.resetEncoder();
        armControl.resetZero();

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        double batteryVoltage = batteryVoltageSensor.getVoltage();

        variableFactory.updateBatteryVoltage(batteryVoltage);

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

        intake.setPower(1);

        new ParallelCommandGroup(
                new SlideToZero(slideControl),
                new Grab2nd(mainSubsystem, variableFactory)
        ).schedule();

        intake.setPower(0);

        armControl.autoArmMover(1540);
        intake.setPower(-1);
        sleep(300);
        intake.setPower(0);
        slideControl.setServoPosition(0);

        new ParallelCommandGroup (
                new ArmToPickupCommand(armControl),
                new Drop2nd(mainSubsystem, variableFactory)).schedule();

        slideControl.autoSlideMover(-3550);
        slideControl.setServoPosition(-80);
        sleep(850);
        slideControl.setServoPosition(0);
        mainSubsystem.rotateToAngle(frontLeft, rearLeft, frontRight, rearRight, variableFactory.getVariable("rotateToPickup3rdAngle"));
        intake.setPower(1);

        new ParallelCommandGroup(
                new SlideToZero(slideControl),
                new Grab3rd(mainSubsystem, variableFactory)).schedule();

        armControl.autoArmMover(1540);
        intake.setPower(-1);
        sleep(300);
        intake.setPower(0);

        new ParallelCommandGroup(
                new ArmToPickupCommand(armControl),
                new Drop3rd(mainSubsystem, variableFactory)).schedule();

        slideControl.autoSlideMover(-3550);
        slideControl.setServoPosition(-80);
        sleep(850);
        slideControl.setServoPosition(0);

        new ParallelCommandGroup(
                new SlideToZero(slideControl),
                new RotateZero(mainSubsystem)).schedule();

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
