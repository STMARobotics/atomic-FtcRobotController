package org.firstinspires.ftc.teamcode.Autos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Commands.ArmToPickupCommand;
import org.firstinspires.ftc.teamcode.Commands.Movement.Drop2nd;
import org.firstinspires.ftc.teamcode.Commands.Movement.Drop3rd;
import org.firstinspires.ftc.teamcode.Commands.Movement.Grab2nd;
import org.firstinspires.ftc.teamcode.Commands.Movement.Grab3rd;
import org.firstinspires.ftc.teamcode.Commands.Movement.RotateZero;
import org.firstinspires.ftc.teamcode.Commands.SlideToZero;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.MainSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;
import org.firstinspires.ftc.teamcode.SubSystems.VariableFactory;

@Autonomous
public class ExperimentalAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

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

        // Start auto
        new InstantCommand(() -> {
            mainSubsystem.moveDrivetrain(variableFactory.getVariable("moveTo1stBasketPower"), variableFactory.getVariable("moveTo1stBasketDuration1"));

            mainSubsystem.rotateToAngle(variableFactory.getVariable("rotateToDrop1stAngle"));

            mainSubsystem.moveDrivetrain(variableFactory.getVariable("moveTo1stBasketPower2"), variableFactory.getVariable("moveTo1stBasketDuration2"));
            mainSubsystem.stopDrivetrain();

            slideControl.autoSlideMover(-3550);
            slideControl.setServoPosition(-80);
        }, mainSubsystem).andThen(
                new WaitCommand(850)
        ).andThen(new InstantCommand(() -> {
                slideControl.setServoPosition(-10);
                armControl.autoArmMover(4950);
                mainSubsystem.rotateToAngle(variableFactory.getVariable("rotateToPickup2ndAngle"));

                intake.setPower(1);
        }, mainSubsystem)).andThen(
                new ParallelCommandGroup(
                        new SlideToZero(slideControl),
                        new Grab2nd(mainSubsystem, variableFactory)
                )
        ).andThen(new InstantCommand(() -> {
            intake.setPower(0);

            armControl.autoArmMover(1540);
            intake.setPower(-1);
        }, mainSubsystem)).andThen(
                new WaitCommand(380)
        ).andThen(new InstantCommand(() -> {
            intake.setPower(0);
            slideControl.setServoPosition(0);
        }, mainSubsystem)).andThen(
                new ParallelCommandGroup(
                        new ArmToPickupCommand(armControl),
                        new Drop2nd(mainSubsystem, variableFactory))
        ).andThen(new InstantCommand(() -> {
            slideControl.autoSlideMover(-3550);
            slideControl.setServoPosition(-80);
        }, mainSubsystem)).andThen(
                new WaitCommand(850)
        ).andThen(new InstantCommand(() -> {
            slideControl.setServoPosition(0);
            mainSubsystem.rotateToAngle(variableFactory.getVariable("rotateToPickup3rdAngle"));
            intake.setPower(1);
        }, mainSubsystem)).andThen(
                new ParallelCommandGroup(
                        new SlideToZero(slideControl),
                        new Grab3rd(mainSubsystem, variableFactory))
        ).andThen(new InstantCommand(() -> {
            armControl.autoArmMover(1540);
            intake.setPower(-1);
        }, mainSubsystem)).andThen(
                new WaitCommand(380)
        ).andThen(new InstantCommand(() -> {
            intake.setPower(0);
        }, mainSubsystem)).andThen(
                new ParallelCommandGroup(
                        new ArmToPickupCommand(armControl),
                        new Drop3rd(mainSubsystem, variableFactory))
        ).andThen(new InstantCommand(() -> {
            slideControl.autoSlideMover(-3550);
            slideControl.setServoPosition(-80);
        }, mainSubsystem)).andThen(
                new WaitCommand(850)
        ).andThen(new InstantCommand(() -> {
            slideControl.setServoPosition(0);
        }, mainSubsystem)).andThen(
                new ParallelCommandGroup(
                        new SlideToZero(slideControl),
                        new RotateZero(mainSubsystem))
        ).andThen(new RunCommand(() -> {
            telemetry.addData("Autonomous", "Complete");
        }));

        while (opModeIsActive()) {
            double batteryVoltage = batteryVoltageSensor.getVoltage();
            variableFactory.updateBatteryVoltage(batteryVoltage);
            CommandScheduler.getInstance().run();
            telemetry.update();
        }

    }
}
