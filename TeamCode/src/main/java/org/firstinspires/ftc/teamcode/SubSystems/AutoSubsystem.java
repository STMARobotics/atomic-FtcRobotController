package org.firstinspires.ftc.teamcode.SubSystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

public class AutoSubsystem {
    private ArmControl armControl;
    private SlideControl slideControl;
    double targetSlidePosition = 0;
    private double targetArmPosition = 0;
    double targetServoPosition = 65;
    double currentSlidePosition;

    public void dumpSampleHigh(){
        slideControl.setTargetPosition(-3550);
        waitForSlide();
        delay(1000);
        slideControl.setServoPosition(-80);
        delay(200);
        slideControl.setServoPosition(-10);
        delay(200);
        slideControl.setTargetPosition(0);
        waitForSlide();
    }

    public void waitForSlide(){
        currentSlidePosition = slideControl.getCurrentPosition();
        targetSlidePosition = slideControl.getTargetPosition();
        while (currentSlidePosition - targetSlidePosition > 2){
            slideControl.update();
        }

    }

    private ElapsedTime timer = new ElapsedTime();

    private void delay(long milliseconds) {
        timer.reset();
        while (timer.milliseconds() < milliseconds) {
        }
    }


}