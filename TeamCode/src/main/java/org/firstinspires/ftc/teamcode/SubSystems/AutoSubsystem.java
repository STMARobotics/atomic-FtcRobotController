package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoSubsystem {
    private ArmControl armControl;
    private SlideControl slideControl;
    double targetSlidePosition = 0;
    private double targetArmPosition = 0;
    double targetServoPosition = 65;
    double currentSlidePosition;
    double currentArmPosition;

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

    public void armOut(){
        slideControl.setTargetPosition(-1820);
        waitForSlide();
        slideControl.setServoPosition(-10);
        delay(250);
        armControl.setPosition(1);
        waitForArm();
        slideControl.setTargetPosition(0);
        waitForSlide();
    }

    public void armIn(){
        slideControl.setTargetPosition(-1820);
        waitForSlide();
        slideControl.setServoPosition(-10);
        delay(250);
        armControl.setPosition(0);
        waitForArm();
        slideControl.setServoPosition(65);
        delay(250);
        slideControl.setTargetPosition(0);
        waitForSlide();
    }

    public void waitForArm(){
        currentArmPosition = armControl.getArmPosition();
        targetArmPosition = armControl.getArmTargetPosition();
        while (currentArmPosition - targetArmPosition > 2){
            slideControl.update();
        }

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