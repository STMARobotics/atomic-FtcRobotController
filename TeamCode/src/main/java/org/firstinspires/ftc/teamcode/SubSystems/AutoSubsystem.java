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
    final CRServo intake = hardwareMap.get(CRServo.class, "intake");

    public void dumpSampleHigh(){
        slideControl.setTargetPosition(-3550);
        waitForSlide();
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
        armControl.setPosition("placeholder");
        waitForArm();
        slideControl.setTargetPosition(0);
        waitForSlide();
    }

    public void armIn(){
        slideControl.setTargetPosition(-1820);
        waitForSlide();
        slideControl.setServoPosition(-10);
        delay(250);
        armControl.setPosition("placeholder");
        waitForArm();
        slideControl.setServoPosition(65);
        delay(250);
        slideControl.setTargetPosition(0);
        waitForSlide();
    }

    pubic void pickup(){
        armControl.setPosition("placeholder");
        waitForArm();
    }

    public void specimenPickup(){
        armControl.setPosition("placeholder");
        waitForArm();
    }

    public void specimenDropoff(){
        armControl.setPosition("placeholder");
        waitForArm();
    }

    public void loadSampleBucket(){
        slideControl.setTargetPosition(-1820);
        waitForSlide();
        armControl.setPosition("placeholder");
        waitForArm();
        intake.setPower(0.3);
        delay(400);
        intake.setPower(0);
        armControl.setPosition("placeholder");
        waitForArm();        
    }

    public void servoDropSample() {
        slideControl.setServoPosition(-80);
        delay(200);
        slideControl.setServoPosition(-10);
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
