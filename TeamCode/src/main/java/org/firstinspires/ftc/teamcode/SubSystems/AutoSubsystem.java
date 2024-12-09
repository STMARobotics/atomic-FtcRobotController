package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

public class AutoSubsystem {
    private ArmControl armControl;
    private SlideControl slideControl;
    double targetSlidePosition = 0;
    private double targetArmPosition = 0;
    double targetServoPosition = 65;
    double currentSlidePosition;
    double currentArmPosition;
    private CRServo intake;



    public AutoSubsystem(HardwareMap hardwareMap, ArmControl armControl, SlideControl slideControl) {
        this.armControl = armControl;
        this.slideControl = slideControl;
        this.intake = hardwareMap.get(CRServo.class, "intake");
    }

    public void dumpSampleHigh(){
        slideControl.setTargetPosition(-3550);
        waitForSlide();
        slideControl.setServoPosition(-80);
        delay(200);
        slideControl.setServoPosition(-10);
        delay(200);
        slideControl.setTargetPosition(-10);
        waitForSlide();
    }

    public void armOut(){
        slideControl.setTargetPosition(-1820);
        waitForSlide();
        slideControl.setServoPosition(-10);
        delay(250);
        armControl.setPosition(3600);
        waitForArm();
        slideControl.setTargetPosition(-10);
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
        slideControl.setTargetPosition(-10);
        waitForSlide();
    }

    public void pickup(){
        armControl.setPosition(4050);
        waitForArm();
    }

    public void specimenPickup(){
        armControl.setPosition(3600);
        waitForArm();
    }

    public void specimenDropoff(){
        armControl.setPosition(2700);
        waitForArm();
    }

    public void loadSampleBucket(){
        slideControl.setTargetPosition(-10);
        waitForSlide();
        armControl.setPosition(1400);
        waitForArm();
        intake.setPower(0.3);
        delay(400);
        intake.setPower(0);
        armControl.setPosition(3600);
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