package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SlideControl extends SubsystemBase {

    private DcMotorEx slide;
    private Servo servo;
    private double targetPosition;
    private double currentPosition;
    private final ElapsedTime runtime = new ElapsedTime();

    private double previousError = 0, integral = 0;

    public SlideControl(DcMotorEx slide, Servo servo) {
        this.slide = slide;
        this.servo = servo;
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.targetPosition = 0;
        this.currentPosition = 0;
    }

    public SlideControl() {
    }

    public void resetEncoder() {
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        currentPosition = slide.getCurrentPosition();
        targetPosition = currentPosition;
    }

    public void updatePIDControl() {
        double error = targetPosition - currentPosition;
        integral += error * runtime.seconds();
        double derivative = (error - previousError) / runtime.seconds();

        double kP = 0.01;
        double kI = 0;
        double kD = 0.0;
        double power = (kP * error) + (kI * integral) + (kD * derivative);

        slide.setPower(power);
        previousError = error;
    }

    public void setTargetPosition(double target) {
        targetPosition = target;
    }

    public double getCurrentPosition() {
        return slide.getCurrentPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setSlidePower(double power) {
        slide.setPower(power);
    }

    public double getSlideAmps() {
        return slide.getCurrent(CurrentUnit.AMPS);
    }

    public void setServoPosition(double position) {
        position = Math.max(-90, Math.min(90, position));

        double mappedPosition = (position + 90) / 180.0;

        servo.setPosition(mappedPosition);
    }

    public void update() {
        currentPosition = slide.getCurrentPosition();
        updatePIDControl();
    }

    public void autoSlideMover(double autoTargetPosition){
        setTargetPosition(autoTargetPosition);
        while (Math.abs(autoTargetPosition - getCurrentPosition()) > 5){
            update();
        }
        slide.setPower(0);
    }

    public double getSlideError() {
        return Math.abs(getTargetPosition()) - Math.abs(getCurrentPosition());
    }


//    public void setPower(double setPower){
//        slide.setPower(setPower);
//    }
}
