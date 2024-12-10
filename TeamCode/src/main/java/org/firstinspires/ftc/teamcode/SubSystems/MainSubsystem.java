package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

public class MainSubsystem {

    private ArmControl armControl;
    private SlideControl slideControl;

    public double getArmPosition() {
        double armActualPosition = armControl.getArmPosition();
        return armActualPosition;
    }

    public double getArmTargetPosition() {
        double armTargetPosition = armControl.getArmTargetPosition();
        return armTargetPosition;
    }

    public double getArmError() {
        double error = armControl.getArmTargetPosition() - armControl.getArmPosition();
        return error;
    }

    public double getSlidePosition() {
        double slideActualPosition = slideControl.getCurrentPosition();
        return slideActualPosition;
    }

    public double getSlideTargetPosition() {
        double slideTargetPosition = slideControl.getTargetPosition();
        return slideTargetPosition;
    }

    public double getSlideError() {
        double slideError = slideControl.getTargetPosition() - slideControl.getCurrentPosition();
        return slideError;
    }

    public double getIntakeServoPower() {
        return 0; // placeholder return value
    }

    public double getBucketServoTargetPosition() {
        double servoPosition = slideControl.getServoPosition();
        return servoPosition;
    }

    public void fixArmError() {
        while (getArmError() > 5) {
            armControl.update();
        }
    }
}
