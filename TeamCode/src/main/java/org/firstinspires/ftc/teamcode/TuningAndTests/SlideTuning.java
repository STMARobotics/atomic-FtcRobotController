package org.firstinspires.ftc.teamcode.TuningAndTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Slide Control TeleOp", group="TeleOp")
public class SlideTuning extends LinearOpMode {

    private SlideControl slideControl;  // Renamed subsystem
    private DcMotorEx slide;
    private Servo servo;
    double targetPosition;
    @Override
    public void runOpMode() throws InterruptedException {

        slide = hardwareMap.get(DcMotorEx.class, "slide");
        servo = hardwareMap.get(Servo.class, "servo");

        slideControl = new SlideControl(slide, servo);

        slideControl.setServoPosition(65);
        slideControl.resetEncoder();

        waitForStart();

        while (opModeIsActive()) {

        if (gamepad2.left_stick_y > 0.1) {
                targetPosition += 10;
            } else if (gamepad2.left_stick_y < -0.1) {
                targetPosition -= 10;
        }

        slideControl.setTargetPosition(targetPosition);
            // Control the servo positions with D-pad
            if (gamepad2.dpad_up) {
                slideControl.setServoPosition(-10);  // Set to -10 degrees
            } else if (gamepad2.dpad_right) {
                slideControl.setServoPosition(65);  // Set to 65 degrees
            } else if (gamepad2.dpad_down) {
                slideControl.setServoPosition(-80);  // Set to -80 degrees
            }

            slideControl.update();
            slideControl.updateTelemetry(telemetry);

            sleep(20);
        }
    }
}
