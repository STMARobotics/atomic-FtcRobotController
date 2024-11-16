package org.firstinspires.ftc.teamcode.TuningAndTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;

@TeleOp(name="Slide Control TeleOp", group="TeleOp")
public class SlideTuning extends LinearOpMode {

    private SlideControl slideControl;  // Renamed subsystem
    private DcMotorEx slideMotor;
    private Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        servo = hardwareMap.get(Servo.class, "servo");

        // Initialize SlideControl subsystem
        slideControl = new SlideControl(slideMotor, servo);

        // Set initial servo position at start of teleop
        slideControl.setServoPosition(65);

        // Reset encoder and set the initial position as zero
        slideControl.resetEncoder();

        waitForStart();

        while (opModeIsActive()) {

            // Control slide motor with left stick Y-axis on gamepad 2
            if (gamepad2.left_stick_y != 0) {
                // Adjust target position based on the Y-axis
                double delta = gamepad2.left_stick_y * 5;  // Increase/decrease by 5 degrees
                slideControl.setTargetPosition(slideControl.getTargetPosition() + delta);
            }

            // Control the servo positions with D-pad
            if (gamepad2.dpad_up) {
                slideControl.setServoPosition(-10);
            } else if (gamepad2.dpad_right) {
                slideControl.setServoPosition(65);
            } else if (gamepad2.dpad_down) {
                slideControl.setServoPosition(-80);
            }

            // Update PID control and telemetry
            slideControl.update(telemetry);

            // Wait for a short time to prevent a busy loop
            sleep(20);
        }
    }
}
