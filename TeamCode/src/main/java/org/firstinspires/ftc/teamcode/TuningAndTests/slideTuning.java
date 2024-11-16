package org.firstinspires.ftc.teamcode.TuningAndTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Slide and Servo Test", group = "Test")
public class slideTuning extends OpMode {
    private SlideControl slideControl;
    private Servo servo;

    private double initialServoPosition = 0;  // Initial position of the servo in degrees (0 = full left, 1 = full right)

    @Override
    public void init() {
        slideControl = new SlideControl(hardwareMap);
        servo = hardwareMap.get(Servo.class, "servo");  // Assuming servo is named "servo" in the configuration
        initialServoPosition = servo.getPosition();  // Record the servo's starting position as 0

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Servo Initial Position", initialServoPosition);
    }

    @Override
    public void loop() {
        // Zero the slide when gamepad2.y is pressed
        if (gamepad2.y) {
            slideControl.resetZeroPosition();
            telemetry.addData("Zero Position", "Slide Reset to Zero");
        }

        // Control the slide degrees using gamepad2.left_stick_y
        double slidePower = -gamepad2.left_stick_y;  // Invert if necessary for correct direction
        if (Math.abs(slidePower) > 0.05) {
            slideControl.setMotorPower(slidePower);  // Directly set motor power for manual control
            telemetry.addData("Slide Manual Control", "Power: %.2f", slidePower);
        } else {
            slideControl.updatePID();  // If no input, continue to use PID for position control
            telemetry.addData("Slide Control", "Maintaining Target Position");
        }

        // Control the servo position using gamepad2.right_stick_y
        double servoPosition = initialServoPosition + gamepad2.right_stick_y * 0.5;  // Scale for servo range (0 to 1)
        servoPosition = Math.min(1, Math.max(0, servoPosition));  // Ensure servo stays within 0 to 1 range
        servo.setPosition(servoPosition);

        telemetry.addData("Slide Position (Degrees)", slideControl.getCurrentPositionDegrees());  // Show current slide position
        telemetry.addData("Servo Target Position", servoPosition);  // Show target servo position
        telemetry.update();
    }

    @Override
    public void stop() {
        slideControl.stopMotor();  // Ensure motor is stopped when op mode ends
    }
}
