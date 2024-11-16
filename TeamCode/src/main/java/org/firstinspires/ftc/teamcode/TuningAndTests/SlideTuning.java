package org.firstinspires.ftc.teamcode.TuningAndTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.SlideControl;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Slide and Servo Test", group = "Test")
public class SlideTuning extends OpMode {
    private SlideControl slideControl;
    private Servo servo;
    double servoInput = 0; // Initial position of the servo

    private double initialServoPosition = 0;  // Initial position of the servo in degrees (0 = start of rotation)
    private boolean isZeroed = false;  // Flag to track if the zero position has been set

    @Override
    public void init() {
        slideControl = new SlideControl(hardwareMap);
        servo = hardwareMap.get(Servo.class, "servo");  // Assuming servo is named "servo" in the configuration
        initialServoPosition = 0;  // Reset the initial position of the servo to 0 degrees

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Servo Initial Position", initialServoPosition);
    }

    @Override
    public void loop() {
        // Zero the slide when gamepad2.y is pressed
        if (gamepad2.y) {
            slideControl.resetZeroPosition();  // Reset slide motor to its current position
            initialServoPosition = 0;  // Reset the initial servo position to 0 degrees
            servo.setPosition(0);  // Set the servo position to 0 degrees (start of rotation)
            isZeroed = true;  // Mark the system as zeroed
            telemetry.addData("Zero Position", "Slide and Servo Reset to Zero");
        }

        // Control the slide degrees using gamepad2.left_stick_y
        double slidePower = -gamepad2.left_stick_y;  // Invert if necessary for correct direction

        // Only control slide if joystick movement exceeds the threshold (0.1 or -0.1)
        if (Math.abs(slidePower) > 0.1) {
            slideControl.setMotorPower(slidePower);  // Directly set motor power for manual control
            telemetry.addData("Slide Manual Control", "Power: %.2f", slidePower);
        } else {
            slideControl.updatePID();  // If no input, continue to use PID for position control
            telemetry.addData("Slide Control", "Maintaining Target Position");
        }


        if (gamepad2.right_stick_y > 0.1) {
            servoInput += 1;
        } else if (gamepad2.right_stick_y < -0.1) {
            servoInput -= 1;
        } else if (gamepad2.dpad_up){
            servoInput = -10;
        } else if (gamepad2.dpad_down){
            servoInput = -80;
        } else if (gamepad2.dpad_right){
            servoInput = 65;
        }

// Hard stop at -90 and 90 degrees
        servoInput = Math.max(-90, Math.min(90, servoInput));

// Convert to normalized servo position (0-1 range for the servo)
        double normalizedServoPosition = (servoInput + 90) / 180;

        servo.setPosition(normalizedServoPosition);


        telemetry.addData("Slide Position (Degrees)", slideControl.getCurrentPositionDegrees());  // Show current slide position
        telemetry.addData("Servo Target Position (Degrees)", servoInput);  // Show target servo position (0-300 degrees)
        telemetry.update();
    }

    @Override
    public void stop() {
        slideControl.stopMotor();  // Ensure motor is stopped when op mode ends
    }
}
