package org.firstinspires.ftc.teamcode.TuningAndTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestServo", group = "Testing")
public class TestServo extends OpMode {

    private Servo servo;
    private static final double MIN_SERVO_POSITION = -180.0;
    private static final double MAX_SERVO_POSITION = 180.0;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        // Get the joystick position
        double joystickY = -gamepad1.left_stick_y; // Invert Y-axis

        // Map the joystick value from -1 to 1 to -180 to 180
        double targetPosition = (joystickY + 1) / 2.0 * (MAX_SERVO_POSITION - MIN_SERVO_POSITION) + MIN_SERVO_POSITION;

        // Ensure the position is within bounds
        targetPosition = Math.max(MIN_SERVO_POSITION, Math.min(MAX_SERVO_POSITION, targetPosition));

        // Map the target position to the servo range [0, 1]
        double servoPosition = (targetPosition + 180) / 360.0;

        // Set the servo position
        servo.setPosition(servoPosition);

        // Telemetry for debugging
        telemetry.addData("Joystick Y", joystickY);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Servo Position", servoPosition);
        telemetry.update();
    }
}
