package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SlideControl;

@TeleOp(name = "Slide Test", group = "Test")
public class TestSlide extends OpMode {
    private SlideControl slideControl;

    @Override
    public void init() {
        slideControl = new SlideControl(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Reset zero position when gamepad2.y is pressed
        if (gamepad2.y) {
            slideControl.resetZeroPosition();
            telemetry.addData("Zero Position", "Reset");
        }

        // Move to predefined positions using bumpers
        if (gamepad2.left_bumper) {
            slideControl.setPickupPosition();
            telemetry.addData("Target Position", "Pickup Position");
        } else if (gamepad2.right_bumper) {
            slideControl.setDropoffPosition();
            telemetry.addData("Target Position", "Dropoff Position");
        }

        // Control slide manually with left stick Y-axis
        double manualControlPower = -gamepad2.left_stick_y;  // Invert if necessary for correct direction
        if (Math.abs(manualControlPower) > 0.05) {
            slideControl.setMotorPower(manualControlPower);
            telemetry.addData("Manual Control", "Power: %.2f", manualControlPower);
        } else {
            // If no manual input, maintain current target position with PID
            slideControl.updatePID();
            telemetry.addData("PID Control", "Maintaining Target Position");
        }

        // Display current slide position in degrees
        telemetry.addData("Current Position (Degrees)", slideControl.getCurrentPositionDegrees());
        telemetry.addData("Target Position (Degrees)", slideControl.getTargetPositionDegrees());
        telemetry.update();
    }

    @Override
    public void stop() {
        slideControl.stopMotor();
    }
}
