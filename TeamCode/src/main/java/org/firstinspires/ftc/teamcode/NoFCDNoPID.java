package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class NoFCDNoPID extends LinearOpMode {
    private boolean halfSpeed = false;
    private boolean lastButtonState = false;

    // Motor power variables for gradual adjustment
    private double armPower = 0.0;
    private double powerIncrement = 0.05;  // Adjust this value for sensitivity
    private double maxPower = 1.0;         // Maximum power for the arm
    private double minPower = -1.0;        // Minimum power for the arm (reverse)

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean currentButtonState = gamepad1.right_stick_button;
            if (currentButtonState && !lastButtonState) {
                halfSpeed = !halfSpeed;
            }
            lastButtonState = currentButtonState;

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double iF = gamepad2.right_trigger;
            double iR = gamepad2.left_trigger;

            if (halfSpeed) {
                y *= 0.3;
                x *= 0.3;
                rx *= 0.3;
            }

            // Gradual arm power adjustment
            if (gamepad2.dpad_up) {
                armPower += powerIncrement;
                if (armPower > maxPower) {
                    armPower = maxPower;  // Cap at maximum power
                }
            } else if (gamepad2.dpad_down) {
                armPower -= powerIncrement;
                if (armPower < minPower) {
                    armPower = minPower;  // Cap at minimum power
                }
            }

            // Set the arm motor power
            arm.setPower(armPower);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontRightPower = (y + x + rx) / denominator;
            double rearRightPower = (y - x + rx) / denominator;
            double rearLeftPower = (y + x - rx) / denominator;
            double frontLeftPower = (y - x - rx) / denominator;
            double intakePower = (iF + iR) / denominator;

            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(rearLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);
            intake.setPower(intakePower);

            telemetry.addData("Half-Speed Mode", halfSpeed ? "ON" : "OFF");
            telemetry.addData("Arm Power", armPower);
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.addData("Front Right Power", frontRight.getPower());
            telemetry.addData("Rear Left Power", rearLeft.getPower());
            telemetry.addData("Rear Right Power", rearRight.getPower());
            telemetry.addData("Battery Voltage", String.format("%.2f V", hardwareMap.voltageSensor.get("Control Hub").getVoltage()));
            telemetry.update();
        }
    }
}