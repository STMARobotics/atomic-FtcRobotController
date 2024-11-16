package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ArmControl;

import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;

@TeleOp
public class RecordAuto extends LinearOpMode {
    private boolean halfSpeed = false;
    private boolean lastButtonState = false;
    private double fieldOffset = 0;
    private ArmControl armControl;

    private double targetArmPosition = 0;

    // Recording state, start time, and data buffer
    private boolean recording = false;
    private long startTime;
    private StringBuilder dataBuffer = new StringBuilder();
    private static final String SERVER_URL = "http://192.168.43.155:5000/record_data";

    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        final DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        final DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        final DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        final DcMotor intake = hardwareMap.dcMotor.get("intake");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        final IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        armControl = new ArmControl(hardwareMap);
        armControl.resetZero();

        // Wait for start
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean currentButtonState = gamepad1.right_stick_button;
            if (currentButtonState && !lastButtonState) {
                halfSpeed = !halfSpeed;
            }
            lastButtonState = currentButtonState;

            // Toggle recording with dpad_up and stop with dpad_down
            if (gamepad1.dpad_up && !recording) {
                recording = true;
                startTime = System.currentTimeMillis(); // Set the start time
                dataBuffer.setLength(0); // Clear previous data
                telemetry.addData("Recording", "Started");
            }

            if (gamepad1.dpad_down && recording) {
                recording = false;
                sendDataToServer(dataBuffer.toString()); // Send data to the server
                telemetry.addData("Recording", "Stopped and data sent");
            }

            // Record gamepad inputs if recording is active
            if (recording) {
                recordGamepadInputs();
            }

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double iF = gamepad2.right_trigger * -0.45;
            double iR = gamepad2.left_trigger * 0.45;

            if (halfSpeed) {
                y *= 0.3;
                x *= 0.3;
            }

            if (gamepad1.y) {
                fieldOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - fieldOffset;
            double headingRad = Math.toRadians(botHeading);

            double temp = y * Math.cos(headingRad) - x * Math.sin(headingRad);
            x = y * Math.sin(headingRad) + x * Math.cos(headingRad);
            y = temp;

            double turningSpeed = 0.3;
            if (Math.abs(rx) > 0.1) {
                rx = turningSpeed * Math.signum(rx);
            } else {
                rx = 0;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontRightPower = (y + x + rx) / denominator;
            double rearRightPower = (y - x + rx) / denominator;
            double rearLeftPower = (y + x - rx) / denominator;
            double frontLeftPower = (y - x - rx) / denominator;
            double intakePower = (iF + iR) / denominator;

            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);
            rearLeft.setPower(rearLeftPower);
            frontLeft.setPower(frontLeftPower);
            intake.setPower(intakePower);

            if (gamepad2.dpad_up) {
                targetArmPosition -= 2;
            } else if (gamepad2.dpad_down) {
                targetArmPosition += 2;
            } else if (gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1) {
                targetArmPosition += gamepad2.right_stick_y * 5;
            }

            armControl.setPosition(targetArmPosition);
            armControl.update();

            telemetry.addData("Half-Speed Mode", halfSpeed ? "ON" : "OFF");
            telemetry.addData("Recording Status", recording ? "Active" : "Inactive");
            telemetry.addData("Arm Target Position", armControl.getArmTargetPosition());
            telemetry.addData("Arm Power", armControl.getArmPower());
            telemetry.addData("Arm Position", armControl.getArmPosition());
            telemetry.addData("Heading", botHeading);
            telemetry.addData("Battery Voltage", String.format("%.2f V", hardwareMap.voltageSensor.get("Control Hub").getVoltage()));
            telemetry.update();
        }
    }

    private void recordGamepadInputs() {
        long elapsedMillis = System.currentTimeMillis() - startTime;
        double elapsedSeconds = elapsedMillis / 1000.0;

        String data = String.format(
                "{\"timestamp\": %.2f, \"gamepad1\": {\"left_stick_y\": %.2f, \"left_stick_x\": %.2f, \"right_stick_x\": %.2f, " +
                        "\"right_trigger\": %.2f, \"left_trigger\": %.2f, \"dpad_up\": %b, \"dpad_down\": %b, \"dpad_left\": %b, \"dpad_right\": %b, " +
                        "\"a\": %b, \"b\": %b, \"x\": %b, \"y\": %b}, " +
                        "\"gamepad2\": {\"left_stick_y\": %.2f, \"left_stick_x\": %.2f, \"right_stick_x\": %.2f, \"right_stick_y\": %.2f, " +
                        "\"right_trigger\": %.2f, \"left_trigger\": %.2f, \"dpad_up\": %b, \"dpad_down\": %b, \"dpad_left\": %b, \"dpad_right\": %b, " +
                        "\"a\": %b, \"b\": %b, \"x\": %b, \"y\": %b}}\n",
                elapsedSeconds,
                gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,
                gamepad1.right_trigger, gamepad1.left_trigger, gamepad1.dpad_up, gamepad1.dpad_down,
                gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y,
                gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x, gamepad2.right_stick_y,
                gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.dpad_up, gamepad2.dpad_down,
                gamepad2.dpad_left, gamepad2.dpad_right, gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y
        );

        dataBuffer.append(data);
    }

    private void sendDataToServer(String data) {
        try {
            URL url = new URL(SERVER_URL);
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("POST");
            connection.setDoOutput(true);
            connection.setRequestProperty("Content-Type", "application/json");

            String jsonData = "{\"data\": \"" + data.replace("\n", "\\n") + "\"}";
            byte[] outputBytes = jsonData.getBytes(StandardCharsets.UTF_8);
            OutputStream os = connection.getOutputStream();
            os.write(outputBytes);
            os.flush();
            os.close();

            int responseCode = connection.getResponseCode();
            telemetry.addData("Server Response", responseCode == HttpURLConnection.HTTP_OK ? "Data sent successfully" : "Failed to send data: " + responseCode);
            connection.disconnect();
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to send data: " + e.getMessage());
        }
    }
}
