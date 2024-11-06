package org.firstinspires.ftc.teamcode;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.time.LocalDateTime;

@TeleOp
public class RecordAuto extends LinearOpMode {
    private boolean halfSpeed = false;
    private boolean lastButtonState = false;
    private boolean recording = false;
    private StringBuilder dataBuffer = new StringBuilder();
    private double targetArmPosition = 0;
    private double fieldOffset = 0;
    private ArmControl armControl;

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

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean currentButtonState = gamepad1.right_stick_button;
            if (currentButtonState && !lastButtonState) {
                halfSpeed = !halfSpeed;
            }
            lastButtonState = currentButtonState;

            if (gamepad1.dpad_up && !recording) {
                recording = true;
                dataBuffer.setLength(0);
            }
            if (gamepad1.dpad_down && recording) {
                recording = false;
                sendDataToServer(dataBuffer.toString());
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
            rx = Math.abs(rx) > 0.1 ? turningSpeed * Math.signum(rx) : 0;

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
                targetArmPosition -= 5;
            } else if (gamepad2.dpad_down) {
                targetArmPosition += 5;
            }
            armControl.setPosition(targetArmPosition);
            armControl.update();

            if (recording) {
                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                String data = null;
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                    data = String.format(
                            "{\"timestamp\": \"%s\", \"gamepad1\": {\"x\": %.2f, \"y\": %.2f, \"rx\": %.2f}, " +
                                    "\"motors\": {\"frontRight\": %.2f, \"rearRight\": %.2f, \"rearLeft\": %.2f, " +
                                    "\"frontLeft\": %.2f, \"intake\": %.2f}, \"heading\": %.2f}\n",
                            LocalDateTime.now(), x, y, rx, frontRightPower, rearRightPower,
                            rearLeftPower, frontLeftPower, intakePower, heading
                    );
                }
                dataBuffer.append(data);
            }

            telemetry.addData("Half-Speed Mode", halfSpeed ? "ON" : "OFF");
            telemetry.addData("Arm Target Position", armControl.getArmTargetPosition());
            telemetry.addData("Arm Power", armControl.getArmPower());
            telemetry.addData("Arm Position", armControl.getArmPosition());
            telemetry.addData("Heading", botHeading);
            telemetry.addData("Battery Voltage", String.format("%.2f V", hardwareMap.voltageSensor.get("Control Hub").getVoltage()));
            telemetry.update();
        }
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
