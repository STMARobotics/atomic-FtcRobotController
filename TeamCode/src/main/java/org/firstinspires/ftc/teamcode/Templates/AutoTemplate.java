//package org.firstinspires.ftc.teamcode.Templates;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@Autonomous
//public class AutoTemplate extends LinearOpMode {
//    // Declare variables
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
//        DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
//        DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
//        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        telemetry.addData("Status", "Initialized. Waiting for start...");
//        telemetry.update();
//        waitForStart();
//        if (isStopRequested()) return;
//
//
//
//
//
//        //stuff i think
//
//
//    }
//
//    private void moveDrivetrain(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double power, int duration) throws InterruptedException {
//        frontLeft.setPower(power);
//        rearLeft.setPower(power);
//        frontRight.setPower(power);
//        rearRight.setPower(power);
//        sleep(duration);
//    }
//
//    private void stopDrivetrain(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight) {
//        frontLeft.setPower(0);
//        rearLeft.setPower(0);
//        frontRight.setPower(0);
//        rearRight.setPower(0);
//    }
//
//    private void rotateDrivetrainLeft(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double power, int duration) throws InterruptedException {
//        frontLeft.setPower(-power);
//        rearLeft.setPower(-power);
//        frontRight.setPower(power);
//        rearRight.setPower(power);
//        sleep(duration);
//    }
//
//    private void rotateDrivetrainRight(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double power, int duration) throws InterruptedException {
//        frontLeft.setPower(power);
//        rearLeft.setPower(power);
//        frontRight.setPower(-power);
//        rearRight.setPower(-power);
//        sleep(duration);
//    }
//
//
//}
