package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class Auton_IMU extends LinearOpMode {

    public DcMotor FR, FL, BR, BL;
    private IMU imu;
    private Orientation angles;

    // Target distance to move (in inches)
    private double targetDistance = 1;

    // Motor power
    private double motorPower = 0.5;

    // Estimated robot speed (inches per second)
    private double robotSpeed = 10; // Adjust this value based on your robot's speed

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware map
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        // Set motor directions
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        waitForStart();

        // Move forward using IMU
//        moveForward(targetDistance, motorPower);
//        sleep(10);
//        moveBackward(targetDistance, motorPower);
//        strafeLeft(1, motorPower);
        strafeRight(1, motorPower);
    }

    private void moveForward(double distance, double power) {
        // Get initial heading
        angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initialHeading = angles.firstAngle;

        // Start moving forward
        FR.setPower(power);
        FL.setPower(power);
        BR.setPower(power);
        BL.setPower(power);

        // Get the current time in milliseconds
        long startTime = System.currentTimeMillis();

        // Move for estimated time
        while ((System.currentTimeMillis() - startTime) < (distance / robotSpeed) * 1000) {
            // Get current heading
            angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            // Calculate heading error
            double headingError = currentHeading - initialHeading;

            // Adjust motor powers based on heading error (optional)
            // double leftPower = power - headingError * kP;
            // double rightPower = power + headingError * kP;

            // Set motor powers (with optional heading correction)
            // BR.setPower(leftPower);
            // BL.setPower(leftPower);
            // FR.setPower(rightPower);
            // FL.setPower(rightPower);

            // For now, keep motor powers constant
            BR.setPower(power);
            BL.setPower(power);
            FR.setPower(power);
            FL.setPower(power);

            // Calculate elapsed time
            double elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0;

            // Estimate distance traveled
            double estimatedDistance = elapsedTime * robotSpeed;

            // Display telemetry
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Time Elapsed", elapsedTime);
            telemetry.addData("Estimated Distance", estimatedDistance);
            telemetry.update();

            sleep(10); // Adjust sleep time for telemetry updates
        }

        // Stop motors
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    private void moveBackward(double distance, double power) {
        // Get initial heading
        angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initialHeading = angles.firstAngle;

        // Start moving backward
        FR.setPower(-power);
        FL.setPower(-power);
        BR.setPower(-power);
        BL.setPower(-power);

        // Get the current time in milliseconds
        long startTime = System.currentTimeMillis();

        // Move for estimated time
        while ((System.currentTimeMillis() - startTime) < (distance / robotSpeed) * 1000) {
            // Get current heading
            angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            // Calculate heading error
            double headingError = currentHeading - initialHeading;

            // Adjust motor powers based on heading error (optional)
            // double leftPower = power - headingError * kP;
            // double rightPower = power + headingError * kP;

            // Set motor powers (with optional heading correction)
            // BR.setPower(leftPower);
            // BL.setPower(leftPower);
            // FR.setPower(rightPower);
            // FL.setPower(rightPower);

            // For now, keep motor powers constant
            BR.setPower(-power);
            BL.setPower(-power);
            FR.setPower(-power);
            FL.setPower(-power);

            // Calculate elapsed time
            double elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0;

            // Estimate distance traveled
            double estimatedDistance = elapsedTime * robotSpeed;

            // Display telemetry
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Time Elapsed", elapsedTime);
            telemetry.addData("Estimated Distance", estimatedDistance);
            telemetry.update();

            sleep(10); // Adjust sleep time for telemetry updates
        }

        // Stop motors
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    private void strafeLeft (double distance,  double power) {

        // Get initial heading
        angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initialHeading = angles.firstAngle;

        // sets desired power for motors
        BR.setPower(-power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(-power);

        // Get the current time in milliseconds
        long startTime = System.currentTimeMillis();

        // Move for estimated time
        while ((System.currentTimeMillis() - startTime) < (distance / robotSpeed) * 1000) {
            // Get current heading
            angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            // Calculate heading error
            double headingError = currentHeading - initialHeading;

            // Adjust motor powers based on heading error (optional)
            // double leftPower = power - headingError * kP;
            // double rightPower = power + headingError * kP;

            // Set motor powers (with optional heading correction)
            // BR.setPower(leftPower);
            // BL.setPower(leftPower);
            // FR.setPower(rightPower);
            // FL.setPower(rightPower);

            // For now, keep motor powers constant
            BR.setPower(-power);
            BL.setPower(power);
            FR.setPower(power);
            FL.setPower(-power);

            // Calculate elapsed time
            double elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0;

            // Estimate distance traveled
            double estimatedDistance = elapsedTime * robotSpeed;

            // Display telemetry
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Time Elapsed", elapsedTime);
            telemetry.addData("Estimated Distance", estimatedDistance);
            telemetry.update();

            sleep(10); // Adjust sleep time for telemetry updates
        }

        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    private void strafeRight (double distance,  double power) {

        // Get initial heading
        angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initialHeading = angles.firstAngle;

        // sets desired power for motors
        BR.setPower(power);
        BL.setPower(-power);
        FR.setPower(-power);
        FL.setPower(power);

        // Get the current time in milliseconds
        long startTime = System.currentTimeMillis();

        // Move for estimated time
        while ((System.currentTimeMillis() - startTime) < (distance / robotSpeed) * 1000) {
            // Get current heading
            angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            // Calculate heading error
            double headingError = currentHeading - initialHeading;

            // Adjust motor powers based on heading error (optional)
            // double leftPower = power - headingError * kP;
            // double rightPower = power + headingError * kP;

            // Set motor powers (with optional heading correction)
            // BR.setPower(leftPower);
            // BL.setPower(leftPower);
            // FR.setPower(rightPower);
            // FL.setPower(rightPower);
            // For now, keep motor powers constant
            BR.setPower(power);
            BL.setPower(-power);
            FR.setPower(-power);
            FL.setPower(power);

            // Calculate elapsed time
            double elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0;

            // Estimate distance traveled
            double estimatedDistance = elapsedTime * robotSpeed;

            // Display telemetry
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Time Elapsed", elapsedTime);
            telemetry.addData("Estimated Distance", estimatedDistance);
            telemetry.update();

            sleep(10); // Adjust sleep time for telemetry updates

        }
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    // FIX LEFT AND RIGHT

//    private void movementRL(AutonTest.robotMotion action, double degree, double power) {
//
//        if(action == AutonTest.robotMotion.left) {
//            // left is moving backwards, right is moving forwards
//            BR.setTargetPosition((int) (BR.getCurrentPosition() + (degree * cpd)));
//            BL.setTargetPosition((int) (BL.getCurrentPosition() - (degree * cpd)));
//            FR.setTargetPosition((int) (FR.getCurrentPosition() + (degree * cpd)));
//            FL.setTargetPosition((int) (FL.getCurrentPosition() - (degree * cpd)));
//
//            // sets desired power for motors
//            BR.setPower(power);
//            BL.setPower(-power);
//            FR.setPower(power);
//            FL.setPower(-power);
//
//            // make motors run to position
//            // done in init
//
//            // loop to get telemetry while motors are running
//            while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
//                telemetry.addLine("Turning Left");
//
//                telemetry.addData("BR Target", BR.getTargetPosition());
//                telemetry.addData("BL Target", BL.getTargetPosition());
//                telemetry.addData("FR Target", FR.getTargetPosition());
//                telemetry.addData("FL Target", FL.getTargetPosition());
//
//                telemetry.addData("BR Current", BR.getCurrentPosition());
//                telemetry.addData("BL Current", BL.getCurrentPosition());
//                telemetry.addData("FR Current", FR.getCurrentPosition());
//                telemetry.addData("FL Current", FL.getCurrentPosition());
//
//                telemetry.update();
//            }
//            // stop motors
//            BR.setPower(0);
//            BL.setPower(0);
//            FR.setPower(0);
//            FL.setPower(0);
//        }
//
//        if(action == AutonTest.robotMotion.right) {
//            // right is moving backwards, left is moving forwards
//            BR.setTargetPosition((int) (BR.getCurrentPosition() - (degree * cpd)));
//            BL.setTargetPosition((int) (BL.getCurrentPosition() + (degree * cpd)));
//            FR.setTargetPosition((int) (FR.getCurrentPosition() - (degree * cpd)));
//            FL.setTargetPosition((int) (FL.getCurrentPosition() + (degree * cpd)));
//
//            // sets desired power for motors
//            BR.setPower(-power);
//            BL.setPower(power);
//            FR.setPower(-power);
//            FL.setPower(power);
//
//            // make motors run to position
//            // done in init
//
//            // loop to get telemetry while motors are running
//            while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
//                telemetry.addLine("Turning Right");
//
//                telemetry.addData("BR Target", BR.getTargetPosition());
//                telemetry.addData("BL Target", BL.getTargetPosition());
//                telemetry.addData("FR Target", FR.getTargetPosition());
//                telemetry.addData("FL Target", FL.getTargetPosition());
//
//                telemetry.addData("BR Current", BR.getCurrentPosition());
//                telemetry.addData("BL Current", BL.getCurrentPosition());
//                telemetry.addData("FR Current", FR.getCurrentPosition());
//                telemetry.addData("FL Current", FL.getCurrentPosition());
//
//                telemetry.update();
//            }
//            // stop motors
//            BR.setPower(0);
//            BL.setPower(0);
//            FR.setPower(0);
//            FL.setPower(0);
//        }
//
//    }


}
