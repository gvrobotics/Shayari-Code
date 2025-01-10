package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// TEST DURING MEETING

@Autonomous
public class Auton_IMU extends LinearOpMode {

    public DcMotor FR, FL, BR, BL, Slides1, Slides2;
    public Servo Elbow1, Elbow2, SClaw, FClaw, SWrist1, SWrist2;
    public Servo Wrist;
    private IMU imu;
    private Orientation angles;

    // double cpi = 50; (this cpi was too short, around 2.5 inches- keeping as backup tho!)
    double cpi = 60;

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

        Slides1 = hardwareMap.get(DcMotor.class, "Slides1");
        Slides2 = hardwareMap.get(DcMotor.class, "Slides2");
        Elbow1 = hardwareMap.get(Servo.class, "Elbow1");
        Elbow2 = hardwareMap.get(Servo.class, "Elbow2");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        SClaw = hardwareMap.get(Servo.class, "SClaw");
        FClaw = hardwareMap.get(Servo.class, "FClaw");
        SWrist1 = hardwareMap.get(Servo.class, "SWrist1");
        SWrist2 = hardwareMap.get(Servo.class, "SWrist2");

        // Set motor directions
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);

        Slides1.setDirection(DcMotor.Direction.REVERSE);
        Slides2.setDirection(DcMotor.Direction.FORWARD);
        SClaw.setDirection(Servo.Direction.FORWARD);
        // set directions for FClaw, SWrist1, SWrist2

        // SWrist1 and SWrist2 must be opposite directions!!
        SWrist1.setDirection(Servo.Direction.FORWARD);
        SWrist2.setDirection(Servo.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Slides1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        waitForStart();


        // COMMENT AND TEST ONE AT A TIME

        // RED/BLUE OBSERVATION
        SClaw.setPosition(0.5);
        linearup(11.5, 0.5);
        SWrist1.setPosition(0.65);
        SWrist2.setPosition(0.65);
        sleep(500);
        backward(19.5, 0.4);
        sleep(500);
        SClaw.setPosition(0.9);
        sleep(500);
        // set back
        SWrist1.setPosition(0.95);
        SWrist2.setPosition(0.95);
        forward(5, 0.4);
        sleep(500);
        lineardown(11.5, 0.5);
        sleep(200);
        strafeLeft(20, 0.6);
        sleep(500);
        spinRight(40, 0.3); // distance 40, power 0.3 is magical
        backward(7.5, 0.3);
        SClaw.setPosition(0.5);
        backward(5, 0.4);
//        SClaw.setPosition(0.9);



        // THIS CODE HAS NOT BEEN TESTED
//        strafeRight(20, 0.3);
//        strafeRight(5, 0.3);
//        backward(15, 0.3);
//
//        strafeRight(6, 0.3);
//        backward(40, 0.3); // push "sample" into parking
//        forward(40, 0.3);
//        strafeRight(12, 0.3);
//        backward(40, 0.3); // push "sample" into parking
//        forward(40, 0.3);
//        strafeRight(12, 0.3);
//        backward(40, 0.3); // push "sample" into parking
//        strafeLeft(4, 0.3);
//        backward(2, 0.3); // PARK


    }

    private void forward(double distance, double power) {
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

    private void backward(double distance, double power) {
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

    private void spinRight(double distance, double power) {
        // Get initial heading
        angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initialHeading = angles.firstAngle;

        // Spin
        FR.setPower(-power);
        FL.setPower(power);
        BR.setPower(-power);
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
            FR.setPower(-power);
            FL.setPower(power);
            BR.setPower(-power);
            BL.setPower(power);

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
        BR.setPower(-power);
        BL.setPower(-power);
        FR.setPower(power);
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
            BR.setPower(-power);
            BL.setPower(-power);
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
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    private void linearup(double inch, double power)
    {
        Slides1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slides2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // ADD DEBOUNCING TO SLIDES??
        // debounce delay 500 ms

        int a = (int) ((cpi*inch) + Slides1.getCurrentPosition());
        int b = (int) ((cpi*inch) + Slides2.getCurrentPosition());
        Slides1.setTargetPosition(a);
        Slides2.setTargetPosition(b);
        Slides1.setPower(power);
        Slides2.setPower(power);

        if (Slides1.getCurrentPosition() == a && Slides2.getCurrentPosition() == b)
        {
            Slides1.setPower(0);
            Slides2.setPower(0);
        }

        while (Slides2.isBusy() && Slides1.isBusy())
        {
            telemetry.addLine("Linear up");
            telemetry.addData("Target Slides1", "%7d", a);
            telemetry.addData("Target Slides2", "%7d", b);
            telemetry.addData("Actual Slides1", "%7d", Slides1.getCurrentPosition());
            telemetry.addData("Actual Slides2", "%7d", Slides2.getCurrentPosition());
            telemetry.update();
        }

    }

    private void lineardown(double inch, double power)
    {
        Slides1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slides2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int a = (int) (Slides1.getCurrentPosition() - (cpi*inch));
        int b = (int) (Slides2.getCurrentPosition() - (cpi*inch));
        Slides1.setTargetPosition(a);
        Slides2.setTargetPosition(b);
        Slides1.setPower(power);
        Slides2.setPower(power);

        if (Slides1.getCurrentPosition() == a && Slides2.getCurrentPosition() == b)
        {
            Slides1.setPower(0);
            Slides2.setPower(0);
        }

        while (Slides2.isBusy() && Slides1.isBusy())
        {
            telemetry.addLine("Linear up");
            telemetry.addData("Target Slides1", "%7d", a);
            telemetry.addData("Target Slides2", "%7d", b);
            telemetry.addData("Actual Slides1", "%7d", Slides1.getCurrentPosition());
            telemetry.addData("Actual Slides2", "%7d", Slides2.getCurrentPosition());
            telemetry.update();
        }



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
