package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;

//import com.qualcomm.robotcore.hardware.IMU; -- see if this is needed

// clean up autontest : test forward, test gyro, test forward + gyro

// push block with side of robot

@Autonomous

public class JagGyro extends LinearOpMode {

    public DcMotor FR, FL, BR, BL, Slides1, Slides2;
    public Servo SClaw, Wrist;
    public GyroSensor JagGyro;
    int xVal, yVal, zVal, heading = 0;

    //hardwareMap.logDevices(); -- is this needed?

    // encoder formula just in case
    //CPI is calculated by ((Clicks of Encoders per revolution)/(diameter*pi)
    //Clicks of Encoders per revolution may change based off of different model types

    // TEST THESE VALUES!!
    // double cpi = 7; // cycles per inch
    double cpi = 30; // cpi is being tested!!
    double cpd = 7; // clicks per degree
    //double acpi = 10; // cpi for arm- verify number

    public enum robotMotion
    {
        right, left, linearup, lineardown;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // hardware map
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        Slides1 = hardwareMap.get(DcMotor.class, "Slides1");
        Slides2 = hardwareMap.get(DcMotor.class, "Slides2");
        JagGyro = hardwareMap.get(GyroSensor.class, "imu");


        Wrist = hardwareMap.get(Servo.class, "Wrist");
        SClaw = hardwareMap.get(Servo.class, "SClaw");

        /*
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);

         */
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        Slides1.setDirection(DcMotorSimple.Direction.FORWARD);
        Slides2.setDirection(DcMotorSimple.Direction.FORWARD);
        SClaw.setDirection(Servo.Direction.REVERSE);

        // set mode
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // set initial power/position
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        Slides1.setPower(0);
        Slides2.setPower(0);
        //EWrist1.setPosition(0.7);
        Wrist.setPosition(0.3);
        //TopWrist.setPosition(-1);
        SClaw.setPosition(0.6);

        //motor powers:
        //forward/backward: 0.5
        //strafe: 0.5
        //right/left: 0.3

        //fl is positive

        // calibrate gyro sensor
        JagGyro.calibrate();

        waitForStart();

        while(JagGyro.isCalibrating()) {
            sleep(50);
        }
        JagGyro.resetZAxisIntegrator();

        xVal = JagGyro.rawX();
        yVal = JagGyro.rawY();
        zVal = JagGyro.rawZ();

        heading = JagGyro.getHeading();

        // telemetry data
        telemetry.addData("1. x", String.format("%03d", xVal));
        telemetry.addData("2. y", String.format("%03d", yVal));
        telemetry.addData("3. z", String.format("%03d", zVal));
        telemetry.addData("4. h", String.format("%03d", heading));


        //TODO: Code Here!
        // going really fast + far at 5 inches??

//        forward(2, 0.1);
//        telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
        // GOAL: self-correct FL
        // get current gyro heading
        // start driving forward
        // add power to one side or the other based on gyro heading
        // stop at end point



    }

    // forward went backward so i updated it
    private void forward(double inch,  double power) {
        // calculate new position for (wheel) motors
        int a = (int)(BR.getCurrentPosition() + (inch * cpi));
        int b = (int)(BL.getCurrentPosition() + (inch * cpi));
        int c = (int)(FR.getCurrentPosition() + (inch * cpi));
        int d = (int)(FL.getCurrentPosition() + (inch * cpi));

        // sets new position for motors
        BR.setTargetPosition(a);
        BL.setTargetPosition(b);
        FL.setTargetPosition(c);                                                               ;
        FR.setTargetPosition(d);

        // sets desired power for motors
        BR.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(power);

        // make motors run to position
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // loop to get telemetry while motors are running
        while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
            telemetry.addLine("Forward");

            telemetry.addData("BR Target", a);
            telemetry.addData("BL Target", b);
            telemetry.addData("FR Target", c);
            telemetry.addData("FL Target", d);

            telemetry.addData("BR Current", BR.getCurrentPosition());
            telemetry.addData("BL Current", BL.getCurrentPosition());
            telemetry.addData("FR Current", FR.getCurrentPosition());
            telemetry.addData("FL Current", FL.getCurrentPosition());

            telemetry.update();
        }
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);

    }

    private void backward(double inch,  double power) {
        // calculate new position for (wheel) motors
        int a = (int)(BR.getCurrentPosition() - (inch * cpi));
        int b = (int)(BL.getCurrentPosition() - (inch * cpi));
        int c = (int)(FR.getCurrentPosition() - (inch * cpi));
        int d = (int)(FL.getCurrentPosition() - (inch * cpi));

        // sets new position for motors
        BR.setTargetPosition(a);
        BL.setTargetPosition(b);
        FR.setTargetPosition(c);
        FL.setTargetPosition(d);

        // sets desired power for motors
        BR.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(power);

        // make motors run to position
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // loop to get telemetry while motors are running
        while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
            telemetry.addLine("Backward");

            telemetry.addData("BR Target", a);
            telemetry.addData("BL Target", b);
            telemetry.addData("FR Target", c);
            telemetry.addData("FL Target", d);

            telemetry.addData("BR Current", BR.getCurrentPosition());
            telemetry.addData("BL Current", BL.getCurrentPosition());
            telemetry.addData("FR Current", FR.getCurrentPosition());
            telemetry.addData("FL Current", FL.getCurrentPosition());

            telemetry.update();
        }
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    private void strafeLeft (double inch,  double power) {
        // FL and BR go backward, FR and BL go forward
        BR.setTargetPosition((int) (BR.getCurrentPosition() - (inch * cpi)));
        BL.setTargetPosition((int) (BL.getCurrentPosition() + (inch * cpi)));
        FR.setTargetPosition((int) (FR.getCurrentPosition() + (inch * cpi)));
        FL.setTargetPosition((int) (FL.getCurrentPosition() - (inch * cpi)));

        // sets desired power for motors
        BR.setPower(-power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(-power);

        // make motors run to position
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // loop to get telemetry while motors are running
        while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
            telemetry.addLine("Strafe Left");

            telemetry.addData("BR Target", BR.getTargetPosition());
            telemetry.addData("BL Target", BL.getTargetPosition());
            telemetry.addData("FR Target", FR.getTargetPosition());
            telemetry.addData("FL Target", FL.getTargetPosition());

            telemetry.addData("BR Current", BR.getCurrentPosition());
            telemetry.addData("BL Current", BL.getCurrentPosition());
            telemetry.addData("FR Current", FR.getCurrentPosition());
            telemetry.addData("FL Current", FL.getCurrentPosition());

            telemetry.update();
        }
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    private void strafeRight (double inch,  double power) {
        // FR and BL go backward, FL and BR go forward
        BR.setTargetPosition((int) (BR.getCurrentPosition() + (inch * cpi)));
        BL.setTargetPosition((int) (BL.getCurrentPosition() - (inch * cpi)));
        FR.setTargetPosition((int) (FR.getCurrentPosition() - (inch * cpi)));
        FL.setTargetPosition((int) (FL.getCurrentPosition() + (inch * cpi)));

        // sets desired power for motors
        BR.setPower(power);
        BL.setPower(-power);
        FR.setPower(-power);
        FL.setPower(power);

        // make motors run to position
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // loop to get telemetry while motors are running
        while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
            telemetry.addLine("Strafe Right");

            telemetry.addData("BR Target", BR.getTargetPosition());
            telemetry.addData("BL Target", BL.getTargetPosition());
            telemetry.addData("FR Target", FR.getTargetPosition());
            telemetry.addData("FL Target", FL.getTargetPosition());

            telemetry.addData("BR Current", BR.getCurrentPosition());
            telemetry.addData("BL Current", BL.getCurrentPosition());
            telemetry.addData("FR Current", FR.getCurrentPosition());
            telemetry.addData("FL Current", FL.getCurrentPosition());

            telemetry.update();
        }
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    private void movementRL(robotMotion action, double degree,  double power) {

        if(action == robotMotion.left) {
            // left is moving backwards, right is moving forwards
            BR.setTargetPosition((int) (BR.getCurrentPosition() + (degree * cpd)));
            BL.setTargetPosition((int) (BL.getCurrentPosition() - (degree * cpd)));
            FR.setTargetPosition((int) (FR.getCurrentPosition() + (degree * cpd)));
            FL.setTargetPosition((int) (FL.getCurrentPosition() - (degree * cpd)));

            // sets desired power for motors
            BR.setPower(power);
            BL.setPower(-power);
            FR.setPower(power);
            FL.setPower(-power);

            // make motors run to position
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // loop to get telemetry while motors are running
            while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
                telemetry.addLine("Turning Left");

                telemetry.addData("BR Target", BR.getTargetPosition());
                telemetry.addData("BL Target", BL.getTargetPosition());
                telemetry.addData("FR Target", FR.getTargetPosition());
                telemetry.addData("FL Target", FL.getTargetPosition());

                telemetry.addData("BR Current", BR.getCurrentPosition());
                telemetry.addData("BL Current", BL.getCurrentPosition());
                telemetry.addData("FR Current", FR.getCurrentPosition());
                telemetry.addData("FL Current", FL.getCurrentPosition());

                telemetry.update();
            }
            // stop motors
            BR.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            FL.setPower(0);
        }

        if(action == robotMotion.right) {
            // right is moving backwards, left is moving forwards
            BR.setTargetPosition((int) (BR.getCurrentPosition() - (degree * cpd)));
            BL.setTargetPosition((int) (BL.getCurrentPosition() + (degree * cpd)));
            FR.setTargetPosition((int) (FR.getCurrentPosition() - (degree * cpd)));
            FL.setTargetPosition((int) (FL.getCurrentPosition() + (degree * cpd)));

            // sets desired power for motors
            BR.setPower(-power);
            BL.setPower(power);
            FR.setPower(-power);
            FL.setPower(power);

            // make motors run to position
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // loop to get telemetry while motors are running
            while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
                telemetry.addLine("Turning Right");

                telemetry.addData("BR Target", BR.getTargetPosition());
                telemetry.addData("BL Target", BL.getTargetPosition());
                telemetry.addData("FR Target", FR.getTargetPosition());
                telemetry.addData("FL Target", FL.getTargetPosition());

                telemetry.addData("BR Current", BR.getCurrentPosition());
                telemetry.addData("BL Current", BL.getCurrentPosition());
                telemetry.addData("FR Current", FR.getCurrentPosition());
                telemetry.addData("FL Current", FL.getCurrentPosition());

                telemetry.update();
            }
            // stop motors
            BR.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            FL.setPower(0);
        }

    }
    private void linearup(double inch, double power)
    {
        int a = (int) (Slides1.getCurrentPosition() - (inch*cpi));
        int b = (int) (Slides2.getCurrentPosition() + (inch*cpi));
        Slides1.setTargetPosition(a);
        Slides2.setTargetPosition(b);
        Slides1.setPower(power);
        Slides2.setPower(power);
        Slides1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slides2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Slides2.isBusy())
        {
            telemetry.addLine("Linear up");
            telemetry.addData("Target Slides1", "%7d", a);
            telemetry.addData("Target Slides2", "%7d", b);
            telemetry.addData("Actual Slides1", "%7d", Slides1.getCurrentPosition());
            telemetry.addData("Actual Slides2", "%7d", Slides2.getCurrentPosition());
            telemetry.update();
        }
        //Slides1.setPower(0);
        //Slides2.setPower(0);
    }
    private void lineardown(double inch, double power)
    {
        int a = (int) (Slides1.getCurrentPosition() + (inch * cpi));
        int b = (int) (Slides2.getCurrentPosition() - (inch * cpi));
        Slides1.setTargetPosition(a);
        Slides2.setTargetPosition(b);
        Slides1.setPower(power);
        Slides2.setPower(power);
        Slides1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slides2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Slides1.isBusy() && Slides2.isBusy()) {
            telemetry.addLine("Linear down");
            telemetry.addData("Target", "%7d", a);
            telemetry.addData("Target", "%7d", b);
            telemetry.addData("Actual", "%7d", Slides1.getCurrentPosition());
            telemetry.addData("Actual", "%7d", Slides2.getCurrentPosition());
            telemetry.update();
        }
//        Slides1.setPower(0);
//        Slides2.setPower(0);
    }

}   // end class
