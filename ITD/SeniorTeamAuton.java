package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class SeniorTeamAuton extends LinearOpMode
{
    public DcMotor FR, FL, BR, BL, Slides1, Slides2;
    private double powerRY, powerRX, powerLX, powerLY, robotAngle, PowerMultiplier, lf, rb, rf, lb;
    public CRServo Roller;
    public Servo Elbow1, Elbow2, EWrist1, EWrist2, TopWrist;

    //CPI is calculated by ((Clicks of Encoders per revolution)/(diameter*pi)
    //Clicks of Encoders per revolution may change based off of different model types

    // TEST THESE VALUES!!
    double cpi = 7; // cycles per inch
    double cpd = 3.7; // clicks per degree
    double acpi = 10; // cpi for arm- verify number

    public enum robotMotion
    {
        right, left, armUp, armDown;
    }

    // TODO: update directions, # of motors, and add any necessary functions (most likely intake/outtake)

    @Override
    public void runOpMode() throws InterruptedException
    {


        // hardware map
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        //Arm = hardwareMap.get(DcMotor.class, "Arm");


        // set direction

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);



        // set mode
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        // set initial power/position
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        // Each square is 24x24
        waitForStart();


        // FOR SCRIMMAGE

        // PRELOADED SAMPLE
        forward(4, 0.3);
        sleep(1000);
        movementRL(robotMotion.left, 45, 0.1);
        sleep(1000);
        forward(3, 0.3);
        sleep(1000);

        // TO SAMPLE #1
        backward(5, 0.3);
        sleep(1000);
        movementRL(robotMotion.right, 45, 0.1);
        sleep(1000);
        forward(4, 0.3);
        sleep(1000);
        movementRL(robotMotion.right, 90, 0.1);
        forward(15, 0.3);

        // PUT INTO LOOP LATER

        // PICK UP SAMPLE #1
        Roller.setPower(0.5);

//        backward(5, 0.3);
//        sleep(1000);
//        movementRL(robotMotion.left, 90, 0.1);
//        sleep(1000);
//        forward(3, 0.3);
//        sleep(1000);
//        movementRL(robotMotion.left, 90, 0.1);
//        sleep(1000);
//        forward(10, 0.3);
//        sleep(1000);
//        movementRL(robotMotion.left, 90, 0.1); // test
//        sleep(1000);
//        movementRL(robotMotion.left, 45, 0.1); // test
//        sleep(1000);
//        forward(8, 0.3);

        // DEPOSIT SAMPLE #1 - control wrist, slides
//        EWrist1.setPosition();
//        EWrist2.setPosition();
//        Slides1.setPower(-1.4);
//        Slides2.setPower(-1.4);
//        TopWrist.setPosition();
//        EWrist1.setPosition();
//        EWrist2.setPosition();
//        Slides1.setPower(0.85);
//        Slides2.setPower(0.85);

        // TO SAMPLE #2
//        backward(12, 0.3);
//        sleep(1000);
//        movementRL(robotMotion.right, 45, 0.1);
//        sleep(1000);
//        forward(2, 0.3);
//        sleep(1000);
//        movementRL(robotMotion.right, 90, 0.1);
//        forward(15, 0.3);

        // PICK UP SAMPLE #2
//        Roller.setPower(0.5);


        // TO SAMPLE #2
//        backward(5, 0.3);
//        sleep(1000);
//        movementRL(robotMotion.left, 90, 0.1);
//        sleep(1000);
//        forward(2, 0.3);
//        sleep(1000);
//        movementRL(robotMotion.left, 90, 0.1);
//        forward(15, 0.3);
//        sleep(1000);
//        movementRL(robotMotion.left, 90, 0.1); // test
//        sleep(1000);
//        movementRL(robotMotion.left, 45, 0.1); // test
//        sleep(1000);
//        forward(8, 0.3);
//        sleep(1000);

        // DEPOSIT SAMPLE #2 - control wrist, slides
//        EWrist1.setPosition(0);
//        EWrist2.setPosition(0);
//        Slides1.setPower(-1.4);
//        Slides2.setPower(-1.4);
//        TopWrist.setPosition();
//        EWrist1.setPosition(0);
//        EWrist2.setPosition(0);
//        Slides1.setPower(0.85);
//        Slides2.setPower(0.85);

        // TO SAMPLE #3
//        backward(6, 0.3);
//        movementRL(robotMotion.right, 45, 0.1); // test
//        forward(3, 0.3);
//        movementRL(robotMotion.right, 90, 0.1); // test
//        forward(8, 0.3);

        // PICK UP SAMPLE #3
//        Roller.setPower(0.5);

//        backward(2, 0.3);
//        movementRL(robotMotion.left, 90, 0.1); // test
//        forward(15, 0.3);

        // DEPOSIT SAMPLE #3
//        EWrist1.setPosition(0);
//        EWrist2.setPosition(0);
//        Slides1.setPower(-1.4);
//        Slides2.setPower(-1.4);
//        TopWrist.setPosition();
//        EWrist1.setPosition(0);
//        EWrist2.setPosition(0);
//        Slides1.setPower(0.85);
//        Slides2.setPower(0.85);

    }


    private void forward(double inch,  double power)
    {
        // calculate new position for (wheel) motors
        // make sure you cast to int!
        int a = (int)(FL.getCurrentPosition() + (inch * cpi));
        int b = (int)(FR.getCurrentPosition() + (inch * cpi));
        int c = (int)(BL.getCurrentPosition() + (inch * cpi));
        int d = (int)(BR.getCurrentPosition() + (inch * cpi));


        // sets new position for motors
        FL.setTargetPosition(a);
        FR.setTargetPosition(b);
        BL.setTargetPosition(c);
        BR.setTargetPosition(d);


        // sets desired power for motors
        // TODO: Do for each wheel motor
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);


        // sets motors to RUN_TO_POSITION
        // TODO: Do for each wheel motor
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //loop to run encoders method
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())
        {
            // TODO: Add telemetry data here
            telemetry.addData("FL Target", a);
            telemetry.addData("FL Actual", FL.getCurrentPosition());

            telemetry.addData("FR Target", b);
            telemetry.addData("FR Actual", FR.getCurrentPosition());

            telemetry.addData("BL Target", c);
            telemetry.addData("BL Actual", BL.getCurrentPosition());

            telemetry.addData("BR Target", d);
            telemetry.addData("BR Actual", BR.getCurrentPosition());

            telemetry.update();
        }


        // stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);




    }


    private void backward(double inch,  double power)
    {
        // calculate new position for motors
        int a = (int)(FL.getCurrentPosition() - (inch * cpi));
        int b = (int)(FR.getCurrentPosition() - (inch * cpi));
        int c = (int)(BL.getCurrentPosition() - (inch * cpi));
        int d = (int)(BR.getCurrentPosition() - (inch * cpi));


        // sets new position for motors
        FL.setTargetPosition(-a);
        FR.setTargetPosition(-b);
        BL.setTargetPosition(-c);
        BR.setTargetPosition(-d);


        // sets desired power for motors
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);


        // make motors run to position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // loop to run encoders method
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())
        {
            telemetry.addData("FL Target", a);
            telemetry.addData("FL Actual", FL.getCurrentPosition());

            telemetry.addData("FR Target", b);
            telemetry.addData("FR Actual", FR.getCurrentPosition());

            telemetry.addData("BL Target", c);
            telemetry.addData("BL Actual", BL.getCurrentPosition());

            telemetry.addData("BR Target", d);
            telemetry.addData("BR Actual", BR.getCurrentPosition());

            telemetry.update();

        }

        // stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);


    }


    private void movementRL(robotMotion action, double degree,  double power)
    {


        if(action == robotMotion.left)
        {
            // left is moving backwards, right is moving forwards
            FL.setTargetPosition((int) (FL.getCurrentPosition() - (degree * cpd)));
            FR.setTargetPosition((int) (FR.getCurrentPosition() + (degree * cpd)));
            BL.setTargetPosition((int) (BL.getCurrentPosition() - (degree * cpd)));
            BR.setTargetPosition((int) (BR.getCurrentPosition() + (degree * cpd)));


            // sets desired power for motors
            FL.setPower(-power);
            FR.setPower(power);
            BL.setPower(-power);
            BR.setPower(power);




            // make motors run to position
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);




            // loop to run encoders method
            while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())
            {
                telemetry.addLine("turn right");

                telemetry.update();
            }


            // stop motors
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }



        if(action == robotMotion.right)
        {
            // calculates new position for motors
            FL.setTargetPosition((int) (FL.getCurrentPosition() + (degree * cpd)));
            FR.setTargetPosition((int) (FR.getCurrentPosition() - (degree * cpd)));
            BL.setTargetPosition((int) (BL.getCurrentPosition() + (degree * cpd)));
            BR.setTargetPosition((int) (BR.getCurrentPosition() - (degree * cpd)));


            // sets desired power for motors
            FL.setPower(power);
            FR.setPower(-power);
            BL.setPower(power);
            BR.setPower(-power);


            // make motors run to position
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // loop to run encoders method
            while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())
            {
                telemetry.addLine("turn left");


                telemetry.update();
            }


            // stop motors
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }


    }

    private void sleep(int sleep, double power)
    {
        BR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
    }


}
