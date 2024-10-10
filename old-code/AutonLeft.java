package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutonLeft extends LinearOpMode
{
    public DcMotor BR, BL, FR, FL,linear;
    public Servo claw;
    public ColorSensor color;

    //clicks per degree
    double cpd = 3;

    //clicks per inch
    double cpi = 7.5;

    //clicks per inch for linear slides since different diameter than wheels and different motor
    double lcpi = 195;

    @Override
    public void runOpMode() throws InterruptedException
    {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        linear = hardwareMap.get(DcMotor.class, "L");
        claw = hardwareMap.get(Servo.class, "c");
        color = hardwareMap.colorSensor.get("Color");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        linear.setDirection(DcMotorSimple.Direction.FORWARD);

        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //   linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        linear.setPower(0);
        claw.setPosition(0);

        waitForStart();

        // claw closes around cone
        claw.setPosition(0.45);

        linearup(12,0.6);
        sleep(1000);
        forward(21,0.1);
        sleep(2000);
        int x = 1;
        int y = 2;
        int z = 3;

        // Turning Left or Right power from 0.25 to 0.15
        final double MOTOR_POWER_RL = 0.15; // speed for motor turning right/left
        final double MOTOR_POWER_FB = 0.40; // speed for motor going forward/backward

        final double LINEAR_POWER = 0.25; // speed for linear slides

        // parking
        if (color.blue()-100 < color.red() && color.blue()+100 > color.red())
        //red (Position one)
        {

            telemetry.addData("RED", x);
            telemetry.update();

            // CONE SEQUENCE
            // GOAL: Puts three cones on high junction in front of stacks
            // NOTE TO SELF: TEST EACH STEP INDIVIDUALLY

            // STEP 1: HIGH JUNCTION
            forward(10, MOTOR_POWER_FB); // almost half tile length
            right(90, MOTOR_POWER_RL);
            forward(10, MOTOR_POWER_FB); // almost half tile length
            left(90, MOTOR_POWER_RL);
            forward(37, MOTOR_POWER_FB); // exit that tile
            linearup(21.5, LINEAR_POWER); // raise to high junction
            claw.setPosition(0.5); // claw opens and releases cone
            claw.setPosition(0.45); // closes claw

            // STEP 2: HIGH JUNCTION (SOUTHWEST OF FIRST CONE)
            left(90, MOTOR_POWER_RL);
            forward(25, MOTOR_POWER_FB); // go to cone stacks
            left(90, MOTOR_POWER_RL);
            forward(10, MOTOR_POWER_FB);
            lineardown(10.5, LINEAR_POWER);
            claw.setPosition(0.5); // opens claw
            claw.setPosition(0.45); // closes claw
            left(90, MOTOR_POWER_RL);
            forward(46, MOTOR_POWER_FB); // around two tile lengths
            right(90, MOTOR_POWER_RL);
            forward(10, MOTOR_POWER_FB);
            linearup(10.5, LINEAR_POWER); // raise to high junction
            claw.setPosition(0.5); // claw opens and releases cone
            claw.setPosition(0.45); // closes claw


            // STEP 3: HIGH JUNCTION (SOUTHEAST OF SECOND CONE)
            // back up
            backward(10, MOTOR_POWER_FB);
            left(90, MOTOR_POWER_RL);
            forward(43, MOTOR_POWER_FB);
            lineardown(10.5, LINEAR_POWER);
            claw.setPosition(0.5); // opens claw
            claw.setPosition(0.45); // closes claw
            right(90, MOTOR_POWER_RL);
            forward(10, MOTOR_POWER_FB);
            right(90, MOTOR_POWER_RL);
            forward(43, MOTOR_POWER_FB);
            linearup(10.5, LINEAR_POWER); // raise to high junction
            claw.setPosition(0.5); // claw opens and releases cone
            claw.setPosition(0.45); // closes claw

            // PARKING FOR RED
            right(90, MOTOR_POWER_RL);
            forward(12, MOTOR_POWER_FB); // go above half tile so you don't crash into cone on turn
            right(90, MOTOR_POWER_RL);
            forward(43, MOTOR_POWER_FB); // almost 3 tiles
            left(90, MOTOR_POWER_RL);
            forward(30, MOTOR_POWER_FB); // park
            lineardown(12,0.5);


        } else if(color.green() > color.blue())
        //green (Position two)
        {
            telemetry.addData("GREEN", y);
            telemetry.update();

            // CONE SEQUENCE

            forward(26, MOTOR_POWER_FB); // 2 tile length - 21
            left(90, MOTOR_POWER_RL);
            forward(8.75, MOTOR_POWER_FB); // half tile length - medium junction base radius

            // lower cone on medium junction
            lineardown(11.5, LINEAR_POWER);
            linearup(11.5, LINEAR_POWER);

            right(90, MOTOR_POWER_RL);
            forward(11.75, MOTOR_POWER_FB); // half tile length
            right(90, MOTOR_POWER_RL);
            forward(47, MOTOR_POWER_FB); // around 2 tile lengths

            // pick up cone from stacks
            lineardown(22, LINEAR_POWER);
            linearup(10.5, LINEAR_POWER);

            right(90, MOTOR_POWER_RL);
            forward(11.75, MOTOR_POWER_FB); // half tile length
            right(90, MOTOR_POWER_RL);
            forward(23.5, MOTOR_POWER_FB); // one tile length

            // lower cone on low junction
            linearup(3.5, LINEAR_POWER);
            lineardown(3.5 ,LINEAR_POWER);

            // PARKING FOR GREEN
            backward(11.75, MOTOR_POWER_FB); // 1/2 tile length
            left(90, MOTOR_POWER_RL);
            forward(11.75, MOTOR_POWER_FB); // 1/2 tile length
            right(90, MOTOR_POWER_RL);
            forward(23.5, MOTOR_POWER_FB); // 1 tile length

            lineardown(12,0.5);




        } else if(color.blue() > color.green())
        // blue (Position three)
        {
            telemetry.addData("BLUE", z);
            telemetry.update();

            // CONE SEQUENCE

            forward(26, MOTOR_POWER_FB); // 2 tile length - 21
            left(90, MOTOR_POWER_RL);
            forward(8.75, MOTOR_POWER_FB); // half tile length - medium junction base radius

            // lower cone on medium junction
            lineardown(11.5, LINEAR_POWER);
            linearup(11.5, LINEAR_POWER);

            right(90, MOTOR_POWER_RL);
            forward(11.75, MOTOR_POWER_FB); // half tile length
            right(90, MOTOR_POWER_RL);
            forward(47, MOTOR_POWER_FB); // around 2 tile lengths

            // pick up cone from stacks
            lineardown(22, LINEAR_POWER);
            linearup(10.5, LINEAR_POWER);

            right(90, MOTOR_POWER_RL);
            forward(11.75, MOTOR_POWER_FB); // half tile length
            right(90, MOTOR_POWER_RL);
            forward(23.5, MOTOR_POWER_FB); // one tile length

            // lower cone on low junction
            linearup(3.5, LINEAR_POWER);
            lineardown(3.5 ,LINEAR_POWER);

            // PARKING FOR BLUE
            backward(11.75, MOTOR_POWER_FB); // 1/2 tile length

            lineardown(12,0.5);


        }
        else //Fail
        {
            claw.setPosition(0.45);
        }

        BR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        linear.setPower(0);

    }


    private void forward(double inch,  double power)
    {

        //Sets new position for motors
        int a = (int) (FL.getCurrentPosition() + (inch*cpi));
        int b = (int) (FR.getCurrentPosition() + (inch*cpi));
        int c = (int) (BL.getCurrentPosition() + (inch*cpi));
        int d = (int) (BR.getCurrentPosition() + (inch*cpi));

        FL.setTargetPosition(a);
        FR.setTargetPosition(b);
        BL.setTargetPosition(c);
        BR.setTargetPosition(d);

        //Sets desired power for motors
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);

        //Makes the motors to run to the position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Loop to run encoders method
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())
        {
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d : %7d : %7d", a, b, c, d);
            telemetry.addData("Actual", "%7d :%7d : %7d : %7d", FL.getCurrentPosition(), FR.getCurrentPosition(), BL.getCurrentPosition(), BR.getCurrentPosition());
            telemetry.update();
        }

        //Eliminates momentum
        FL.setPower(-0.1);
        FR.setPower(-0.1);
        BL.setPower(-0.1);
        BR.setPower(-0.1);
        sleep(150);

        //Stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void backward(double inch, double power)
    {
        //Sets new position for motors
        int a = (int) (FL.getCurrentPosition() - (inch*cpi));
        int b = (int) (FR.getCurrentPosition() - (inch*cpi));
        int c = (int) (BL.getCurrentPosition() - (inch*cpi));
        int d = (int) (BR.getCurrentPosition() - (inch*cpi));

        FL.setTargetPosition(a);
        FR.setTargetPosition(b);
        BL.setTargetPosition(c);
        BR.setTargetPosition(d);

        //Sets desired power for motors
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        FL.setPower(power);

        //Makes the motors to run to the position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Loop to run encoders method
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())
        {
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d : %7d : %7d", a, b, c, d);
            telemetry.addData("Actual", "%7d :%7d : %7d : %7d", FL.getCurrentPosition(), FR.getCurrentPosition(), BL.getCurrentPosition(), BR.getCurrentPosition());
            telemetry.update();
        }

        //Eliminates momentum
        FL.setPower(0.1);
        FR.setPower(0.1);
        BL.setPower(0.1);
        BR.setPower(0.1);
        sleep(150);

        //Stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void right(double degree, double power) {
        //Sets new position for motors
        FL.setTargetPosition((int) (FL.getCurrentPosition() + (degree * cpd)));
        FR.setTargetPosition((int) (FR.getCurrentPosition() - (degree * cpd)));
        BL.setTargetPosition((int) (BL.getCurrentPosition() + (degree * cpd)));
        BR.setTargetPosition((int) (BR.getCurrentPosition() - (degree * cpd)));

        //Sets desired power for motors
        FL.setPower(power);
        FR.setPower(-power);
        BL.setPower(power);
        BR.setPower(-power);

        //Makes the motors to run to the position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Loop to run encoders method
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {

        }

        //Eliminates momentum
        FL.setPower(-0.1);
        FR.setPower(0.1);
        BL.setPower(-0.1);
        BR.setPower(0.1);
        sleep(150);

        //Stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void left(double degree, double power) {
        //Sets new position for motors
        FL.setTargetPosition((int) (FL.getCurrentPosition() - (degree * cpd)));
        FR.setTargetPosition((int) (FR.getCurrentPosition() + (degree * cpd)));
        BL.setTargetPosition((int) (BL.getCurrentPosition() - (degree * cpd)));
        BR.setTargetPosition((int) (BR.getCurrentPosition() + (degree * cpd)));

        //Sets desired power for motors
        FL.setPower(power);
        FR.setPower(-power);
        BL.setPower(power);
        FL.setPower(-power);

        //Makes the motors to run to the position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Loop to run encoders method
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {

        }

        //Eliminates momentum
        FL.setPower(-0.1);
        FR.setPower(0.1);
        BL.setPower(-0.1);
        BR.setPower(0.1);
        sleep(150);

        //Stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }


    private void linearup(double inch, double power)
    {
        int a = (int) (linear.getCurrentPosition() + (inch*lcpi));
        linear.setTargetPosition(a);
        linear.setPower(power);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (linear.isBusy())
        {
            telemetry.addLine("Linear up");
            telemetry.addData("Target", "%7d", a);
            telemetry.addData("Actual", "%7d", linear.getCurrentPosition());
            telemetry.update();
        }
        //Changes power based on height to stop gravity from pulling down linear slides
        if(inch < 8)
        {
            linear.setPower(0.1);
        }else if (inch >= 8 && inch < 18)
        {
            linear.setPower(0.2);
        }else if(inch >= 18)
        {
            linear.setPower(0.3);
        }
    }
    private void lineardown(double inch, double power)
    {
        int a = (int) (linear.getCurrentPosition() - (inch*lcpi));
        linear.setTargetPosition(a);
        linear.setPower(power);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (linear.isBusy())
        {
            telemetry.addLine("Linear up");
            telemetry.addData("Target", "%7d", a);
            telemetry.addData("Actual", "%7d", linear.getCurrentPosition());
            telemetry.update();
        }
        linear.setPower(0);
    }

    private void robotsleep(int sleep, double power)
    {
        BR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        linear.setPower(0);
    }






}











