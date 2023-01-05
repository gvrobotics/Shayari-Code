package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutonRight extends LinearOpMode
{
    public DcMotor BR, BL, FR, FL,linear;
    public Servo claw;
    public ColorSensor color;

    //clicks per degree
    double cpd = 3;

    //clicks per inch
    double cpi = 7.5;

    //clicks per inch for linear slides since different diameter than wheels and different motor
    double lcpi = 114;

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
        linear.setDirection(DcMotorSimple.Direction.REVERSE);

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

        linearup(12,0.6);
        sleep(1000);
        forward(21,0.1);
        sleep(2000);
        int x = 1;
        int y = 2;
        int z = 3;

        final double MOTOR_POWER = 0.1;
        final double LINEAR_POWER = 0.25;


        // parking
        if (color.blue()-100 < color.red() && color.blue()+100 > color.red())
        //red (Position one)
        {
            telemetry.addData("RED", x);
            telemetry.update();

            // CONE SEQUENCE

            forward(26, MOTOR_POWER); // 2 tile lengths - 21 = 26
            left(90, MOTOR_POWER);
            forward(8.75, MOTOR_POWER); // 1/2 tile length - 3 (junction radius) = 35/4

            // 12 inches up so far
            // lower preloaded cone on MEDIUM junction
            linearup(11.5, LINEAR_POWER);
            lineardown(11.5, LINEAR_POWER);

            right(90, MOTOR_POWER);
            forward(11.75, MOTOR_POWER); // 1/2 tile length
            right(90, MOTOR_POWER);
            forward(44, MOTOR_POWER); // 2 tile lengths - low junction base radius

            // pick up 2nd cone from stacks
            linearup(22, LINEAR_POWER);
            lineardown(10.5, LINEAR_POWER);

            right(90, LINEAR_POWER);
            forward(11.75, MOTOR_POWER); // 1/2 tile length
            right(90, LINEAR_POWER);
            forward(20.5, MOTOR_POWER); // 1 tile length - radius of ground junction =

            // lower 2nd cone one LOW junction
            linearup(24, 0.5);

            // PARKING FOR RED
            backward(11.75, MOTOR_POWER); // 1/2 tile length
            right(90, LINEAR_POWER);
            forward(11.75, MOTOR_POWER); // 1/2 tile length
            left(90, LINEAR_POWER);
            forward(35.23, MOTOR_POWER); // 1 1/2 tile length


            lineardown(12,0.5);




        } else if(color.green() > color.blue())
        //green (Position two)
        {
            telemetry.addData("GREEN", y);
            telemetry.update();

            // CONE SEQUENCE

            forward(26, MOTOR_POWER); // 2 tile lengths - 21 = 26
            left(90, MOTOR_POWER);
            forward(8.75, MOTOR_POWER); // 1/2 tile length - 3 (junction radius) = 35/4

            // 12 inches up so far
            // lower preloaded cone on MEDIUM junction
            linearup(11.5, LINEAR_POWER);
            lineardown(11.5, LINEAR_POWER);

            right(90, MOTOR_POWER);
            forward(11.75, MOTOR_POWER); // 1/2 tile length
            right(90, MOTOR_POWER);
            forward(44, MOTOR_POWER); // 2 tile lengths - low junction base radius

            // pick up 2nd cone from stacks
            linearup(22, LINEAR_POWER);
            lineardown(10.5, LINEAR_POWER);

            right(90, LINEAR_POWER);
            forward(11.75, MOTOR_POWER); // 1/2 tile length
            right(90, LINEAR_POWER);
            forward(20.5, MOTOR_POWER); // 1 tile length - radius of ground junction =

            // lower 2nd cone one LOW junction
            linearup(24, 0.5);


            // PARKING FOR GREEN
            backward(11.75, MOTOR_POWER); // 1/2 tile length
            left(90, LINEAR_POWER);
            forward(11.75, MOTOR_POWER); // 1/2 tile length
            right(90, LINEAR_POWER);
            forward(11.75, MOTOR_POWER); // 1/2 tile length


        } else if(color.blue() > color.green())
        // blue (Position three)
        {
            telemetry.addData("BLUE", z);
            telemetry.update();

            // CONE SEQUENCE

            forward(26, MOTOR_POWER); // 2 tile lengths - 21 = 26
            left(90, MOTOR_POWER);
            forward(8.75, MOTOR_POWER); // 1/2 tile length - 3 (junction radius) = 35/4

            // 12 inches up so far
            // lower preloaded cone on MEDIUM junction
            linearup(11.5, LINEAR_POWER);
            lineardown(11.5, LINEAR_POWER);

            right(90, MOTOR_POWER);
            forward(11.75, MOTOR_POWER); // 1/2 tile length
            right(90, MOTOR_POWER);
            forward(44, MOTOR_POWER); // 2 tile lengths - low junction base radius

            // pick up 2nd cone from stacks
            linearup(22, LINEAR_POWER);
            lineardown(10.5, LINEAR_POWER);

            right(90, LINEAR_POWER);
            forward(11.75, MOTOR_POWER); // 1/2 tile length
            right(90, LINEAR_POWER);
            forward(20.5, MOTOR_POWER); // 1 tile length - radius of ground junction =

            // lower 2nd cone one LOW junction
            linearup(24, 0.5);


            // PARKING FOR BLUE
            backward(11.75, MOTOR_POWER); // 1/2 tile length


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













