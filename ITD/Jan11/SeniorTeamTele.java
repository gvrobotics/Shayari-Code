package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SeniorTeamTele extends OpMode
{
    public DcMotor FR, FL, BR, BL, Slides1, Slides2;
    public Servo Elbow1, Elbow2, SClaw, FClaw, SWrist1, SWrist2;
    public Servo Wrist;
    private double powerRY, powerRX, powerLX, powerLY, robotAngle, PowerMultiplier, lf, rb, rf, lb;

    // double cpi = 50; (this cpi was too short, around 2.5 inches- keeping as backup tho!)
    double cpi = 125;

    double cpd = 3.7;
    int inch = 0;


    // Test SWrist1 + SWrist2 on SWristTest
    @Override
    public void init()
    {
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


        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        // note for FL: wires should be flipped (black-red, red-black)
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        Slides1.setDirection(DcMotor.Direction.REVERSE);
        Slides2.setDirection(DcMotor.Direction.FORWARD);
        SClaw.setDirection(Servo.Direction.FORWARD);
        // set directions for FClaw, SWrist1, SWrist2

        // SWrist1 and SWrist2 must be opposite directions!! - to test
//        SWrist1.setDirection(Servo.Direction.FORWARD);
//        SWrist2.setDirection(Servo.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        Slides1.setPower(0);
        Slides2.setPower(0);
        Wrist.setPosition(0.3);
        FClaw.setPosition(0.7);
        // SClaw works with 0.69/0.7
        SClaw.setPosition(0.69);
        Elbow1.setPosition(0.9);
        Elbow2.setPosition(0);


//        SWrist1.setPosition(0);
//        SWrist2.setPosition(-0);

        // SWrist1, SWrist2 need to be initialized
    }

    @Override
    public void loop() {

        powerLX = gamepad1.left_stick_x;
        powerLY = gamepad1.left_stick_y;
        powerRX = gamepad1.right_stick_x;
        powerRY = gamepad1.right_stick_y;

        robotAngle = Math.atan2(powerLX, powerLY);
        telemetry.addData("Robot angle:", robotAngle);
        telemetry.addData("powerRX: ", gamepad1.right_stick_x);
        telemetry.addData("powerRY: ", gamepad1.right_stick_y);
        telemetry.addData("powerLX: ", gamepad1.left_stick_x);
        telemetry.addData("powerLY: ", gamepad1.left_stick_y);

        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());
        telemetry.update();


        PowerMultiplier = Math.sqrt((Math.pow(powerLX, 2) + Math.pow(powerLY, 2)));

        lf = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4)))) - powerRX;
        rb = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4)))) + powerRX;
        lb = (PowerMultiplier*Math.sin(robotAngle+(Math.PI/4))) - powerRX;
        rf = (PowerMultiplier*Math.sin(robotAngle+(Math.PI/4))) + powerRX;

        // drivetrain
        FR.setPower(rf);
        FL.setPower(lf);
        BR.setPower(rb);
        BL.setPower(lb);



        // kills on both gamepads for slides
        if(gamepad2.left_bumper){

            Slides1.setPower(0);
            Slides2.setPower(0);

        }

        if (gamepad2.right_bumper)
        {
            Slides1.setPower(0);
            Slides2.setPower(0);
        }

        // slides up
        if(gamepad2.dpad_up){

            // doesn't go exactly 5 inches - try adjusting cpi + encoder wire disconnected

            linearup(5, 0.3);

            // backup code if encoder malfunctions
            // Slides1.setPower(0.3);
            // Slides2.setPower(0.3);

        }


        // press down once or else battery disconnects
        // slides down
        if(gamepad2.dpad_down){

            lineardown(4, 0.1);

            // backup code if encoder malfunctions
            // Slides1.setPower(-0.1);
            // Slides2.setPower(-0.1);
        }

        // FClaw disconnects easily, may revise code

        // pull IN elbow + wrist for intake + close claw
        if(gamepad2.dpad_left){
            FClaw.setPosition(0);
            Elbow1.setPosition(0.9);
            Elbow2.setPosition(0);
            Wrist.setPosition(0.35);
        }

        // pull OUT elbow + wrist for intake + open claw
        // open claw spits out block so revise
        if(gamepad2.dpad_right){
            FClaw.setPosition(0.7);
            Elbow1.setPosition(0.7);
            Elbow2.setPosition(0.3);
            Wrist.setPosition(0.95);
        }

        // open SClaw
        if (gamepad2.x)
        {
           SClaw.setPosition(0.8);
        }

        // close SClaw
        if (gamepad2.y)
        {
            SClaw.setPosition(0.69);
        }

        telemetry.addData("FR Power: ", FR.getPower());
        telemetry.addData("FL Power: ", FL.getPower());
        telemetry.addData("BR Power: ", BR.getPower());
        telemetry.addData("BL Power: ", BL.getPower());
        telemetry.addData("FR Position: ", FR.getCurrentPosition());
        telemetry.addData("FL Position: ", FL.getCurrentPosition());
        telemetry.addData("BR Position: ", BR.getCurrentPosition());
        telemetry.addData("BL Position: ", BL.getCurrentPosition());

        telemetry.addData("Slides1 Power: ", Slides1.getPower());
        telemetry.addData("Slides2 Power: ", Slides2.getPower());
        telemetry.addData("Slides1 Position: ", Slides1.getCurrentPosition());
        telemetry.addData("Slides2 Position : ", Slides2.getCurrentPosition());

        telemetry.addData("SClaw Current Position: ", SClaw.getPosition());
        telemetry.addData("Wrist: ", Wrist.getPosition());

        telemetry.addData("Elbow1: ", Elbow1.getPosition());
        telemetry.addData("Elbow2: ", Elbow2.getPosition());

        telemetry.addData("FClaw: ", FClaw.getPosition());


        telemetry.update();

    }
    // see if actual meeting target
    // write some code so it stays up n not slide down (if possible)
    private void linearup(double inch, double power)
    {
        Slides1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int a = (int) ((cpi*inch) + Slides1.getCurrentPosition());
        int b = (int) ((cpi*inch) + Slides2.getCurrentPosition());
        Slides1.setTargetPosition(a);
        Slides2.setTargetPosition(b);
        Slides1.setPower(power);
        Slides2.setPower(power);
        Slides1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slides2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Slides2.isBusy() && Slides1.isBusy())
        {
            telemetry.addLine("Linear up");
            telemetry.addData("Target Slides1", "%7d", a);
            telemetry.addData("Target Slides2", "%7d", b);
            telemetry.addData("Actual Slides1", "%7d", Slides1.getCurrentPosition());
            telemetry.addData("Actual Slides2", "%7d", Slides2.getCurrentPosition());
            telemetry.update();
        }

        Slides1.setPower(0);
        Slides2.setPower(0);
    }

    private void lineardown(double inch, double power)
    {
        Slides1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int a = (int) (Slides1.getCurrentPosition() - (cpi*inch));
        int b = (int) (Slides2.getCurrentPosition() - (cpi*inch));
        Slides1.setTargetPosition(a);
        Slides2.setTargetPosition(b);
        Slides1.setPower(power);
        Slides2.setPower(power);
        Slides1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slides2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Slides2.isBusy() && Slides1.isBusy())
        {
            telemetry.addLine("Linear up");
            telemetry.addData("Target Slides1", "%7d", a);
            telemetry.addData("Target Slides2", "%7d", b);
            telemetry.addData("Actual Slides1", "%7d", Slides1.getCurrentPosition());
            telemetry.addData("Actual Slides2", "%7d", Slides2.getCurrentPosition());
            telemetry.update();
        }

        Slides1.setPower(0);
        Slides2.setPower(0);

    }




}