package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class SeniorTeamTele extends OpMode
{
    public DcMotor FR, FL, BR, BL, Slides1, Slides2;
    private double powerRY, powerRX, powerLX, powerLY, robotAngle, PowerMultiplier, lf, rb, rf, lb;
    public CRServo Roller;
    public Servo Elbow1, Elbow2, EWrist1, EWrist2, TopWrist;

    @Override
    public void init()
    {
        // Wheels
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        // Slides
        Slides1 = hardwareMap.get(DcMotor.class, "Slides1");
        Slides2 = hardwareMap.get(DcMotor.class, "Slides2");

        // Elbow
        Elbow1 = hardwareMap.get(Servo.class, "Elbow1");
        Elbow2 = hardwareMap.get(Servo.class, "Elbow2");

        // Wrist at Elbow
        // VERIFY
        EWrist1 = hardwareMap.get(Servo.class, "EWrist1");
        EWrist2 = hardwareMap.get(Servo.class, "EWrist2");

        // Wrist at Top
        TopWrist = hardwareMap.get(Servo.class, "TopWrist");

        // Roller
        Roller = hardwareMap.get(CRServo.class, "Roller");

        // Wheels Set-Up
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Slides Set-Up
        Slides1.setDirection(DcMotorSimple.Direction.FORWARD);
        Slides2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set Powers
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        Elbow1.setPosition(0);
        Elbow2.setPosition(0);

        EWrist1.setPosition(0);
        EWrist2.setPosition(0);
        TopWrist.setPosition(0);

        Roller.setPower(0);
    }

    @Override
    public void loop()
    {
        powerLX = gamepad1.left_stick_x/2;
        powerLY = gamepad1.left_stick_y/2;
        powerRX = gamepad1.right_stick_x/2;
        powerRY = gamepad1.right_stick_y/2;

        robotAngle = Math.atan2(powerLX, powerLY);
        
        // All telemetry
        telemetry.addData("Robot angle:", robotAngle);
        telemetry.addData("powerRX: ", gamepad1.right_stick_x);
        telemetry.addData("powerRY: ", gamepad1.right_stick_y);
        telemetry.addData("powerLX: ", gamepad1.left_stick_x);
        telemetry.addData("powerLY: ", gamepad1.left_stick_y);

        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());
        
        telemetry.addData("Slides1: ", Slides1.getPower());
        telemetry.addData("Slides2: ", Slides2.getPower());

        telemetry.addData("Elbow1: ", Elbow1.getPosition());
        telemetry.addData("Elbow2: ", Elbow2.getPosition());
        telemetry.addData("EWrist1: ", EWrist1.getPosition());
        telemetry.addData("EWrist2: ", EWrist2.getPosition());
        telemetry.addData("TopWrist: ", TopWrist.getPosition());
        
        telemetry.addData("Roller: ", Roller.getPower());
        

        telemetry.update();


        PowerMultiplier = Math.sqrt((Math.pow(powerLX, 2) + Math.pow(powerLY, 2)));

        lf = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4)))) - powerRX;
        rb = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4)))) + powerRX;
        lb = (PowerMultiplier*Math.sin(robotAngle+(Math.PI/4))) - powerRX;
        rf = (PowerMultiplier*Math.sin(robotAngle+(Math.PI/4))) + powerRX;

        FR.setPower(rf);
        FL.setPower(lf);
        BR.setPower(rb);
        BL.setPower(lb);

        // Slides to lift bucket
        if (gamepad2.dpad_up)
        {
            // adjust position
            //EWrist1.setPosition();
            //EWrist1.setPosition();

            Slides1.setPower(-1.4);
            Slides2.setPower(-1.4);

        }
        if (gamepad2.dpad_down)
        {
            // adjust position
            //EWrist1.setPosition();
            //EWrist1.setPosition();

            Slides1.setPower(0.85);
            Slides2.setPower(0.85);
        }

        if (gamepad2.a)
        {
            // adjust position
            //TopWrist.setPosition();
        }

        // Roller - take in specimen/sample
        if (gamepad2.x)
        {
            Roller.setPower(0.5);
        }
        if (gamepad2.b)
        {
            Roller.setPower(-1.0);
        }
        if (gamepad2.y)
        {
            Roller.setPower(0);
        }

        // Rigging
        if (gamepad2.left_bumper)
        {
            // stop
            Slides1.setPower(0);
            Slides2.setPower(0);
        }
        if (gamepad2.right_bumper)
        {
            // max power
            Slides1.setPower(1);
            Slides2.setPower(1);
        }




    }
}
