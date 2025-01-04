package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SWristTest extends OpMode
{
    public DcMotor FR, FL, BR, BL, Slides1, Slides2;
    public Servo Elbow1, Elbow2, SClaw, FClaw, SWrist1, SWrist2;
    public Servo EWrist1, Wrist;
    private double powerRY, powerRX, powerLX, powerLY, robotAngle, PowerMultiplier, lf, rb, rf, lb;
    //    double cpi = 50; (original cpi was too short, around 2.5 inches)
    double cpi = 125;

    double prevServo1 = 0;
    double prevServo2 = 0;

    double cpd = 3.7;
    int inch = 0;


    @Override
    public void init()
    {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        // testing slides1 and slides2

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
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        Slides1.setDirection(DcMotor.Direction.REVERSE);
        Slides2.setDirection(DcMotor.Direction.FORWARD);
        SClaw.setDirection(Servo.Direction.FORWARD);
        // set directions for FClaw, SWrist1, SWrist2

        // SWrist1 and SWrist2 must be opposite directions!! - to test
        SWrist1.setDirection(Servo.Direction.FORWARD);
        SWrist2.setDirection(Servo.Direction.REVERSE);

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
        FClaw.setPosition(0);
        Elbow1.setPosition(1);
        Elbow2.setPosition(0);
        // SClaw works with 0.69/0.7
        SClaw.setPosition(0.69);

        // spin 90 degrees upward
//        SWrist1.setPosition(0.65);
//        SWrist2.setPosition(0.65);

        SWrist1.setPosition(1);
        SWrist2.setPosition(1);

        // 0.2, 0 spins all the way around

    }

    @Override
    public void loop() {
    }


}
