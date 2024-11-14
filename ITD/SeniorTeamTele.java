package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp
public class SeniorTeamTele extends OpMode
{
    public DcMotor FR, FL, BR, BL, slides;
    private double powerRY, powerRX, powerLX, powerLY, robotAngle, PowerMultiplier, lf, rb, rf, lb;
    public CRServo Servo;

    @Override
    public void init()
    {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        slides = hardwareMap.get(DcMotor.class, "slides");
        Servo = hardwareMap.get(CRServo.class, "Servo");

        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setDirection(DcMotorSimple.Direction.FORWARD);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        Servo.setPower(0);
        slides.setPower(0);

    }

    @Override
    public void loop()
    {
        powerLX = gamepad1.left_stick_x/2;
        powerLY = gamepad1.left_stick_y/2;
        powerRX = gamepad1.right_stick_x/2;
        powerRY = gamepad1.right_stick_y/2;

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

        // CRServo Test
        telemetry.addData("Servo: ", Servo.getPower());

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

        // Servo Intake
        if (gamepad1.a)
        {
            Servo.setPower(-1.0);

        }
        if (gamepad1.b)
        {
            Servo.setPower(0.5);
        }
        if (gamepad1.x)
        {
            Servo.setPower(0);
        }

        // SLIDES
        if (gamepad2.dpad_up)
        {
            slides.setPower(-1.4);
        }
        if (gamepad2.dpad_down)
        {
            slides.setPower(0.85);
        }



    }
}
