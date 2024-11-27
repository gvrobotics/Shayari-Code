package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class SeniorTeamTele extends OpMode
{
    public DcMotor FR, FL, BR, BL, Slides1, Slides2;
    public Servo Elbow1, Elbow2;
    public Servo TopWrist, EWrist1, Wrist;
    public CRServo Roller;
    private double powerRY, powerRX, powerLX, powerLY, robotAngle, PowerMultiplier, lf, rb, rf, lb;

    double cpd = 3.7;

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
        Roller = hardwareMap.get(CRServo.class, "Roller");
        TopWrist = hardwareMap.get(Servo.class, "TopWrist");
        // EWrist1 = hardwareMap.get(Servo.class, "EWrist1");
        Wrist = hardwareMap.get(Servo.class, "Wrist");

        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);


        Slides1.setDirection(DcMotorSimple.Direction.FORWARD);
        Slides2.setDirection(DcMotorSimple.Direction.FORWARD);

        //test, may remove
        TopWrist.setDirection(Servo.Direction.REVERSE);


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
        //EWrist1.setPosition(0.7);
        Wrist.setPosition(0);
        Roller.setPower(0);
        TopWrist.setPosition(-1);
        Elbow1.setPosition(1);
        Elbow2.setPosition(0);

        //elbow1 and elbow2 are continous servos
        //left servo is 0 and right servo is -0.5
    }

    @Override
    public void loop()
    {
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

        FR.setPower(rf);
        FL.setPower(lf);

        BR.setPower(rb);
        BL.setPower(lb);

        // EWrist1 (right) - port 0 of control hub
        // EWrist2 (left) - port 1 of expansion

        if(gamepad2.y){
            Wrist.setPosition(1);
            // EWrist1.setPosition(0);
            /*Slides1.setPower(-1.4);
            Slides2.setPower(-1.4); */
        }
        if(gamepad2.a){
            Wrist.setPosition(0.35);
            //TopWrist.setPosition(initial_position);
            //Slides1.setPower(0.85);
            //Slides2.setPower(0.85);
            //EWrist1.setPosition(0.7);


        }
        if(gamepad2.dpad_left){
            Elbow1.setPosition(1);
            Elbow2.setPosition(0);
        }
        if(gamepad2.dpad_right){
            Elbow1.setPosition(0);
            Elbow2.setPosition(1);
        }

      /*  if(gamepad2.dpad_up){
            Slides1.setPower(-0.3);
            Slides2.setPower(0.3);
        } */

        if(gamepad2.b){
            TopWrist.setPosition(TopWrist.getPosition() + 0.08);
        }
        if(gamepad2.x){
            TopWrist.setPosition(TopWrist.getPosition() - 0.08);
        }

//        if(gamepad2.x){
//            Roller.setPower(0.5);
//        }
//        if(gamepad2.y){
//            Roller.setPower(0);
//        }
//        if(gamepad2.b){
//            Roller.setPower(-1);
//        }

        if (gamepad2.left_trigger > 0.5)
        {
            Roller.setPower(0.5);

        }
        else if (gamepad2.right_trigger > 0.5)
        {
            Roller.setPower(-1);
        }
        else {
            Roller.setPower(0);
        }


        if(gamepad1.left_bumper){
            Slides1.setPower(0);
            Slides2.setPower(0);
        }
//        if(gamepad1.right_bumper){
//            Slides1.setPower(0.9);
//            Slides2.setPower(0.9);
//        }

        if(gamepad2.dpad_down){
            Slides1.setPower(-0.55);
            Slides2.setPower(0.55);
        }

        if(gamepad2.dpad_up){
        Slides1.setPower(0.55);
            Slides2.setPower(-0.55);
        }


        telemetry.addData("Robot angle:", robotAngle);
        telemetry.addData("powerRX: ", gamepad1.right_stick_x);
        telemetry.addData("powerRY: ", gamepad1.right_stick_y);
        telemetry.addData("powerLX: ", gamepad1.left_stick_x);
        telemetry.addData("powerLY: ", gamepad1.left_stick_y);


        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());
        telemetry.addData("Roller: ", Roller.getPower());

        telemetry.addData("Slides1: ", Slides1.getPower());
        telemetry.addData("Slides2: ", Slides2.getPower());


        telemetry.addData("Elbow1: ", Elbow1.getPosition());
        telemetry.addData("Elbow2: ", Elbow2.getPosition());
        // telemetry.addData("EWrist1: ", EWrist1.getPosition());
        telemetry.addData("Wrist: ", Wrist.getPosition());
        telemetry.addData("TopWrist: ", TopWrist.getPosition());
        telemetry.addData("TopWrist: ", TopWrist.getDirection());





        telemetry.update();





    }
}
