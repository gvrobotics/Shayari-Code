package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Tele_Op_SS extends OpMode {
    // Two motors for back wheel drive
    // One motor for intake rod and one outtake motor for intake slides
    public DcMotor BR, BL, Intake, Outtake;

    // Servo for bucket:
    public Servo BUCKET;

    // Wheel power:
    private double power_BR, power_BL;


    @Override
    public void init() {
        // Set hardware maps for all DC motors:
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BUCKET = hardwareMap.get(Servo.class, "BUCKET");

        // Set all known directions for all DC motors:
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Need to test Outtake direction--currently Forward!
        Outtake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Notes to self: counterclockwise (intake)-> positive, clockwise (outtake)-> negative

        // Set all motors' power to 0 and servo position to 0:
        BR.setPower(0);
        BL.setPower(0);
        Intake.setPower(0);
        Outtake.setPower(0);
        BUCKET.setPosition(0);
    }

    @Override
    public void loop() {

        // Drivetrain:

        // Back wheels powers:
        power_BR = gamepad1.right_stick_y;
        power_BL = gamepad1.left_stick_y;


        // Setting dead-zone:
        if (power_BR < -0.2 || power_BR > 0.2)
        {
            BR.setPower(power_BR);
        }

        if (power_BL < -0.2 || power_BL > 0.2)
        {
            BL.setPower(power_BL);
        }

        // Intake:
        double i_power = 0.2;
        if (gamepad1.right_bumper && gamepad1.left_bumper){

            Intake.setPower(i_power);

        }
        else{
            Intake.setPower(0);
        }

        // Outtake:
        // slide
        double o_power = 1;
        // Make slide go up:
        if (gamepad1.right_trigger < -0.1 || gamepad1.right_trigger > 0.1){

            Outtake.setPower(o_power);

        }
        else{
            Outtake.setPower(0);
        }

        // Make slide go down:
        if (gamepad1.left_trigger < -0.1 || gamepad1.right_trigger > 0.1){
            Outtake.setPower(-o_power);
        }
        else{
            Outtake.setPower(0);
        }

        // Servo code:
        if (gamepad1.a){
            BUCKET.setPosition(1);
        }
        else{
            BUCKET.setPosition(0);
        }

        if (gamepad1.b){
            BUCKET.setPosition(0.5);
        }
        else{
            BUCKET.setPosition(0);
        }

        if (gamepad1.x){
            BUCKET.setPosition(-0.5);
        }
        else{
            BUCKET.setPosition(0);
        }

        if (gamepad1.y){
            BUCKET.setPosition(-1);
        }
        else{
            BUCKET.setPosition(0);
        }

    }






}

