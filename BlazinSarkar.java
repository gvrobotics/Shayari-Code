package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class BlazinSarkar extends OpMode {
    private double powerLX, powerLY, powerRX, powerRY;
    private DcMotor FL, BL, FR, BR;
    public Servo claw;
    public DcMotor linear;

    @Override
    public void init() {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        linear = hardwareMap.get(DcMotor.class, "L");
        claw = hardwareMap.get(Servo.class, "c");
        linear.setDirection(DcMotorSimple.Direction.REVERSE);

        linear.setPower(0);
        claw.setPosition(0);

    }

    @Override
    public void loop() {
        powerLX = gamepad1.left_stick_x/2;
        powerLY = gamepad1.left_stick_y/2;
        powerRX = gamepad1.right_stick_x/2;
        powerRY = gamepad1.right_stick_y/2;

        if (gamepad1.right_bumper || gamepad1.left_bumper)
        {
            powerLX = gamepad1.left_stick_x/3;
            powerLY = gamepad1.left_stick_y/3;
            powerRX = gamepad1.right_stick_x/3;
            powerRY = gamepad1.right_stick_y/3;
        }

        if (powerLY > 0.07 || powerLY < -0.07) {
            FL.setPower(powerLY);
            BL.setPower(powerLY);
        }
        else {
            FL.setPower(0);
            BL.setPower(0);
        }

        if (powerRY > 0.07 || powerRY < -0.07) {
            FR.setPower(powerRY);
            BR.setPower(powerRY);
        }
        else {
            FR.setPower(0);
            BR.setPower(0);
        }

        if (gamepad2.dpad_up) {
            linear.setPower(0.5);
        }
        else {
            linear.setPower(0);
        }

        if (gamepad2.dpad_down) {
            linear.setPower(-0.5);
        }
        else {
            linear.setPower(0);
        }

        if (gamepad2.a) {
            claw.setPosition(0.45);
        }
        else if (gamepad2.y) {
            claw.setPosition(0);
        }

        if (gamepad1.left_trigger>0) {
            BR.setPower(gamepad1.left_trigger);
            BL.setPower(-gamepad1.left_trigger);
            FR.setPower(-gamepad1.left_trigger);
            FL.setPower(gamepad1.left_trigger);
        }

        if (gamepad1.right_trigger>0) {
            BR.setPower(-gamepad1.right_trigger);
            BL.setPower(gamepad1.right_trigger);
            FR.setPower(gamepad1.right_trigger);
            FL.setPower(-gamepad1.right_trigger);
        }
    }
}
