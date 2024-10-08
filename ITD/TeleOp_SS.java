/* never forget the package! this supplies most of your code*/

package org.firstinspires.ftc.teamcode;

// USE FOR 5440


/* all other libraries required, any unused will appear gray*/
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



/* class must match name of file*/
@TeleOp
public class TeleOp_SS extends OpMode {


    private double powerLX, powerLY, powerRX, powerRY, strafePower;
    private DcMotor FL, BL, FR, BR, Arm0, Arm1, Rigging1, Rigging2, Wrist;
    private Servo Launcher, Claw;




    @Override
    public void init() {


        // TODO: TEST CLAW + UPDATE RIGGING

        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        Arm0 = hardwareMap.get(DcMotor.class, "Arm0");
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Rigging1 = hardwareMap.get(DcMotor.class, "Rigging1");
        Rigging2 = hardwareMap.get(DcMotor.class, "Rigging2");
        Wrist = hardwareMap.get(DcMotor.class, "Wrist");
        Launcher = hardwareMap.get(Servo.class, "Launcher");
        //Claw = hardwareMap.get(Servo.class, "Claw");


        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm0.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm1.setDirection(DcMotorSimple.Direction.FORWARD);
        Rigging1.setDirection(DcMotorSimple.Direction.REVERSE);
        Rigging2.setDirection(DcMotorSimple.Direction.REVERSE);
        Wrist.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: FIGURE OUT DIRECTION FOR CLAW


        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        Arm0.setPower(0);
        Arm1.setPower(0);
        Rigging1.setPower(0);
        Rigging2.setPower(0);
        Wrist.setPower(0);
        Launcher.setPosition(0);
        //Claw.setPosition(0);

    }


    @Override
    public void loop() {

        strafePower = 0.85;
        powerLX = gamepad1.left_stick_x/2;
        powerLY = gamepad1.left_stick_y/2;
        powerRX = gamepad1.right_stick_x/2;
        powerRY = gamepad1.right_stick_y/2;

        // DRIVETRAIN
        if(powerLY > 0.05 || powerLY < -0.05)
        {
            FL.setPower(powerLY);
            BL.setPower(powerLY);
        } else
        {
            FL.setPower(0);
            BL.setPower(0);
        }


        if(powerRY > 0.05 || powerRY < -0.05)
        {
            FR.setPower(powerRY);
            BR.setPower(powerRY);
        } else
        {
            FR.setPower(0);
            BR.setPower(0);
        }

        // STRAFING
        if(gamepad1.left_bumper)
        {
            FR.setPower(strafePower);
            FL.setPower(-strafePower);
            BR.setPower(-strafePower);
            BL.setPower(strafePower);
        } else {
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        }

        if(gamepad1.right_bumper)
        {
            FR.setPower(-strafePower);
            FL.setPower(strafePower);
            BR.setPower(strafePower);
            BL.setPower(-strafePower);
        } else {
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        }

        // INTAKE/OUTTAKE

        // going up
        if (gamepad2.right_stick_button)
        {
            Arm0.setPower(0.6);
            Arm1.setPower(0.6);
        }
        // stopping when going up
        if(gamepad2.left_stick_button) {
            Arm0.setPower(-0.2);
            Arm1.setPower(-0.2);
        }
        // stop TOTALLY
        if(gamepad2.x){
            Arm0.setPower(0);
            Arm1.setPower(0);

        }

        // RIGGING
        if (gamepad2.dpad_up)
        {
            Rigging1.setPower(0.15);
            Rigging2.setPower(0.15);
        }
        if (gamepad2.dpad_down)
        {
            Rigging1.setPower(-0.15);
            Rigging2.setPower(-0.15);

        }
        if(gamepad2.dpad_left){
            Rigging1.setPower(0);
            Rigging2.setPower(0);

        }
        //max power
        if(gamepad2.dpad_right){
            Rigging1.setPower(-0.8);
            Rigging2.setPower(-0.8);

        }

        // LAUNCHER
        if (gamepad1.a)
        {
            Launcher.setPosition(0.4);
        }
        // reset
        else if (gamepad1.y)
        {
            Launcher.setPosition(0);
        }

        // WRIST
        if (gamepad2.left_bumper){
            Wrist.setPower(0.4);
        }
        if (gamepad2.right_bumper)
        {
            Wrist.setPower(-0.4);
        }

        // CLAW

        // TODO: FIGURE OUT STUFF FOR CLAW

        // close
        /*
        if (gamepad2.a) {
            Claw.setPosition(0);
        }

        // open claw wide
        if (gamepad2.y) {
            Claw.setPosition(0.25);
        }

        // open claw slightly
        if (gamepad2.b) {
            Claw.setPosition(0.03);
        }
        */



        // TELEMETRY

        telemetry.addData("powerRX", "%.2f", gamepad1.right_stick_x);
        telemetry.addData("powerRY", "%.2f", gamepad1.right_stick_y);
        telemetry.addData("powerLX", "%.2f", gamepad1.left_stick_x);
        telemetry.addData("powerLY", "%.2f", gamepad1.left_stick_y);


        telemetry.addData("FR START: ", FR.getPower());
        telemetry.addData("FL START: ", FL.getPower());
        telemetry.addData("BR START: ", BR.getPower());
        telemetry.addData("BL START: ", BL.getPower());

        telemetry.addData("Arm0 Power: ", Arm0.getPower());
        telemetry.addData("Arm1 Power: ", Arm1.getPower());
        telemetry.addData("Rigging1 Power: ", Rigging1.getPower());
        telemetry.addData("Rigging2 Power: ", Rigging2.getPower());
        telemetry.addData("Launcher Position: ", Launcher.getPosition());
        //telemetry.addData("Claw Position: ", Claw.getPosition());

        telemetry.update();

        // TODO: TEST LOGGING? IF TIME ALLOWS
        // telemetry.log(); -- should create log file
        // telemetry.clear(); -- clears log





    }
}
