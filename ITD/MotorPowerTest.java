// AI-GENERATED: gets motor power

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "MotorPowerTest")
public class MotorPowerTest extends LinearOpMode {

    public DcMotor FR, FL, BR, BL;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware map
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        // Set motor directions
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        // Set power to all motors
        FR.setPower(0.5);
        FL.setPower(0.5);
        BR.setPower(0.5);
        BL.setPower(0.5);

        // Display motor powers on telemetry
        telemetry.addData("FR Power", FR.getPower());
        telemetry.addData("FL Power", FL.getPower());
        telemetry.addData("BR Power", BR.getPower());
        telemetry.addData("BL Power", BL.getPower());
        telemetry.update();

        // Wait for a short time
        sleep(1000);

        // Stop all motors
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }
}
