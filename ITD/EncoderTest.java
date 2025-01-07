/ * AI GENERATED - CHECKS ENCODER DIRECTION */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {

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

        // Test each motor individually
        testEncoderDirection(FR, "FR");
        testEncoderDirection(FL, "FL");
        testEncoderDirection(BR, "BR");
        testEncoderDirection(BL, "BL");
    }

    private void testEncoderDirection(DcMotor motor, String motorName) {
        telemetry.addLine("Testing " + motorName);
        telemetry.update();

        // Get initial encoder value
        int initialValue = motor.getCurrentPosition();

        // Move the motor for a short time
        motor.setPower(0.3);
        sleep(1000); // Run motor for 1 second
        motor.setPower(0);

        // Get final encoder value
        int finalValue = motor.getCurrentPosition();

        // Compare the initial and final values
        if (finalValue > initialValue) {
            telemetry.addLine("Encoder direction for " + motorName + " is correct.");
        } else {
            telemetry.addLine("Encoder direction for " + motorName + " is incorrect.");
        }

        telemetry.update();
        sleep(500); // Pause between tests
    }
}
