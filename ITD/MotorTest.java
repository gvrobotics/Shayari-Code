// AI-GENERATED: tests motors

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "MotorTest")
public class MotorTest extends LinearOpMode {

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
        testMotor(FR, "FR");
        testMotor(FL, "FL");
        testMotor(BR, "BR");
        testMotor(BL, "BL");
    }

    private void testMotor(DcMotor motor, String motorName) {
        telemetry.addLine("Testing " + motorName);
        telemetry.update();
        motor.setPower(0.3);
        sleep(1000); // Run motor for 1 second
        motor.setPower(0);
        telemetry.addLine("Test complete for " + motorName);
        telemetry.update();
        sleep(500); // Pause between tests
    }
}
