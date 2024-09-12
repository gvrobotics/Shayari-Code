package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class EncTest extends LinearOpMode
{
    DcMotor motor;
    private double cpi;

    @Override
    public void runOpMode()
    {
        motor = hardwareMap.get(DcMotor.class, "Motor");
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("ENCODER VALUE: ", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}



// forward [? inches]
// phone prints out clicks/inches
