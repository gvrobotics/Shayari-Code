// AI-GENERATED: gets current heading from IMU

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "IMU_Test")
public class IMU_Test extends LinearOpMode {

    private IMU imu;
    private Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Wait for start signal
        waitForStart();

        while (opModeIsActive()) {
            // Get current heading
            angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle;

            // Display heading on telemetry
            telemetry.addData("Heading", heading);
            telemetry.update();

            // Wait for a short time
            sleep(100);
        }
    }
}
