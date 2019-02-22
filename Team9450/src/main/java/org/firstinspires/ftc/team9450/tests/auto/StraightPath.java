package org.firstinspires.ftc.team9450.tests.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.MotionTracker;

@Autonomous(name = "Straight Path", group = "test path")
public class StraightPath extends LinearOpMode {
    private Drivetrain drive;
    private Gyroscope imu;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(hardwareMap);/*new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF),hardwareMap.dcMotor.get(Constants.Drivetrain.LB),
                hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));*/
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));

        waitForStart();
        while (opModeIsActive()) {
            double correction = imu.getAngle()/10;
            drive.setPower(new double[]{1 + correction, 1 + correction, 1 - correction, 1 - correction});
        }
    }
}
