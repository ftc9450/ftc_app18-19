package org.firstinspires.ftc.team9450.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team9450.sensors.GoldDetector;
import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.util.Constants;

@TeleOp
public class FollowCube extends OpMode {
    private Drivetrain drive;
    private Gyroscope imu;
    private GoldDetector gold;
    private int connection;
    private int signal;
    private double angle;

    @Override
    public void init() {
        drive = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF), hardwareMap.dcMotor.get(Constants.Drivetrain.LB),
                hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        gold = new GoldDetector(tfodMonitorViewId);
        connection = -1;
    }

    @Override
    public void loop() {
        signal = gold.process();
        if (signal != -1) connection = signal;
        else drive.setPower(new double[]{0, 0, 0, 0});
        if (connection > 0) {
            angle = gold.getAngle();
            telemetry.addData("angle", angle);
            drive.setPower(new double[]{angle/45, angle/45, -angle/45, -angle/45});
        }
    }
}
