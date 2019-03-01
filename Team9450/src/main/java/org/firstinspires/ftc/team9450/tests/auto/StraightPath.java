package org.firstinspires.ftc.team9450.tests.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.analysis.function.Gaussian;
import org.apache.commons.math3.special.Erf;
import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.MotionTracker;

@Autonomous(name = "Curved Path", group = "Auto")
public class StraightPath extends LinearOpMode {
    private Drivetrain drive;
    private Gyroscope imu;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(hardwareMap);
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        double TARGET = 1440*3;
        //TARGET -= 560-(30*(TARGET/1440));
        double error = TARGET;
        double curvewidth = error/6;
        double correction = imu.getAngle()/100;
        double power;
        double OFFSET;
        Gaussian gauss = new Gaussian(TARGET/2.0, TARGET/6.0);

        waitForStart();
        while (opModeIsActive() && error > 0) {
            error = TARGET - drive.getPosition();
            power = (500*gauss.value(error)) + 0.15;
            correction = imu.getAngle()/100;
            drive.setPower(new double[]{power + correction, power + correction, power - correction, power - correction});
            telemetry.addData("power", power);
            telemetry.addData("error", error);
            telemetry.addData("correction", correction);
            telemetry.update();
        }
        drive.setPower(new double[]{0,0,0,0});
        while(opModeIsActive()){
            error = TARGET - drive.getPosition();
            telemetry.addData("error", error);
        }
    }
}
