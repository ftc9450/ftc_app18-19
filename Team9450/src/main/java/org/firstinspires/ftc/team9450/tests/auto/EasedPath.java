package org.firstinspires.ftc.team9450.tests.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.analysis.function.Gaussian;
import org.apache.commons.math3.special.Erf;
import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.util.Bezier;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.MotionTracker;

@Autonomous(name = "Eased Path", group = "Auto")
public class EasedPath extends LinearOpMode {
    private Drivetrain drive;
    private Gyroscope imu;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(hardwareMap);
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        Bezier bezier = new Bezier(0, 0, 0, 2000, 0, 2000, 1000, 2000);
        double TARGET = bezier.length();
        //TARGET -= 560-(30*(TARGET/1440));
        double error = TARGET;
        double correction = imu.getAngle()/1000;
        double power;
        double derivX, derivY;
        Gaussian gauss = new Gaussian(TARGET/2.0, TARGET/6.0);

        waitForStart();
        while (opModeIsActive() && error > 0) {
            error = TARGET - drive.getPosition();
            power = 0.4;//(200*gauss.value(error)) + 0.2;
            derivX = bezier.derivativeX(drive.getPosition()/TARGET);
            derivY = bezier.derivativeY(drive.getPosition()/TARGET);
            double angle = 0.0;
            if(derivX == 0){
                if(derivY > 0){
                    angle = 90.0;
                } else{
                    angle = 270.0;
                }
            } else{
                angle = Math.toDegrees(Math.atan(derivY/derivX));
            }
            correction = (imu.getAngle() - angle)/1000;
            drive.setPower(new double[]{power + correction, power + correction, power - correction, power - correction});
            telemetry.addData("power", power);
            telemetry.addData("max power", power + correction);
            telemetry.addData("error", error);
            telemetry.addData("correction", correction);
            telemetry.update();
        }
        telemetry.addLine("done");
        drive.setPower(new double[]{0,0,0,0});
        while(opModeIsActive()){
            error = TARGET - drive.getPosition();
            telemetry.addData("error", error);
        }
    }
}