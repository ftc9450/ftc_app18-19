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

@Autonomous(name = "Straight Path", group = "Auto")
public class StraightPath extends LinearOpMode {
    private Drivetrain drive;
    private Gyroscope imu;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(hardwareMap);
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        double TARGET = 1440;
        double error = TARGET;
        double curvewidth = error/6;
        double correction = imu.getAngle()/100;
        double power;
        double OFFSET;
        Gaussian gauss = new Gaussian(TARGET/2.0, TARGET/6.0);

        waitForStart();
        while (opModeIsActive() && error > 0) {
            error = TARGET - drive.getPosition();
            power = 0.7;//1.0 / (1.0 + Math.exp(-error/(TARGET) + 4));
            OFFSET = 0.3; //TODO: @Tangerine tweak this value to get robot to start moving
            //power = 0.6*2.0/(0.5*Math.sqrt(2*Math.PI)) * Math.exp(-((error*3.0/TARGET)-1.5)*((error*3.0/TARGET)-1.5)/(2*0.25)) + OFFSET;
            //power = 0.3;//0.7 / (1.0 + Math.exp(-error/(TARGET) + 4));
            //power = ((100000/(curvewidth*Math.sqrt(2*Math.PI))) * Math.exp(-0.5*error-(TARGET/1)/curvewidth))+0.1;
            power = gauss.value(error) + 0.05;
            correction = imu.getAngle()/100;
            drive.setPower(new double[]{power + correction, power + correction, power - correction, power - correction});
            telemetry.addData("power", power);
            telemetry.addData("error", error);
            telemetry.addData("correction", correction);
            telemetry.update();
        }
        drive.setPower(new double[]{0,0,0,0});
        while(opModeIsActive()){}
    }
}
