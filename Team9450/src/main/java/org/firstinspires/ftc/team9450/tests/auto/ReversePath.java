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

import java.lang.annotation.Target;

@Autonomous(name = "Reverse Path", group = "Auto")
public class ReversePath extends LinearOpMode {
    private Drivetrain drive;
    private Gyroscope imu;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(hardwareMap);
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        double TARGET = -6000;
        double errorCorrect= 1174*(Math.pow(0.9997, TARGET));
        double error = TARGET;
        double correction = imu.getAngle()/100;
        double power;
        Gaussian gauss = new Gaussian(TARGET/2.0, TARGET/6.0);

        waitForStart();
        if(TARGET > errorCorrect){
            TARGET-=errorCorrect;
            error=TARGET;
            while (opModeIsActive() && error > 0) {
                error = TARGET - drive.getPosition();
                power = -(500*gauss.value(error)) + 0.15;
                correction = imu.getAngle()/100;
                drive.setPower(new double[]{power + correction, power + correction, power - correction, power - correction});
                telemetry.addData("power", power);
                telemetry.addData("error", error);
                telemetry.addData("correction", correction);
                telemetry.update();
            }
        }else{
            while(opModeIsActive() && error > 0){
                error = TARGET - drive.getPosition();
                power = -0.2;
                correction = imu.getAngle()/100;
                drive.setPower(new double[]{power + correction, power + correction, power - correction, power - correction});
                telemetry.addData("power", power);
                telemetry.addData("error", error);
                telemetry.addData("correction", correction);
                telemetry.update();
            }
        }
        drive.setPower(new double[]{0,0,0,0});
        while(opModeIsActive()){
            error = TARGET - drive.getPosition();
            telemetry.addData("error", error);
            telemetry.update();
        }
    }
}
