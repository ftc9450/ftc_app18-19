package org.firstinspires.ftc.teamcode.tests;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MotionTracker;

@Autonomous
public class AutoPivotTest extends LinearOpMode {

    private Drivetrain drivetrain;
    private Gyroscope imu;
    private MotionTracker tracker;
    private Climber climb;
    private GoldAlignDetector detector;
    private final float initialAngle = -45;

    public void runOpMode() throws InterruptedException{
        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF), hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));

        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain, imu, initialAngle); //TODO: check angle

        climb = new Climber(hardwareMap.dcMotor.get(Constants.Climber.CL));
        drivetrain.enableAndResetEncoders();

        pivotClockwise(90);
        //pivotCounterclockwise(90);
        //pivotTo(180);

    }

    public void pivotClockwise(double angle){
        drivetrain.setState(Drivetrain.DrivetrainState.Turning);
        float startAngle = tracker.getAbsoluteAngle();
        double error = angle - (tracker.getAbsoluteAngle() - startAngle + 360) % 360;
        while(error > Constants.Auto.PIVOT_THRESHOLD) {
            error = angle - (tracker.getAbsoluteAngle() - startAngle + 360) % 360;
            double power = Constants.Auto.MAX_PIVOT_POWER * (error/180) + Constants.Auto.MIN_PIVOT_POWER;
            double[] driveSignal = new double[]{power, power, -power, -power};
            drivetrain.setPower(driveSignal);
        }
        drivetrain.setPower(0);
        tracker.updatePosition();
    }
    public void pivotCounterclockwise(double angle){
        drivetrain.setState(Drivetrain.DrivetrainState.Turning);
        float startAngle = tracker.getAbsoluteAngle();
        double error = angle - (startAngle - tracker.getAbsoluteAngle() + 360) % 360;
        while(error > Constants.Auto.PIVOT_THRESHOLD) {
            error = angle - (tracker.getAbsoluteAngle() - startAngle + 360) % 360;
            double power = Constants.Auto.MAX_PIVOT_POWER * (error/180) + Constants.Auto.MIN_PIVOT_POWER;
            double[] driveSignal = new double[]{-power, -power, power, power};
            drivetrain.setPower(driveSignal);
        }
        drivetrain.setPower(0);
        tracker.updatePosition();
    }
    public void pivotTo(float targetAngle){ // relative to field boundaries
        float currentAngle = tracker.getAbsoluteAngle();
        if(Math.abs(currentAngle - targetAngle) > 180){ // must cross the line theta = 0
            if(currentAngle > targetAngle){
                pivotClockwise(360 - (currentAngle - targetAngle));            } else if(targetAngle > currentAngle){
                pivotCounterclockwise(360 - (targetAngle - currentAngle));
            }
        } else{
            if(currentAngle > targetAngle){
                pivotCounterclockwise(currentAngle - targetAngle);
            } else if(targetAngle > currentAngle){
                pivotClockwise(targetAngle - currentAngle);
            }
        }
    }


}
