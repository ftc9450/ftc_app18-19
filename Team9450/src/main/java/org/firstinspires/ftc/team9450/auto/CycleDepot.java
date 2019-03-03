package org.firstinspires.ftc.team9450.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Climber;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.subsystems.Intake;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.MotionTracker;


/**
 * Created by Avi on 12/31/2018.
 */
@Autonomous
public class CycleDepot extends LinearOpMode {

    private Drivetrain drivetrain;
    private Gyroscope imu;
    private MotionTracker tracker;
    private Climber climb;
    private Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF), hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));

        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        //intake = new Intake();
        climb = new Climber(hardwareMap);
        drivetrain.enableAndResetEncoders();
        double power = 0.05;

        waitForStart();
        climb.land();

        drivetrain.moveFB(4);
        pivot(-90);
        drivetrain.moveFB(15);
        pivot(-100);
        drivetrain.moveFB(-40);
        pivot(55);
        drivetrain.moveFB(50);
        drivetrain.moveFB(-55);
        pivot(-45);
        drivetrain.moveFB(50);

        int mineralPosition = 1; // placeholder // TODO: get position of gold sample (0, 1, 2) -> (left, center, right)
        switch (mineralPosition) {
            //pivot to face gold mineral
            case 0:
                pivot(135);
                //intake in
                pivot(-55);
                //elevator up and dump

                break;
            case 1:
                pivot(90);
                //intake out
                //intake in
                pivot(-10);
                //elevator up and dump
                break;
            case 2:
                pivot(45);
                //intake out
                //intake in
                pivot(-35);
                //elevator up and dump
                break;
        }
        pivot(90);
        drivetrain.moveFB(50);
        pivot(35);
        drivetrain.moveFB(10);
        //intake out
    }
    public void pivot(double angle){
        double pow = Constants.Drivetrain.PIVOT_POWER;
        if(angle > 0){
            drivetrain.setPower(new double[]{-pow, -pow, pow, pow});
        } else {
            drivetrain.setPower(new double[]{pow, pow, -pow, -pow});
        }
        while(opModeIsActive() && Math.abs(imu.getAngle() - angle) > 0){ }
        drivetrain.setPower(new double[]{0, 0, 0, 0});
    }

    public void setIntake(Intake.RollerState state){
        intake.setRollerState(state);
        intake.loop();
        sleep(2000);
        intake.setRollerState(state);
        intake.loop();
    }
}
