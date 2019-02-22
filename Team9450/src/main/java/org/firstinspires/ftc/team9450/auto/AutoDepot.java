package org.firstinspires.ftc.team9450.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Climber;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.MotionTracker;


/**
 * Created by Avi on 12/31/2018.
 */
@Autonomous
public class AutoDepot extends LinearOpMode {

    private Drivetrain drivetrain;
    private Gyroscope imu;
    private MotionTracker tracker;
    private Climber climb;
    private final float initialAngle = 45;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF), hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));

        //imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        //tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain, imu, initialAngle); //TODO: check angle
        tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain, initialAngle); //TODO: check angle

        /*climb = new Climber(hardwareMap.dcMotor.get(Constants.Climber.EL),hardwareMap.dcMotor.get(Constants.Climber.PI),
                hardwareMap.servo.get(Constants.Climber.HK), hardwareMap.servo.get(Constants.Climber.PL));*/
        drivetrain.enableAndResetEncoders();
        double c=0.05;
        /*
        // Setup detector
        SamplingOrderDetector detector = new SamplingOrderDetector(); // Create the detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize detector with app context and camera
        detector.useDefaults(); // Set detector to use default settings

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable(); // Start detector
        */


        //float angleWhenHanging = tracker.getAbsoluteAngle();
        waitForStart();
        //drivetrain.moveFB(24,.3,true,tracker);
        // lower robot
        /*climb.setElevatorState(Climber.ElevatorState.OFF);
        climb.setPawlState(Climber.PawlState.DISENGAGED);
        climb.loop();
        climb.setElevatorState(Climber.ElevatorState.UP);
        climb.loop();
        while(climb.getPosition()<Constants.Climber.CLIMBED){}
        climb.setElevatorState(Climber.ElevatorState.OFF);
        climb.setHookState(Climber.HookState.OPEN);
        climb.loop();
        */
        // pivotTo(angleWhenHanging - 5);//correct for difference in angle caused by dropping
        drivetrain.moveFB(18,c,true,tracker);// drive forward until at corner of mat with samples
        int mineralPosition = 1; // placeholder // TODO: get position of gold sample (0, 1, 2) -> (left, center, right)
        switch(mineralPosition){
            //pivot to face gold mineral
            case 0:
                pivotClockwise(45);
                break;
            case 1:
                // should be aligned
                break;
            case 2:
                pivotCounterclockwise(45);
                break;
        }
        sleep(500);
        drivetrain.moveFB(26,c,true,tracker);// knock off gold mineral
        sleep(500);
        drivetrain.moveFB(26,c,false,tracker);// return to position
        //pivotCounterclockwise(90);
        pivot(90, true);
        /*pivotTo(135); // Drive will then drive backwards
        drivetrain.moveFB(45,.7,false,tracker);// drive backward to waypoint (on safe auto paths map)
        pivotClockwise(90 + Math.abs(initialAngle)); // turn left so back is facing depot // TODO: check value, right now it is the same as the initial angle
        drivetrain.moveFB(38,.7,false,tracker);// drive backward until depot
        // TODO: deposit team marker (from back of robot)
        pivotTo(180); // turn left to face crater
        drivetrain.moveFB(80,.7,true,tracker);// drive forward until parked on crater
*/
    }
    public void pivotClockwise(int angle){
        drivetrain.setPower(new double[]{0.05,0.05,-0.05,-0.05});
        sleep(5*angle);
        drivetrain.setPower(new double[]{0,0,0,0});
    }
    public void pivotCounterclockwise(int angle){
        drivetrain.setPower(new double[]{-0.05,-0.05,0.05,0.05});
        sleep(5*angle);
        drivetrain.setPower(new double[]{0,0,0,0});
    }
    public void pivot(double angle, boolean cc) {
        double Q = Math.PI/25;
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        if (cc) {
            drivetrain.setPower(new double[]{-0.3, -0.3, 0.3, 0.3});
        } else {
            drivetrain.setPower(new double[]{0.3, 0.3, -0.3, -0.3});
        }
        while (opModeIsActive() && Math.abs(imu.getAngle()) < angle - Q) {}
        drivetrain.setPower(0);
    }

/*
    public void pivotClockwise(double angle){ // Turn clockwise given degree angle
        drivetrain.setState(Drivetrain.DrivetrainState.Turning);
        float startAngle = imu.getAngle();
        double power = Constants.Auto.PIVOT_POWER;
        double[] driveSignal = new double[]{power,power,-power,-power};
        drivetrain.setPower(driveSignal);
        while(opModeIsActive() && angle - (imu.getAngle() - startAngle + 360) % 360 > Constants.Auto.PIVOT_THRESHOLD); // TODO: check value
        drivetrain.setPower(new double[]{0,0,0,0});
        tracker.updatePosition();
    }
    
    public void pivotCounterclockwise(double angle){ // Turn counterclockwise given degree angle
        drivetrain.setState(Drivetrain.DrivetrainState.Turning);
        float startAngle = imu.getAngle();
        double power = Constants.Auto.PIVOT_POWER;
        double[] driveSignal = new double[]{-power,-power,power,power};
        drivetrain.setPower(driveSignal);
        while(opModeIsActive() && angle - (startAngle - imu.getAngle() + 360) % 360 > Constants.Auto.PIVOT_THRESHOLD); // TODO: check value
        drivetrain.setPower(new double[]{0,0,0,0});
        tracker.updatePosition();
    }
    public void pivotTo(float targetAngle){ // relative to field boundaries
        float currentAngle = tracker.getAbsoluteAngle();
        if(Math.abs(currentAngle - targetAngle) > 180){ // must cross the line theta = 0
            if(currentAngle > targetAngle){
                pivotClockwise(360 - (currentAngle - targetAngle));
            } else if(targetAngle > currentAngle){
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
*/
}

