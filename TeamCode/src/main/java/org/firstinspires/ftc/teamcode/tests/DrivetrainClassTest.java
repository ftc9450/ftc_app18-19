package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MotionTracker;
import org.firstinspires.ftc.teamcode.util.Vector2D;

@TeleOp
public class DrivetrainClassTest extends OpMode {

    private Drivetrain drivetrain;
    private Gyroscope imu;
    private MotionTracker tracker;
    private boolean aPressed;
    private boolean bPressed;
    private boolean xPressed;
    private boolean yPressed;

    public void init(){
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF),hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));
        tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain, imu, 0);

    }
    public void loop(){
        Vector2D v = new Vector2D();
        v.x = gamepad1.left_stick_x + (gamepad1.dpad_left? -0.5: gamepad1.dpad_right? 0.5:0);
        v.y = -gamepad1.left_stick_y + (gamepad1.dpad_down? -0.5: gamepad1.dpad_up? 0.5:0);
        float z = gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger)/2;
        double[] driveSignal = new double[]{0,0,0,0};
        driveSignal[0]= (v.x + v.y + z); // up on left stick is -1.
        driveSignal[1]= (-v.x + v.y + z);
        driveSignal[2]= (-v.x + v.y - z);
        driveSignal[3]= (v.x + v.y - z);
        drivetrain.setPower(driveSignal);
        drivetrain.setState(Drivetrain.DrivetrainState.Linear);

        if(gamepad1.a){
            if(!aPressed) {
                tracker.enableAndResetEncoders();
                //drivetrain.moveFB(12,0.25,true,tracker);
                testForward(12, 0.25);
                aPressed = true;
            }
        } else{
            aPressed = false;
        }
        if(gamepad1.b){
            if(!bPressed) {
                tracker.enableAndResetEncoders();
                drivetrain.moveFB(8,0.25,false,tracker);
                bPressed = true;
            }
        } else{
            bPressed = false;
        }
        if(gamepad1.x){
            if(!xPressed) {
                tracker.enableAndResetEncoders();
                pivotClockwise(90);
                xPressed = true;
            }
        } else{
            xPressed = false;
        }
        if(gamepad1.y){
            if(!yPressed) {
                tracker.enableAndResetEncoders();
                pivotCounterclockwise(90);
                yPressed = true;
            }
        } else{
            yPressed = false;
        }
        telemetry.addData("x: ",tracker.getXEncoderValue());
        telemetry.addData("y: ", tracker.getYEncoderValue());
        telemetry.addData("angle: ", imu.getAngle());
        telemetry.update();



    }

    public void testForward(double distance, double power){
        double leftPower = power * Constants.Drivetrain.FB_LEFT_POWER;
        double rightPower = power * Constants.Drivetrain.FB_RIGHT_POWER;
        double clicks = distance * Constants.MotionTracker.CLICKS_PER_INCH;
        drivetrain.setPower(new double[]{leftPower,leftPower,rightPower,rightPower});
        //drivetrain.setPower(0.5);
        try{while(tracker.getXEncoderValue() - 0 < clicks){}}catch(Exception e){telemetry.update();}// - Constants.Drivetrain.FB_THRESHOLD)
        drivetrain.setPower(0);
    }
    public void pivotClockwise(double angle){ // Turn clockwise given degree angle
        drivetrain.setState(Drivetrain.DrivetrainState.Turning);
        float startAngle = imu.getAngle();
        double power = Constants.Auto.PIVOT_POWER;
        double[] signal = new double[]{power,power,-power,-power};
        drivetrain.setPower(signal);
        while(angle - (imu.getAngle() - startAngle + 360) % 360 > Constants.Auto.PIVOT_THRESHOLD); // TODO: check value
        drivetrain.setPower(new double[]{0,0,0,0});
        tracker.updatePosition();
    }
    public void pivotCounterclockwise(double angle){ // Turn counterclockwise given degree angle
        drivetrain.setState(Drivetrain.DrivetrainState.Turning);
        float startAngle = imu.getAngle();
        double power = Constants.Auto.PIVOT_POWER;
        double[] driveSignal = new double[]{-power,-power,power,power};
        drivetrain.setPower(driveSignal);
        while(angle - (startAngle - imu.getAngle() + 360) % 360 > Constants.Auto.PIVOT_THRESHOLD); // TODO: check value
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
}
