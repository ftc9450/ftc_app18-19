package org.firstinspires.ftc.team9450.tests.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.MotionTracker;
import org.firstinspires.ftc.team9450.util.Vector2D;

@TeleOp(name = "Drivetrain Test", group = "Drivetrain")
public class DrivetrainTest extends OpMode {
    private Drivetrain drive;
    //private Gyroscope imu;
    private MotionTracker tracker;

    @Override
    public void init() {
        drive = new Drivetrain(hardwareMap);/*new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF),hardwareMap.dcMotor.get(Constants.Drivetrain.LB),
                hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));*/
        //imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
    }

    @Override
    public void loop() {
        drive.setPower(driveSignal());
        ///telemetry.addData("angle", imu.getAngle());
        int pos = drive.getPosition();
        telemetry.addData("position", pos);
    }

    private double[] driveSignal() {
        Vector2D v = new Vector2D();
        v.x = gamepad1.left_stick_x + (gamepad1.dpad_left? -0.5: gamepad1.dpad_right? 0.5:0);
        v.y = -gamepad1.left_stick_y + (gamepad1.dpad_down? -0.5: gamepad1.dpad_up? 0.5:0);
        float z = gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger)/2;
        double[] driveSignal = new double[]{0,0,0,0};
        driveSignal[0]= v.x + v.y + z; // up on left stick is -1.
        driveSignal[1]= -v.x + v.y + z;
        driveSignal[2]= -v.x + v.y - z;
        driveSignal[3]= v.x + v.y - z;
        return driveSignal;
    }
}
