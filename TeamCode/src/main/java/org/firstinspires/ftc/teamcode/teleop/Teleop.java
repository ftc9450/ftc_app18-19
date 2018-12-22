package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lifter;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Vector2D;

public class Teleop extends OpMode{
    Drivetrain drivetrain;
    Lifter lifter;
    SubsystemManager subsystemManager;
    Intake intake;
    Climber climb;
    @Override
    public void init() {
        drivetrain=new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF),hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));
        lifter = new Lifter(hardwareMap.dcMotor.get(Constants.Lifter.LI));
        intake = new Intake(hardwareMap.dcMotor.get(Constants.Intake.RO),hardwareMap.dcMotor.get(Constants.Intake.BA));
        climb = new Climber(hardwareMap.dcMotor.get(Constants.Climber.CL));
        subsystemManager = new SubsystemManager();
        subsystemManager = subsystemManager.add(drivetrain);
        subsystemManager = subsystemManager.add(lifter);
    }
    public void loop(){
        Vector2D v = new Vector2D();
        v.x = gamepad1.left_stick_x + (gamepad1.dpad_left? -0.5: gamepad1.dpad_right? 0.5:0);
        v.y = -gamepad1.left_stick_y + (gamepad1.dpad_down? -0.5: gamepad1.dpad_up? 0.5:0);
        float z = gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger)/2;
        double[] driveSignal = new double[4];
        driveSignal[0]=v.x + v.y + z;
        driveSignal[1]=-v.x + v.y + z;
        driveSignal[2]=-v.x + v.y - z;
        driveSignal[3]=v.x + v.y - z;
        drivetrain.setPower(driveSignal);
        subsystemManager.loop();
    }
}
