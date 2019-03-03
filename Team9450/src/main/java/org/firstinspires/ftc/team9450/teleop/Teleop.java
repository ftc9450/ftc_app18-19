package org.firstinspires.ftc.team9450.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team9450.subsystems.Climber;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.subsystems.Elevator;
import org.firstinspires.ftc.team9450.subsystems.Intake;

import org.firstinspires.ftc.team9450.subsystems.SubsystemManager;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.Vector2D;

@TeleOp
public class Teleop extends OpMode{
    Drivetrain drivetrain;
    SubsystemManager subsystemManager;
    Intake intake;
    Climber climb;
    Elevator elevator;
    @Override
    public void init() {

        drivetrain = new Drivetrain(hardwareMap);
        elevator = new Elevator(hardwareMap);
        climb = new Climber(hardwareMap);
        intake = new Intake(hardwareMap);

        subsystemManager = new SubsystemManager();
        subsystemManager = subsystemManager.add(climb).add(intake).add(elevator);
    }
    public void loop(){
        Vector2D v = new Vector2D();
        v.x = gamepad1.left_stick_x + (gamepad1.dpad_left? -0.5: gamepad1.dpad_right? 0.5:0);
        v.y = -gamepad1.left_stick_y + (gamepad1.dpad_down? -0.5: gamepad1.dpad_up? 0.5:0);
        float z = gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger)/2;
        double[] driveSignal = new double[]{0,0,0,0};
        double modifier = gamepad1.right_bumper? 0.5:1;
        driveSignal[0]= v.x + v.y + z; // up on left stick is -1.
        driveSignal[1]= -v.x + v.y + z;
        driveSignal[2]= -v.x + v.y - z;
        driveSignal[3]= v.x + v.y - z;
        for (int i = 0; i < 4; i++) driveSignal[i] *= modifier;
        drivetrain.setPower(driveSignal);

        if (gamepad1.a) {
            climb.setElevatorState(Climber.ElevatorState.UP);
        } else if (gamepad1.b) {
            climb.setElevatorState(Climber.ElevatorState.DOWN);
        }

        if (gamepad2.right_bumper) {
            intake.setSliderState(Intake.SlideState.OUT);
        } else if (gamepad2.left_bumper) {
            intake.setSliderState(Intake.SlideState.IN);
        } else {
            intake.setSliderState(Intake.SlideState.OFF);
        }

        if (gamepad2.right_trigger > 0.15) {
            intake.setRollerState(Intake.RollerState.IN);
        } else if (gamepad2.left_trigger > 0.15) {
            intake.setRollerState(Intake.RollerState.OUT);
        } else {
            intake.setRollerState(Intake.RollerState.OFF);
        }

        if (gamepad2.dpad_up) {
            intake.setPivotState(Intake.PivotState.UP);
        } else if (gamepad2.dpad_down) {
            intake.setPivotState(Intake.PivotState.DOWN);
        }

        subsystemManager.loop();
    }
}
