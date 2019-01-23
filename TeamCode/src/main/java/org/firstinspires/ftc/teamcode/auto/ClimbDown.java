package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MotionTracker;
@Autonomous
public class ClimbDown extends LinearOpMode {
    private Climber climb;
    private Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        climb = new Climber(hardwareMap.dcMotor.get(Constants.Climber.EL),hardwareMap.dcMotor.get(Constants.Climber.PI),
                hardwareMap.servo.get(Constants.Climber.HK), hardwareMap.servo.get(Constants.Climber.PL));
        intake = new Intake(hardwareMap.dcMotor.get(Constants.Intake.PI), hardwareMap.dcMotor.get((Constants.Intake.RO)));
        waitForStart();
        // lower robot
        intake.setPivotState(Intake.PivotState.DOWN);intake.loop();
        Thread.sleep(1500);
        intake.setPivotState(Intake.PivotState.LEVEL);intake.loop();
        climb.setElevatorState(Climber.ElevatorState.OFF);
        climb.setPawlState(Climber.PawlState.DISENGAGED);
        climb.loop();
        climb.setElevatorState(Climber.ElevatorState.UP);
        climb.loop();
        sleep(6000);
        climb.setHookState(Climber.HookState.OPEN);
        sleep(1000);
        climb.setElevatorState(Climber.ElevatorState.DOWN);
        climb.loop();
        sleep(2000);
        climb.setElevatorState(Climber.ElevatorState.OFF);
    }

    public void run(Drivetrain drivetrain, MotionTracker tracker) throws InterruptedException {
        intake.setPivotState(Intake.PivotState.UP);
        Thread.sleep(2000);
        drivetrain.moveFB(40, 0.5, true, tracker);
        intake.setPivotState(Intake.PivotState.DOWN);
        Thread.sleep(2000);

    }

}