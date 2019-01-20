package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MotionTracker;
@Autonomous
public class ClimbDown extends LinearOpMode {
    private Climber climb;

    @Override
    public void runOpMode() throws InterruptedException {
        climb = new Climber(hardwareMap.dcMotor.get(Constants.Climber.EL),hardwareMap.dcMotor.get(Constants.Climber.PI),
                hardwareMap.servo.get(Constants.Climber.HK), hardwareMap.servo.get(Constants.Climber.PL));

        waitForStart();
        // lower robot
        climb.setElevatorState(Climber.ElevatorState.OFF);
        climb.setPawlState(Climber.PawlState.DISENGAGED);
        climb.loop();
        climb.setElevatorState(Climber.ElevatorState.UP);
        climb.loop();
        while(opModeIsActive() && climb.getPosition()<Constants.Climber.CLIMBED){}
        climb.setElevatorState(Climber.ElevatorState.OFF);
        climb.setHookState(Climber.HookState.OPEN);
        climb.loop();

    }

}