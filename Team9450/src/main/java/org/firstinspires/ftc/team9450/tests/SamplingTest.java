package org.firstinspires.ftc.team9450.tests;

import org.firstinspires.ftc.team9450.sensors.GoldDetector;
import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Climber;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.util.Constants;

public class SamplingTest {
    private Drivetrain drive;
    private Climber climb;
    private GoldDetector gold;
    private Gyroscope gyro;
    private int connection;
    private int signal;
    private double angle;
    @Override
    public void init() {
        drive = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF), hardwareMap.dcMotor.get(Constants.Drivetrain.LB),
                hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));
        climb = new Climber(hardwareMap.dcMotor.get(Constants.Climber.EL));
        climb.setElevatorState(Climber.ElevatorState.OFF);

    }
    @Override
    public void loop() {
        climb.setElevatorState(Climber.ElevatorState.DOWN);

    }



}
