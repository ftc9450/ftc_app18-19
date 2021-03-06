package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemManager;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Vector2D;

@TeleOp
public class Teleop extends OpMode{
    Drivetrain drivetrain;
    //Lifter lifter;
    SubsystemManager subsystemManager;
    Intake intake;
    Climber climb;
    //DcMotor climb;
    private boolean rollerIn;
    private boolean rollerInPressed;
    private boolean rollerOut;
    private boolean rollerOutPressed;
    @Override
    public void init() {

        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF),hardwareMap.dcMotor.get(Constants.Drivetrain.LB),
                hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));
        //lifter = new Lifter(hardwareMap.dcMotor.get(Constants.Lifter.LIFT), hardwareMap.servo.get(Constants.Lifter.LID));
        intake = new Intake(hardwareMap.dcMotor.get(Constants.Intake.PI), hardwareMap.dcMotor.get((Constants.Intake.RO)));
        climb = new Climber(hardwareMap.dcMotor.get(Constants.Climber.EL), hardwareMap.dcMotor.get(Constants.Climber.PI),
                hardwareMap.servo.get(Constants.Climber.HK), hardwareMap.servo.get(Constants.Climber.PL));
        //climb = hardwareMap.dcMotor.get(Constants.Climber.EL, Constants.Climber.PI, Constants.Climber.HK);
        subsystemManager = new SubsystemManager();
        subsystemManager = subsystemManager.add(climb).add(intake);//.add(climb);
        rollerIn = false;
        rollerInPressed = false;
        rollerOut = false;
        rollerOutPressed = false;
    }
    public void loop(){
        Vector2D v = new Vector2D();
        v.x = gamepad1.left_stick_x + (gamepad1.dpad_left? -0.5: gamepad1.dpad_right? 0.5:0);
        v.y = -gamepad1.left_stick_y + (gamepad1.dpad_down? -0.5: gamepad1.dpad_up? 0.5:0);
        float z = gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger)/2;
        double[] driveSignal = new double[]{0,0,0,0};
        driveSignal[0]= v.x + v.y + z; // up on left stick is -1.
        driveSignal[1]= -v.x + v.y + z;
        driveSignal[2]= -v.x + v.y - z;
        driveSignal[3]= v.x + v.y - z;
        drivetrain.setPower(driveSignal);
        /*if (gamepad1.x){
            climb.setClimberState(Climber.ClimberState.UP);
        } else if(gamepad1.y) {
            climb.setClimberState(Climber.ClimberState.DOWN);
        } else {
            climb.setClimberState(Climber.ClimberState.OFF);
        }*/


        if (gamepad2.x) {
            climb.setElevatorState(Climber.ElevatorState.UP);
        } else if(gamepad2.y){
            climb.setElevatorState(Climber.ElevatorState.DOWN);
        } else{
            climb.setElevatorState(Climber.ElevatorState.OFF);
        }


        if (gamepad2.dpad_up) {
            intake.setSlideState(Intake.SlideState.IN);
        } else if (gamepad2.dpad_down) {
            intake.setSlideState(Intake.SlideState.OUT);
        } else {
            intake.setSlideState(Intake.SlideState.OFF);
        }


        if (gamepad2.a) {
            intake.setPivotState(Intake.PivotState.UP);
        } else if (gamepad2.b) {
            intake.setPivotState(Intake.PivotState.DOWN);
        } else {
            intake.setPivotState(Intake.PivotState.OFF);
        }

        if (gamepad2.left_stick_y >= 1) {
            intake.setPivotState(Intake.PivotState.UP);
        } else if (gamepad2.left.stick_y <= -1) {
            intake.setPivotState(Intake.PivotState.DOWN);
        } else {
            intake.setPivotState(Intake.PivotState.OFF);
        }
        if (gamepad2.right_stick_y >= 1) {
            climb.setElevatorState(Climber.ElevatorState.UP);
        } else if (gamepad2.right.stick_y <= -1) {
            climb.setElevatorState(Climber.ElevatorState.DOWN);
        } else {
            climb.setElevatorState(Climber.ElevatorState.OFF);
        }



        /*if(gamepad2.b){ // toggles roller on/off
            if(!rollerInPressed){
                if(!rollerIn){
                    intake.setRollerState(Intake.RollerState.IN);
                    rollerIn = false;
                } else{
                    intake.setRollerState(Intake.RollerState.OFF);
                    rollerIn = true;
                }
                rollerOut = false;
            }
            rollerInPressed = true;
        } else if(gamepad2.a){
            if(!rollerOutPressed){
                if(!rollerOut){
                    intake.setRollerState(Intake.RollerState.OUT);
                    rollerOut = false;
                } else{
                    intake.setRollerState(Intake.RollerState.OFF);
                    rollerOut = true;
                }
                rollerIn = false;
            }
            rollerOutPressed = true;
        } else{

            intake.setRollerState(Intake.RollerState.OFF);
        }*/

        if (gamepad2.right_bumper){
            intake.setPivotState(Intake.PivotState.DOWN);
        } else if(gamepad2.left_bumper){
            intake.setPivotState(Intake.PivotState.UP);
        } else{
            intake.setPivotState(Intake.PivotState.OFF);
        }



        subsystemManager.loop();
    }
}
