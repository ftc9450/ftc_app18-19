package org.firstinspires.ftc.team9450.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team9450.subsystems.Climber;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.subsystems.Intake;
import org.firstinspires.ftc.team9450.subsystems.Lifter;

import org.firstinspires.ftc.team9450.subsystems.SubsystemManager;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.Vector2D;

@TeleOp
public class Teleop extends OpMode{
    Drivetrain drivetrain;
    //Lifter lifter;
    SubsystemManager subsystemManager;
    Intake intake;
    Climber climb;
    Lifter lifter;
    //DcMotor climb;
    private boolean rollerIn;
    private boolean rollerInPressed;
    private boolean rollerOut;
    private boolean rollerOutPressed;
    @Override
    public void init() {

        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF),hardwareMap.dcMotor.get(Constants.Drivetrain.LB),
                hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));
        //lifter = new Lifter(hardwareMap.dcMotor.get(Constants.Lifter.LIFT), hardwareMap.servo.get(Constants.Lifter.DUMP));
        //intake = new Intake(hardwareMap.servo.get(Constants.Intake.PI), hardwareMap.servo.get((Constants.Intake.RO)),hardwareMap.dcMotor.get((Constants.Intake.SL)));
        climb = new Climber(hardwareMap.dcMotor.get(Constants.Climber.EL));
        //climb = hardwareMap.dcMotor.get(Constants.Climber.EL, Constants.Climber.PI, Constants.Climber.HK);
        subsystemManager = new SubsystemManager();
        //subsystemManager = subsystemManager.add(climb).add(intake);//.add(climb);
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
        if (gamepad2.a) {
            //intake.setPivotState(Intake.PivotState.UP);
        } else if(gamepad2.b){
            //intake.setPivotState(Intake.PivotState.DOWN);
        } else{
            //intake.setPivotState(Intake.PivotState.OFF);
        }

        if (gamepad2.dpad_up) {
            //intake.setSliderState(Intake.SlideState.IN);
        } else if (gamepad2.dpad_down) {
            //intake.setSliderState(Intake.SlideState.OUT);
        } else {
            //intake.setSliderState(Intake.SlideState.OFF);
        }

        if (gamepad2.right_trigger > 0.15) {
            //intake.setRollerState(Intake.RollerState.IN);
        } else if (gamepad2.left_trigger > 0.15) {
            //intake.setRollerState(Intake.RollerState.OUT);
        } else {
            //intake.setRollerState(Intake.RollerState.OFF);
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





        subsystemManager.loop();
    }
}
