package org.firstinspires.ftc.team9450.tests.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team9450.subsystems.Subsystem;
import org.firstinspires.ftc.team9450.subsystems.SubsystemManager;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.MotionTracker;
import org.firstinspires.ftc.team9450.subsystems.Climber;
import org.firstinspires.ftc.team9450.util.Vector2D;

public class climbtest {}/*extends OpMode {
    Climber climb;
    SubsystemManager subsystemManager;


    public void init() {
        climb = new Climber(hardwareMap.dcMotor.get(Constants.Climber.EL));
        subsystemManager = new SubsystemManager();
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
            if (gamepad1.x){
                climb.setClimberState(Climber.ClimberState.UP);
            } else if(gamepad1.y) {
                climb.setClimberState(Climber.ClimberState.DOWN);
            } else {
                climb.setClimberState(Climber.ClimberState.OFF);
            }


    }


    }
*/