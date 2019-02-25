package org.firstinspires.ftc.team9450.tests.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Power Distribution", group = "Drivetrain")
public class PowerDistribution extends OpMode {
    private DcMotor lf, lb, rf, rb;
    private DcMotor[] drive;
    private float lfscale, lbscale, rfscale, rbscale;
    private Mode mode;
    private float add;
    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("frontleft");
        lb = hardwareMap.dcMotor.get("backleft");
        rf = hardwareMap.dcMotor.get("frontright");
        rb = hardwareMap.dcMotor.get("backright");
        drive = new DcMotor[]{lf, lb, rf, rb};
        for (DcMotor motor:drive) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        drive[0].setDirection(DcMotorSimple.Direction.REVERSE);
        drive[2].setDirection(DcMotorSimple.Direction.REVERSE);
        lfscale = lbscale = rfscale = rbscale = 1;
        mode = Mode.LF;
        add = 0;
    }

    @Override
    public void loop() {
        float r = gamepad1.right_trigger - gamepad1.left_trigger;
        lf.setPower(lfscale*(gamepad1.left_stick_x - gamepad1.left_stick_y + r));
        lb.setPower(lbscale*(-gamepad1.left_stick_x - gamepad1.left_stick_y + r));
        rf.setPower(rfscale*(-gamepad1.left_stick_x - gamepad1.left_stick_y - r));
        rb.setPower(rbscale*(gamepad1.left_stick_x - gamepad1.left_stick_y - r));
        telemetry.addData("controls", "lf:left, rf:up, rb:right, lb:down");
        for (int i = 0; i < 4; i++) telemetry.addData("motor "+i+" position", drive[i].getCurrentPosition());
        if (gamepad1.dpad_left) mode = Mode.LF;
        else if (gamepad1.dpad_up) mode = Mode.RF;
        else if (gamepad1.dpad_right) mode = Mode.RB;
        else if (gamepad1.dpad_down) mode = Mode.LB;
        if (gamepad1.left_bumper) add = (float) -0.001;
        else if (gamepad1.right_bumper) add = (float) 0.001;
        else add = 0;
        switch (mode) {
            case LF:
                lfscale += add;
                break;
            case LB:
                lbscale += add;
                break;
            case RF:
                rfscale += add;
                break;
            case RB:
                rbscale += add;
                break;
        }
        telemetry.addData("lf", lfscale);
        telemetry.addData("lb", lbscale);
        telemetry.addData("rf", rfscale);
        telemetry.addData("rb", rbscale);
    }

    private enum Mode {
        LF, LB, RF, RB;
    }
}
