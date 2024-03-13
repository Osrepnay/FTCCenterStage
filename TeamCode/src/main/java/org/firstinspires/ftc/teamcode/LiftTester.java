package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LiftTester extends OpMode {
    private DcMotorEx arm0;
    private Servo servo;
    @Override
    public void init() {
        arm0 = hardwareMap.get(DcMotorEx.class, "arm0");
        arm0.setDirection(DcMotorSimple.Direction.REVERSE);
        // arm0.setTargetPosition(1000);
        arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // arm0.setPIDFCoefficients(new PIDFCoefficients());
        // arm0.setPower(1);
        servo = hardwareMap.get(Servo.class, "intakeLift");
        servo.setPosition(0);
    }

    private boolean leftDisable = false;
    private boolean rightDisable = false;

    @Override
    public void loop() {
        telemetry.addData("arm0", arm0.getCurrentPosition());
        telemetry.addData("x right 1", gamepad1.right_stick_x);
        telemetry.addData("y right 1", gamepad1.right_stick_y);
        telemetry.addData("x left 1", gamepad1.left_stick_x);
        telemetry.addData("y left 1", gamepad1.left_stick_y);
        telemetry.addData("x right 2", gamepad2.right_stick_x);
        telemetry.addData("y right 2", gamepad2.right_stick_y);
        telemetry.addData("x left 2", gamepad2.left_stick_x);
        telemetry.addData("y left 2", gamepad2.left_stick_y);
        if (gamepad1.left_bumper) {
            if (!leftDisable) {
                leftDisable = true;
                servo.setPosition(servo.getPosition() - (11.5 / 36 / 4));
            }
        } else {
            if (leftDisable) leftDisable = false;
            if (gamepad1.right_bumper) {
                if (!rightDisable) {
                    rightDisable = true;
                    servo.setPosition(servo.getPosition() + (11.5 / 36 / 4));
                }
            } else {
                if (rightDisable) rightDisable = false;
            }
        }
        telemetry.addData("e", servo.getPosition());
    }
}
