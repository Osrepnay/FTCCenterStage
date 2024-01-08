package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class LiftTester extends OpMode {
    private DcMotorEx arm0;
    private DcMotorEx arm1;
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
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        // arm1.setTargetPosition(1000);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // arm1.setPower(1);
    }

    @Override
    public void loop() {
        telemetry.addData("arm0", arm0.getCurrentPosition());
        telemetry.addData("arm1", arm1.getCurrentPosition());
    }
}
