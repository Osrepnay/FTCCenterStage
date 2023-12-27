package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Intake extends OpMode {
    private DcMotor intake;
    private DcMotor arm;
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");
    }

    @Override
    public void loop() {
        intake.setPower(gamepad1.left_stick_y);
        arm.setPower(gamepad1.right_stick_y);
    }
}
