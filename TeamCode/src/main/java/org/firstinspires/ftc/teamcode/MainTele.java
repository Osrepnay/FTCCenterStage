/*
Copyright 2022 FIRST Tech Challenge Team 22087

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class MainTele extends OpMode {
    /* Declare OpMode members. */
    private Blinker controlHub;
    private DcMotor[][] wheels; // see init() for layout
    // private DcMotor[] arms;
    private Servo armServo;
    private DcMotor intake;
    private CRServo intakeWinch;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        controlHub = hardwareMap.get(Blinker.class, "Control Hub");

        String[][] wheelNames = {
                {"wheelFrontleft", "wheelFrontright"},
                {"wheelBackleft", "wheelBackright"}
        };
        wheels = new DcMotor[2][2];
        for (int i = 0; i < wheelNames.length; i++) {
            // left wheel turns back when motor turning clockwise
            wheels[i][0] = hardwareMap.get(DcMotor.class, wheelNames[i][0]);
            wheels[i][0].setDirection(DcMotor.Direction.REVERSE);
            // wheels[i][0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // wheels[i][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            wheels[i][1] = hardwareMap.get(DcMotor.class, wheelNames[i][1]);
            wheels[i][1].setDirection(DcMotor.Direction.FORWARD);
            // wheels[i][1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // wheels[i][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        // arms = new DcMotor[2];
        // arms[0] = hardwareMap.get(DcMotor.class, "armRight");
        // arms[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // arms[1] = hardwareMap.get(DcMotor.class, "armRight");
        // arms[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // arms[1].setDirection(DcMotorSimple.Direction.FORWARD);
        // armServo = hardwareMap.get(Servo.class, "armServo");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeWinch = hardwareMap.get(CRServo.class, "intakeWinch");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    private double goodabs(double n) {
        if (n < 0) {
            return -n;
        } else {
            return n;
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double[][] rotateRightScale = {
                {1, -1},
                {1, -1}
        };
        double[][] rotateLeftScale = {
                {-1, 1},
                {-1, 1}
        };

        double[][] activeScale;
        double pressedY = gamepad1.right_stick_y;
        double powerY = pressedY;

        double pressedX = -gamepad1.right_stick_x;
        double powerX = pressedX;

        double power;
        telemetry.addData("pressedx", pressedX);
        telemetry.addData("pressedy", pressedY);
        telemetry.addData("powerx", powerX);
        telemetry.addData("powery", powerY);
        if (gamepad1.right_trigger > 0.01) {
            power = gamepad1.right_trigger;
            activeScale = rotateRightScale;
        } else if (gamepad1.left_trigger > 0.01) {
            power = gamepad1.left_trigger;
            activeScale = rotateLeftScale;
        } else {
            activeScale = new double[][]{
                    {powerX + powerY, powerY - powerX},
                    {powerY - powerX, powerX + powerY}
            };
            power = 1;
        }
        telemetry.addData("Power", power);
        for (int i = 0; i < activeScale.length; i++) {
            for (int j = 0; j < activeScale[i].length; j++) {
                wheels[i][j].setPower(power * activeScale[i][j]);
            }
        }

        double armPower = -gamepad1.left_stick_y + gamepad2.left_stick_y;
        // arms[0].setPower(armPower);
        // arms[1].setPower(armPower);
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            //armServo.setPosition(0);
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            // armServo.setPosition(0.256);
        }
        // telemetry.addData("Servo angle", armServo.getPosition());

        if (gamepad1.x) {
            intakeWinch.setPower(-1);
        } else if (gamepad1.a) {
            intakeWinch.setPower(0);
        } else if (gamepad1.b) {
            intakeWinch.setPower(1);
        }
        if (gamepad1.dpad_up) {
            intake.setPower(1);
        } else if (gamepad1.dpad_left) {
            intake.setPower(0);
        } else if (gamepad1.dpad_down) {
            intake.setPower(-1);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
