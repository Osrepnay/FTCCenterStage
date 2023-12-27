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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
    private IMU imu;
    private DcMotor[][] wheels; // see init() for layout
    // private DcMotor[] arms;
    private DcMotor intake;
    private DcMotor winch;
    private DcMotor[] arm;
    private Servo grabber;
    private final double GRABBER_START = 0.025;
    private final double GRABBER_SINGLE = 0.06;
    private final double GRABBER_DOUBLE = 0.15;
    private Servo wrist;
    private final double WRIST_START = 0.013;
    private final double WRIST_DUMP = 0.27;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        controlHub = hardwareMap.get(Blinker.class, "Control Hub");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );
        imu.resetYaw();

        String[][] wheelNames = {
                {"wheelFrontleft", "wheelFrontright"},
                {"wheelBackleft", "wheelBackright"}
        };
        wheels = new DcMotor[2][2];
        for (int i = 0; i < wheelNames.length; i++) {
            // left wheel turns back when motor turning clockwise
            wheels[i][0] = hardwareMap.get(DcMotor.class, wheelNames[i][0]);
            wheels[i][0].setDirection(DcMotor.Direction.REVERSE);
            wheels[i][0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // wheels[i][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            wheels[i][1] = hardwareMap.get(DcMotor.class, wheelNames[i][1]);
            wheels[i][1].setDirection(DcMotor.Direction.FORWARD);
            wheels[i][1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // wheels[i][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intake = hardwareMap.get(DcMotor.class, "intake");
        winch = hardwareMap.get(DcMotor.class, "winch");
        arm = new DcMotor[2];
        arm[0] = hardwareMap.get(DcMotor.class, "arm0");
        arm[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm[1] = hardwareMap.get(DcMotor.class, "arm1");
        arm[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber = hardwareMap.get(Servo.class, "grabber");
        grabber.setPosition(GRABBER_START);
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        wrist.setPosition(WRIST_START);
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

    private double[][] xyrPower(double x, double y, double rot) {
        return new double[][] {
                {x + y + rot, y - x - rot},
                {y - x + rot, x + y - rot}
        };
    }

    private void powerWheels(double[][] power) {
        double normFac = 1;
        for (int i = 0; i < wheels.length; i++) {
            for (int j = 0; j < wheels[i].length; j++) {
                normFac = Math.max(power[i][j], normFac);
            }
        }
        for (int i = 0; i < wheels.length; i++) {
            for (int j = 0; j < wheels[i].length; j++) {
                wheels[i][j].setPower(power[i][j]);
            }
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double[][] activeScale;
        double pressedX = gamepad1.right_stick_x;
        double pressedY = -gamepad1.right_stick_y;
        // field centric drive
        // https://matthew-brett.github.io/teaching/rotation_2d.html
        double yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("yaw", yaw);
        double yawCos = Math.cos(yaw);
        double yawSin = Math.sin(yaw);
        double adjustedX = yawCos * pressedX - yawSin * pressedY;
        double adjustedY = yawSin * pressedX + yawCos * pressedY;
        telemetry.addData("adjX", adjustedX);
        telemetry.addData("adjY", adjustedY);
        double rotate = gamepad1.right_trigger - gamepad1.left_trigger;
        powerWheels(xyrPower(adjustedX, adjustedY, rotate));

        double armPower = gamepad1.left_stick_y + gamepad2.left_stick_y;
        for (DcMotor m : arm) {
            m.setPower(armPower);
        }

        if (gamepad1.dpad_up) {
            intake.setPower(1);
        } else if (gamepad1.dpad_left) {
            intake.setPower(0);
        } else if (gamepad1.dpad_down) {
            intake.setPower(-1);
        }

        if (gamepad1.options) {
            imu.resetYaw();
        }
        if (gamepad1.x) grabber.setPosition(GRABBER_START);
        if (gamepad1.y) grabber.setPosition(GRABBER_SINGLE);
        if (gamepad1.b) grabber.setPosition(GRABBER_DOUBLE);
        if (gamepad1.left_bumper) wrist.setPosition(WRIST_START);
        if (gamepad1.right_bumper) wrist.setPosition(WRIST_DUMP);
        if (gamepad1.x) {
            winch.setPower((gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0));
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
