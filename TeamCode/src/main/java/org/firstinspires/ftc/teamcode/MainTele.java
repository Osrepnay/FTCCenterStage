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

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
    Blinker controlHub;
    IMU imu;
    private DcMotor[][] wheels;
    private DcMotor winch;
    private Servo launch;
    private Servo hangLatch;
    private StateManager stateManager;
    private static final double STRAFE_MULT = SampleMecanumDrive.LATERAL_MULTIPLIER;
    private static final double LAUNCH_START = 0;
    private static final double LAUNCH_LAUNCH = LAUNCH_START + 0.5;
    private static final double HANG_START = 0;
    private static final double HANG_RELEASE = HANG_START + 0.06;

    private GamepadWrapper up1;
    private GamepadWrapper left1;
    private GamepadWrapper right1;
    private GamepadWrapper down1;
    private GamepadWrapper x1;
    private GamepadWrapper y1;
    private GamepadWrapper leftBumper1;
    private GamepadWrapper rightBumper1;
    private GamepadWrapper leftBumper2;
    private GamepadWrapper rightBumper2;
    private GamepadWrapper back1;

    PIDFController headingController = new PIDFController(new PIDCoefficients(0.7, 0.3, 0.3));

    @Override
    public void init() {
        up1 = new GamepadWrapper(gamepad1, g -> g.dpad_up);
        left1 = new GamepadWrapper(gamepad1, g -> g.dpad_left);
        right1 = new GamepadWrapper(gamepad1, g -> g.dpad_right);
        down1 = new GamepadWrapper(gamepad1, g -> g.dpad_down);
        x1 = new GamepadWrapper(gamepad1, g -> g.x);
        y1 = new GamepadWrapper(gamepad1, g -> g.y);
        leftBumper1 = new GamepadWrapper(gamepad1, g -> g.left_bumper);
        rightBumper1 = new GamepadWrapper(gamepad1, g -> g.right_bumper);
        leftBumper2 = new GamepadWrapper(gamepad2, g -> g.left_bumper);
        rightBumper2 = new GamepadWrapper(gamepad2, g -> g.right_bumper);
        back1 = new GamepadWrapper(gamepad1, g -> g.back);

        headingController.setInputBounds(0, 2 * Math.PI);
        headingController.setOutputBounds(-3, 3);

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
            wheels[i][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            wheels[i][1] = hardwareMap.get(DcMotor.class, wheelNames[i][1]);
            wheels[i][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            wheels[i][0].setDirection(DcMotor.Direction.REVERSE);
            wheels[i][1].setDirection(DcMotor.Direction.FORWARD);
        }
        winch = hardwareMap.get(DcMotor.class, "winch");
        stateManager = new StateManager(hardwareMap, telemetry);
        launch = hardwareMap.get(Servo.class, "launch");
        launch.setDirection(Servo.Direction.REVERSE);
        launch.setPosition(LAUNCH_START);
        hangLatch = hardwareMap.get(Servo.class, "hangLatch");
        hangLatch.setPosition(HANG_START);
        gamepad1.reset();
        gamepad2.reset();
        telemetry.addData("Status", "Initialized");
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
        x *= STRAFE_MULT;
        return new double[][]{
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
                wheels[i][j].setPower(power[i][j] / normFac);
            }
        }
    }

    private double scaleMagnitude(double magnitude) {
        double slowMax = 0.4;
        double slowRegion = 0.6;
        if (magnitude <= slowRegion) {
            return slowMax / slowRegion * magnitude;
        } else {
            return (1 - slowMax) / (1 - slowRegion) * (magnitude - slowRegion) + slowMax;
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    private boolean tanking = false;

    @Override
    public void loop() {
        up1.update();
        left1.update();
        right1.update();
        down1.update();
        x1.update();
        y1.update();
        leftBumper1.update();
        rightBumper1.update();
        leftBumper2.update();
        rightBumper2.update();
        back1.update();

        double pressedX = gamepad1.right_stick_x + gamepad2.right_stick_x;
        double pressedY = -(gamepad1.right_stick_y + gamepad2.right_stick_y);
        // change power curve while maintaining relative power
        double magnitudeRaw = Math.sqrt(pressedX * pressedX + pressedY * pressedY);
        double pressedMagnitude = scaleMagnitude(magnitudeRaw);
        double pressedAngle = Math.atan2(pressedY, pressedX);
        pressedX = Math.cos(pressedAngle) * pressedMagnitude;
        pressedY = Math.sin(pressedAngle) * pressedMagnitude;

        // field centric drive
        // https://matthew-brett.github.io/teaching/rotation_2d.html
        double yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double yawCos = Math.cos(yaw);
        double yawSin = Math.sin(yaw);
        double adjustedX = yawCos * pressedX - yawSin * pressedY;
        double adjustedY = yawSin * pressedX + yawCos * pressedY;
        double rotate =
                scaleMagnitude(gamepad1.left_stick_x + gamepad2.left_stick_x + gamepad1.right_trigger - gamepad1.left_trigger + gamepad2.right_trigger - gamepad2.left_trigger);
        double[][] powers = xyrPower(adjustedX, adjustedY, rotate);
        if (tanking) {
            if (pressedMagnitude > 0.01) {
                headingController.setTargetPosition((-pressedAngle + Math.PI / 2) % (2 * Math.PI));
                double adj = headingController.update(yaw);
                // telemetry.addData("out", adj);
                for (int r = 0; r < powers[0].length; r++) {
                    powers[r][0] += adj;
                }
                for (int r = 0; r < powers[0].length; r++) {
                    powers[r][1] -= adj;
                }
            } else {
                headingController.reset();
            }
        }
        powerWheels(powers);

        if (gamepad1.options || gamepad2.options) {
            imu.resetYaw();
        }

        stateManager.update();
        if (down1.get()) {
            stateManager.queueState(StateManager.State.LOWERED);
        }
        if (left1.get()) {
            // tanking = !tanking;
            // headingController.reset();
            stateManager.lowerExtendo();
        }
        if (right1.get()) {
            stateManager.raiseExtendo();
        }
        if (up1.get()) {
            stateManager.queueState(StateManager.State.INTAKING);
        }
        if (x1.get()) {
            stateManager.queueState(StateManager.State.SCORE_SINGLE);
        }
        if (y1.get()) {
            stateManager.queueState(StateManager.State.SCORE_DOUBLE);
        }
        if (leftBumper1.get()) {
            stateManager.lowerArm();
        }
        if (rightBumper1.get()) {
            stateManager.raiseArm();
        }
        if (leftBumper2.get()) {
        }
        if (rightBumper2.get()) {
        }

        if (gamepad1.b) {
            telemetry.addData("winching", "winching");
            winch.setPower(1);
        } else if (gamepad1.a) {
            winch.setPower(-1);
        } else {
            winch.setPower(0);
        }
        if (back1.get()) {
            stateManager.propsTo(x -> new StateManager.StateProps(
                    -x.intakePower, x.liftHeight, x.wristState, x.grabberState, x.extendoIdx));
        }

        if (gamepad1.ps) {
            launch.setPosition(LAUNCH_LAUNCH);
        }
        if (gamepad1.touchpad) {
            hangLatch.setPosition(HANG_RELEASE);
        }

        telemetry.addData("state", stateManager.state);
        telemetry.addData("yaw", yaw / Math.PI * 180);
        telemetry.addData("height", stateManager.stateProps.liftHeight);
        // telemetry.setAutoClear(false);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
