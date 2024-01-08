package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.StateManager.SERVO_WAIT_LONG_MS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opencv.Propecessor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Auto", group = "Iterative Opmode", preselectTeleOp = "MainTele")
public class Auto extends OpMode {
    private DcMotor[] arm;
    private Propecessor processor;
    private VisionPortal portal;
    private Propecessor.Spike spike;
    private SampleMecanumDrive drive;
    private Pose2d startPose = new Pose2d(-46.719 + 18 / 2, -70.281 + 18 / 2, Math.toRadians(-90));

    private enum State {
        TO_SPIKE(null),
        WAIT_SPIKE_DROP(0),
        TO_BACKDROP(null),
        WAIT_BACKDROP(0),
        PARK(null);
        Object initFinishData;

        State(Object initFinishData) {
            this.initFinishData = initFinishData;
        }
    }

    private boolean armIsBusy() {
        return true;
   }

    private boolean stateFinishChecker(State state, Object data) {
        switch (state) {
            case TO_SPIKE:
            case TO_BACKDROP:
                return !drive.isBusy() && !armIsBusy();
            case WAIT_SPIKE_DROP:
                long start = (long) data;
                return System.currentTimeMillis() - start >= SERVO_WAIT_LONG_MS;
            case WAIT_BACKDROP:
                break;
            case PARK:
                return !drive.isBusy();
        }
        return false;
    }

    @Override
    public void init() {
        arm = new DcMotor[2];
        arm[0] = hardwareMap.get(DcMotor.class, "arm0");
        arm[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm[0].setDirection(DcMotorSimple.Direction.REVERSE);
        arm[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm[1] = hardwareMap.get(DcMotor.class, "arm1");
        arm[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm[1].setDirection(DcMotorSimple.Direction.REVERSE);
        arm[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        processor = new Propecessor(telemetry);
        portal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                processor
        );
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
    }

    @Override
    public void start() {
        // portal.stopStreaming();
        spike = processor.selectedSpike;
        TrajectorySequence auto = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(startPose.getX(), startPose.getY() + 3, startPose.getHeading()), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(0)), Math.toRadians(80))
                .build();
        drive.followTrajectorySequenceAsync(auto);
    }

    @Override
    public void loop() {
        drive.update();
    }
}
