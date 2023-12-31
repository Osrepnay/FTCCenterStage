package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.Propecessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Auto", group = "Iterative Opmode")
public class Auto extends OpMode {
    private DcMotor[][] wheels;
    private VisionPortal portal;

    @Override
    public void init() {
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

            wheels[i][1] = hardwareMap.get(DcMotor.class, wheelNames[i][1]);
            wheels[i][1].setDirection(DcMotor.Direction.FORWARD);
            wheels[i][1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), new Propecessor(telemetry));
    }

    @Override
    public void start() {
        portal.stopStreaming();
    }

    @Override
    public void loop() {
    }
}
