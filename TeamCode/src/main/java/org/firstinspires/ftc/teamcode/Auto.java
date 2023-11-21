package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto", group = "Iterative Opmode")
public class Auto extends OpMode {
    private DcMotor[][] wheels;
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
            // wheels[i][0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // wheels[i][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            wheels[i][1] = hardwareMap.get(DcMotor.class, wheelNames[i][1]);
            wheels[i][1].setDirection(DcMotor.Direction.FORWARD);
            // wheels[i][1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // wheels[i][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private long start = -1;
    @Override
    public void loop() {
        double powerX = -1;
        double powerY = 0;
        double[][] activeScale = new double[][]{
                {powerX + powerY, powerY - powerX},
                {powerY - powerX, powerX + powerY}
        };
        if (start == -1) {
            start = System.currentTimeMillis();
        } else {
            if (System.currentTimeMillis() > start + 700) {
                activeScale = new double[][] {
                        {0, 0}, {0, 0}
                };
            }
        }
        for (int i = 0; i < activeScale.length; i++) {
            for (int j = 0; j < activeScale[i].length; j++) {
                wheels[i][j].setPower(activeScale[i][j]);
            }
        }
    }
}
