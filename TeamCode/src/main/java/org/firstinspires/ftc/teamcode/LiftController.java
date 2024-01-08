package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Function;

public class LiftController {
    private double proportional;
    private double integral;
    private double derivative;
    private Function<Integer, Double> feedforward;
    DcMotor[] liftMotors;

    private int setpoint = -1;
    private double integralAccum = 0;

    public LiftController(
            double proportional,
            double integral,
            double derivative,
            Function<Integer, Double> feedforward,
            HardwareMap hardwareMap
    ) {
        this.proportional = proportional;
        this.integral = integral;
        this.derivative = derivative;
        this.feedforward = feedforward;
        liftMotors[0] = hardwareMap.get(DcMotor.class, "arm0");
        liftMotors[1] = hardwareMap.get(DcMotor.class, "arm1");
    }

    long lastUpdate = -1;
    public void update() {

    }
}
