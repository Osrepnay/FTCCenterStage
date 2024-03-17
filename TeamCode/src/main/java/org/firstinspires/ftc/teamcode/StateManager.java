package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.UnaryOperator;

public class StateManager {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    DcMotor intake;
    private DcMotorEx[] arm;
    private Servo grabber;
    private Servo wrist;
    private Servo intakeLift;
    static final long LIFT_DROP_TIMEOUT_MS = 2000;
    static final long LIFT_TICKS_THRESHOLD = 5;
    static final int LIFT_PIXEL_TICKS = 217;
    static final int LIFT_LOWEST_SCORE_TICKS = 1160 - LIFT_PIXEL_TICKS;
    static final double LIFT_HOLDING_POWER = 0.05;
    static final double LIFT_MOVING_POWER = 1;
    static final double GRABBER_START = 0.0295;
    static final double GRABBER_SINGLE = GRABBER_START + 0.0232;
    static final double GRABBER_DOUBLE = GRABBER_START + 0.122;
    static final double WRIST_START = 0.26;
    static final double WRIST_DUMP = WRIST_START + 0.263;
    static final long SERVO_WAIT_SHORT_MS = 150;
    static final long SERVO_WAIT_LONG_MS = 300;

    static final double INTAKE_LIFT_FLAT = 0;
    static final double INTAKE_LIFT_POS_PER_RAD = 11.5 / 36 / (Math.PI / 2);
    static final double PIXEL_HEIGHT_RATIO = 0.5 / (172 / 25.4);
    static final double[] INTAKE_LIFT_POSITIONS = {
            INTAKE_LIFT_FLAT + INTAKE_LIFT_POS_PER_RAD * (Math.asin(PIXEL_HEIGHT_RATIO * 0) - 0.05),
            INTAKE_LIFT_FLAT + INTAKE_LIFT_POS_PER_RAD * (Math.asin(PIXEL_HEIGHT_RATIO * 1) - 0.05),
            INTAKE_LIFT_FLAT + INTAKE_LIFT_POS_PER_RAD * (Math.asin(PIXEL_HEIGHT_RATIO * 2) - 0.05),
            INTAKE_LIFT_FLAT + INTAKE_LIFT_POS_PER_RAD * (Math.asin(PIXEL_HEIGHT_RATIO * 3) - 0.05),
            INTAKE_LIFT_FLAT + INTAKE_LIFT_POS_PER_RAD * (Math.asin(PIXEL_HEIGHT_RATIO * 4) - 0.05),
            INTAKE_LIFT_FLAT + INTAKE_LIFT_POS_PER_RAD * (Math.asin(PIXEL_HEIGHT_RATIO * 5) - 0.05),
            INTAKE_LIFT_FLAT + INTAKE_LIFT_POS_PER_RAD * (Math.PI / 2)
    };

    /*
        INTAKING - LOWERED
                      |
    SCORE_SINGLE - RAISIN - SCORE_DOUBLE
         \                       /
          -----------------------
    */

    public static class StateProps {
        public enum WristState {
            WRIST_IN, WRIST_OUT
        }

        public enum GrabberState {
            GRABBER_GRAB, GRABBER_ONE, GRABBER_TWO
        }

        public double intakePower;
        public int liftHeight;
        public WristState wristState;
        public GrabberState grabberState;
        public int extendoIdx;

        public StateProps(double intakePower, int liftHeight, WristState wristState, GrabberState grabberState,
                          int extendoIdx) {
            this.intakePower = intakePower;
            this.liftHeight = liftHeight;
            this.wristState = wristState;
            this.grabberState = grabberState;
            this.extendoIdx = extendoIdx;
        }
    }

    private int liftTicks(int liftHeight) {
        return Math.min(2850, liftHeight == 0 ? 0 : liftHeight * LIFT_PIXEL_TICKS + LIFT_LOWEST_SCORE_TICKS);
    }

    private Runnable transitionProps(StateProps from, StateProps to) {
        Runnable run = () -> {};
        // collapse bucket while going down or up
        if (from.liftHeight == 0 && to.liftHeight != 0
                || from.liftHeight != 0 && to.liftHeight == 0) {
            run = sequence(run, () -> {
                setServoBlocking(wrist, WRIST_START, 0);
                setServoBlocking(grabber, GRABBER_START, SERVO_WAIT_LONG_MS);
            });
        }
        if (from.liftHeight != to.liftHeight) {
            run = sequence(run, () -> setArmBlocking(liftTicks(to.liftHeight)));
        }
        if (from.wristState != to.wristState) {
            double wristPos;
            switch (to.wristState) {
                case WRIST_IN:
                    wristPos = WRIST_START;
                    break;
                case WRIST_OUT:
                    wristPos = WRIST_DUMP;
                    break;
                default:
                    throw new IllegalStateException("didn't catch " + to.wristState);
            }
            run = sequence(run, () -> setServoBlocking(wrist, wristPos, SERVO_WAIT_LONG_MS));
        }
        if (from.grabberState != to.grabberState) {
            double grabberPos;
            switch (to.grabberState) {
                case GRABBER_GRAB:
                    grabberPos = GRABBER_START;
                    break;
                case GRABBER_ONE:
                    grabberPos = GRABBER_SINGLE;
                    break;
                case GRABBER_TWO:
                    grabberPos = GRABBER_DOUBLE;
                    break;
                default:
                    throw new IllegalStateException("didn't catch " + to.grabberState);
            }
            run = sequence(run, () -> setServoBlocking(grabber, grabberPos, SERVO_WAIT_SHORT_MS));
        }
        if (from.intakePower != to.intakePower) {
            run = sequence(run, () -> intake.setPower(to.intakePower));
        }
        telemetry.addData("amongus sus before", from.extendoIdx);
        telemetry.addData("amongus sus after", from.extendoIdx);
        if (from.extendoIdx != to.extendoIdx) {
            run = sequence(run, () -> intakeLift.setPosition(INTAKE_LIFT_POSITIONS[to.extendoIdx]));
        }
        return run;
    }

    public void propsTo(UnaryOperator<StateProps> change) {
        StateProps to = change.apply(stateProps);
        runner.submit(transitionProps(stateProps, to));
        stateProps = to;
    }

    public enum State {
        INTAKING,
        LOWERED,
        RAISIN,
        SCORE_SINGLE,
        SCORE_DOUBLE;

        public StateProps props() {
            switch (this) {
                case INTAKING:
                    return new StateProps(-1, 0, StateProps.WristState.WRIST_IN, StateProps.GrabberState.GRABBER_TWO,
                            INTAKE_LIFT_POSITIONS.length - 1);
                case LOWERED:
                    return new StateProps(0, 0, StateProps.WristState.WRIST_IN, StateProps.GrabberState.GRABBER_GRAB,
                            INTAKE_LIFT_POSITIONS.length - 1);
                case RAISIN:
                    return new StateProps(0, 1, StateProps.WristState.WRIST_OUT, StateProps.GrabberState.GRABBER_GRAB
                            , INTAKE_LIFT_POSITIONS.length - 1);
                case SCORE_SINGLE:
                    return new StateProps(0, 1, StateProps.WristState.WRIST_OUT, StateProps.GrabberState.GRABBER_ONE,
                            INTAKE_LIFT_POSITIONS.length - 1);
                case SCORE_DOUBLE:
                    return new StateProps(0, 1, StateProps.WristState.WRIST_OUT, StateProps.GrabberState.GRABBER_TWO,
                            INTAKE_LIFT_POSITIONS.length - 1);
                default:
                    throw new IllegalStateException("missed " + this);
            }
        }
    }

    public State state = State.LOWERED;
    public StateProps stateProps = state.props();
    private List<Future<?>> runnerStatus = new ArrayList<>();
    private ExecutorService runner = Executors.newFixedThreadPool(1);
    private final Integer ticker = 0;

    private boolean isArmBusy() {
        return Arrays.stream(arm)
                .allMatch(m -> Math.abs(m.getCurrentPosition() - m.getTargetPosition())
                        >= LIFT_TICKS_THRESHOLD);
    }

    private void setArmBlocking(int position) {
        for (DcMotor m : arm) {
            m.setTargetPosition(position);
            m.setPower(LIFT_MOVING_POWER);
        }
        long start = System.currentTimeMillis();
        try {
            while (isArmBusy() && System.currentTimeMillis() - start < LIFT_DROP_TIMEOUT_MS) {
                synchronized (ticker) {
                    ticker.wait();
                }
            }
            for (DcMotor m : arm) {
                m.setPower(LIFT_HOLDING_POWER);
            }
        } catch (InterruptedException ignored) {}
    }

    private void setServoBlocking(Servo servo, double position, long wait) {
        if (servo.getPosition() != position) {
            servo.setPosition(position);
            try {
                Thread.sleep(wait);
            } catch (InterruptedException ignored) {}
        }
    }

    private Runnable sequence(Runnable first, Runnable second) {
        return () -> {
            first.run();
            second.run();
        };
    }

    public StateManager(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        intake = hardwareMap.get(DcMotor.class, "intake");

        arm = new DcMotorEx[1];
        arm[0] = hardwareMap.get(DcMotorEx.class, "arm0");
        // arm[1] = hardwareMap.get(DcMotorEx.class, "arm1");
        for (DcMotorEx m : arm) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(0);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(LIFT_HOLDING_POWER);
            m.setTargetPositionTolerance((int) LIFT_TICKS_THRESHOLD);
        }

        grabber = hardwareMap.get(Servo.class, "grabber");
        grabber.setPosition(GRABBER_START);

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        wrist.setPosition(WRIST_START);

        intakeLift = hardwareMap.get(Servo.class, "intakeLift");
        intakeLift.setPosition(INTAKE_LIFT_POSITIONS[stateProps.extendoIdx]);
    }

    public void queueState(State newState) {
        // both raised, defer to old height
        StateProps props = newState.props();
        if (props.liftHeight != 0 && stateProps.liftHeight != 0) {
            props.liftHeight = stateProps.liftHeight;
        }
        propsTo(_p -> props);
    }

    public void raiseArm() {
        if (stateProps.liftHeight == 0) {
            queueState(State.RAISIN);
        } else {
            propsTo(p -> new StateProps(p.intakePower, p.liftHeight + 1, p.wristState, p.grabberState, p.extendoIdx));
        }
    }

    public void lowerArm() {
        if (stateProps.liftHeight != 0 && stateProps.liftHeight > 0) {
            propsTo(p -> new StateProps(p.intakePower, p.liftHeight - 1, p.wristState, p.grabberState, p.extendoIdx));
        }
    }

    public void raiseExtendo() {
        propsTo(p -> {
            int newIdx = p.extendoIdx + 1;
            if (newIdx >= INTAKE_LIFT_POSITIONS.length) {
                newIdx = p.extendoIdx;
            }
            return new StateProps(p.intakePower, p.liftHeight, p.wristState, p.grabberState, newIdx);
        });
    }

    public void lowerExtendo() {
        propsTo(p -> {
            int newIdx = p.extendoIdx - 1;
            if (newIdx < 0) {
                newIdx = INTAKE_LIFT_POSITIONS.length - 1;
            }
            return new StateProps(p.intakePower, p.liftHeight, p.wristState, p.grabberState, newIdx);
        });
    }

    public void update() {
        runnerStatus.removeIf(Future::isDone);

        synchronized (ticker) {
            ticker.notifyAll();
        }
    }

    public boolean isBusy() {
        return runnerStatus.stream().anyMatch(f -> !f.isDone());
    }
}
