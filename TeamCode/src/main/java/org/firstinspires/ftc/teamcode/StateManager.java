package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;

public class StateManager {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    DcMotor intake;
    private DcMotorEx[] arm;
    private Servo grabber;
    private Servo wrist;
    static final long LIFT_DROP_TIMEOUT_MS = 2000;
    static final long LIFT_TICKS_THRESHOLD = 5;
    static final int LIFT_PIXEL_TICKS = 217;
    static final int LIFT_LOWEST_SCORE_TICKS = 1160 - LIFT_PIXEL_TICKS;
    static final double LIFT_HOLDING_POWER = 0.05;
    static final double LIFT_MOVING_POWER = 1;
    static final double GRABBER_START = 0.0295;
    static final double GRABBER_SINGLE = GRABBER_START + 0.0232;
    static final double GRABBER_DOUBLE = GRABBER_START + 0.122;
    static final double WRIST_START = 0.008;
    static final double WRIST_DUMP = WRIST_START + 0.26;
    static final long SERVO_WAIT_SHORT_MS = 150;
    static final long SERVO_WAIT_LONG_MS = 300;

    /*
        INTAKING - LOWERED
                      |
    SCORE_SINGLE - RAISIN - SCORE_DOUBLE
         \                       /
          -----------------------
    */
    public enum State {
        INTAKING,
        LOWERED,
        RAISIN,
        SCORE_SINGLE,
        SCORE_DOUBLE;

        public boolean isArmDown() {
            return this == INTAKING || this == LOWERED;
        }
    }

    private AtomicInteger armPixelsHigh = new AtomicInteger(1);
    public int armTicksOverride = -1;

    private int armTicks() {
        return armTicksOverride == -1
                ? Math.max(0, armPixelsHigh.get()) * LIFT_PIXEL_TICKS + LIFT_LOWEST_SCORE_TICKS
                : armTicksOverride;
    }

    public State state = State.LOWERED;
    private Queue<State> futureStates = new LinkedList<>();
    private Future<?> runnerStatus;
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
                if (position == 0) {
                    // take the opportunity to recalibrate encoders
                    // TODO PEE
                    // m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    m.setPower(LIFT_HOLDING_POWER); // TODO fix motor fighting
                } else {
                    m.setPower(LIFT_HOLDING_POWER);
                }
            }
        } catch (InterruptedException ignored) {}
    }

    private void setServoBlocking(Servo servo, double position, long wait) {
        servo.setPosition(position);
        try {
            Thread.sleep(wait);
        } catch (InterruptedException ignored) {
        }
    }

    private Runnable sequence(Runnable first, Runnable second) {
        return () -> {
            first.run();
            second.run();
        };
    }

    Map<State, Map<State, Runnable>> transitions;

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

        transitions = new EnumMap<>(State.class);
        Map<State, Map<State, Integer>> dists = new EnumMap<>(State.class);

        Map<State, Runnable> targets = new EnumMap<>(State.class);
        targets.put(State.INTAKING, () -> {});
        targets.put(State.LOWERED, () -> {
            intake.setPower(0);
            setServoBlocking(grabber, GRABBER_START, SERVO_WAIT_SHORT_MS);
        });
        transitions.put(State.INTAKING, targets);

        targets = new EnumMap<>(State.class);
        targets.put(State.LOWERED, () -> {});
        targets.put(State.INTAKING, () -> {
            intake.setPower(-1);
            setServoBlocking(grabber, GRABBER_DOUBLE, SERVO_WAIT_SHORT_MS);
        });
        targets.put(State.RAISIN, () -> setArmBlocking(armTicks()));
        transitions.put(State.LOWERED, targets);

        targets = new EnumMap<>(State.class);
        targets.put(State.RAISIN, () -> {});
        targets.put(State.LOWERED, () -> setArmBlocking(0));
        targets.put(State.SCORE_SINGLE, () -> {
            setServoBlocking(wrist, WRIST_DUMP, SERVO_WAIT_LONG_MS);
            setServoBlocking(grabber, GRABBER_SINGLE, SERVO_WAIT_SHORT_MS);
        });
        targets.put(State.SCORE_DOUBLE, () -> {
            setServoBlocking(wrist, WRIST_DUMP, SERVO_WAIT_LONG_MS);
            setServoBlocking(grabber, GRABBER_DOUBLE, SERVO_WAIT_SHORT_MS);
        });
        transitions.put(State.RAISIN, targets);

        targets = new EnumMap<>(State.class);
        targets.put(State.SCORE_SINGLE, () -> {});
        targets.put(State.RAISIN, () -> {
            wrist.setPosition(WRIST_START);
            grabber.setPosition(GRABBER_START);
            try {
                Thread.sleep(SERVO_WAIT_LONG_MS);
            } catch (InterruptedException ignored) {
            }
        });
        targets.put(State.SCORE_DOUBLE,
                () -> setServoBlocking(grabber, GRABBER_DOUBLE, SERVO_WAIT_SHORT_MS));
        transitions.put(State.SCORE_SINGLE, targets);

        targets = new EnumMap<>(State.class);
        targets.put(State.SCORE_DOUBLE, () -> {});
        targets.put(State.RAISIN, () -> {
            wrist.setPosition(WRIST_START);
            grabber.setPosition(GRABBER_START);
            try {
                Thread.sleep(SERVO_WAIT_LONG_MS);
            } catch (InterruptedException ignored) {
            }
        });
        targets.put(State.SCORE_SINGLE,
                () -> setServoBlocking(grabber, GRABBER_SINGLE, SERVO_WAIT_SHORT_MS));
        transitions.put(State.SCORE_DOUBLE, targets);

        for (Map.Entry<State, Map<State, Runnable>> target : transitions.entrySet()) {
            State from = target.getKey();
            Map<State, Integer> targetDists = new EnumMap<>(State.class);
            for (Map.Entry<State, Runnable> transition : target.getValue().entrySet()) {
                State to = transition.getKey();
                targetDists.put(to, from == to ? 0 : 1);
            }
            dists.put(from, targetDists);
        }

        for (State intermediary : State.values()) {
            for (State from : State.values()) {
                for (State to : State.values()) {
                    int currDist = dists.get(from)
                            .getOrDefault(to, Integer.MAX_VALUE - 1);
                    int newDist = Optional.ofNullable(dists.get(from).get(intermediary))
                            .flatMap(fi -> Optional.ofNullable(dists.get(intermediary).get(to))
                                    .map(it -> fi + it))
                            .orElse(Integer.MAX_VALUE);
                    if (newDist < currDist) {
                        dists.get(from).put(to, newDist);
                        transitions.get(from).put(to,
                                sequence(transitions.get(from).get(intermediary),
                                        transitions.get(intermediary).get(to)));
                    }
                }
            }
        }
    }

    public void queueState(State newState) {
        futureStates.add(newState);
    }

    public void raiseArm() {
        if (state.isArmDown()) {
            queueState(State.RAISIN);
        } else {
            armPixelsHigh.incrementAndGet();
        }
    }

    public void lowerArm() {
        if (!state.isArmDown()) {
            if (armPixelsHigh.getAndUpdate(x -> x <= 1 ? 1 : x - 1) <= 1) {
                queueState(State.LOWERED);
            }
        }
    }

    public void update() {
        telemetry.addData("arm0At", arm[0].getCurrentPosition());
        // telemetry.addData("arm1At", arm[1].getCurrentPosition());
        telemetry.addData("arm0", arm[0].getTargetPosition());
        // telemetry.addData("arm1", arm[1].getTargetPosition());
        State nextState = state;
        boolean changed = false;
        if (!futureStates.isEmpty() && !isBusy()) {
            nextState = futureStates.poll();
            changed = true;
        }
        Runnable transition = transitions.get(state).get(nextState);
        state = nextState;
        telemetry.addData("busy", isBusy());
        if (!isBusy() && !state.isArmDown()) {
            int targetPos = armTicks();
            boolean busy = isArmBusy();
            for (DcMotor m : arm) {
                m.setTargetPosition(targetPos);
                m.setPower(busy ? LIFT_MOVING_POWER : LIFT_HOLDING_POWER);
            }
        }
        // this has to go after motor set target positions
        // otherwise waitToIdle on the arm finishes too fast because the target position is old
        if (changed) { // without guard, status constantly reset -> never busy
            runnerStatus = runner.submit(transition);
        }

        synchronized (ticker) {
            ticker.notifyAll();
        }
    }

    public boolean isBusy() {
        return runnerStatus != null && !runnerStatus.isDone();
    }
}
