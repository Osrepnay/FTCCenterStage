package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
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

public class StateManager {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    private DcMotor intake;
    private DcMotor[] arm;
    private Servo grabber;
    private Servo wrist;
    static final long LIFT_DROP_TIMEOUT_MS = 2000;
    static final long LIFT_TICKS_THRESHOLD = 5;
    static final int LIFT_PIXEL_TICKS = 217;
    static final int LIFT_LOWEST_SCORE_TICKS = 1083 - LIFT_PIXEL_TICKS;
    static final double GRABBER_START = 0.025;
    static final double GRABBER_SINGLE = GRABBER_START + 0.0262;
    static final double GRABBER_DOUBLE = GRABBER_START + 0.125;
    static final double WRIST_START = 0.008;
    static final double WRIST_DUMP = WRIST_START + 0.257;
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

    private int armPixelsHigh = 1;

    public State state = State.LOWERED;
    private Queue<State> futureStates = new LinkedList<>();
    private Future<?> runnerStatus;
    private ExecutorService runner = Executors.newFixedThreadPool(1);
    private final Integer ticker = 0;

    private void waitToArmIdle() {
        long start = System.currentTimeMillis();
        try {
            while (Arrays.stream(arm)
                    .allMatch(m -> Math.abs(m.getCurrentPosition() - m.getTargetPosition())
                            >= LIFT_TICKS_THRESHOLD)
                    && System.currentTimeMillis() - start < LIFT_DROP_TIMEOUT_MS) {
                synchronized (ticker) {
                    ticker.wait();
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

        arm = new DcMotor[2];
        arm[0] = hardwareMap.get(DcMotor.class, "arm0");
        arm[1] = hardwareMap.get(DcMotor.class, "arm1");
        for (DcMotor m : arm) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(0);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(1);
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
        targets.put(State.RAISIN, () -> {
            if (armPixelsHigh < 1) {
                armPixelsHigh = 1;
            }
            waitToArmIdle();
        });
        transitions.put(State.LOWERED, targets);

        targets = new EnumMap<>(State.class);
        targets.put(State.RAISIN, () -> {});
        targets.put(State.LOWERED, this::waitToArmIdle);
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
            armPixelsHigh++;
        }
    }

    public void lowerArm() {
        if (state.isArmDown()) {
            queueState(State.RAISIN);
        } else {
            if (armPixelsHigh <= 1) {
                queueState(State.LOWERED);
            } else {
                armPixelsHigh--;
            }
        }
    }

    public void update() {
        State nextState = state;
        boolean changed = false;
        if (!futureStates.isEmpty() && !isBusy()) {
            nextState = futureStates.poll();
            changed = true;
        }
        Runnable transition = transitions.get(state).get(nextState);
        state = nextState;
        telemetry.addData("busy", isBusy());
        int targetPos = state.isArmDown()
                ? 0
                : armPixelsHigh == 0
                ? 0
                : armPixelsHigh * LIFT_PIXEL_TICKS + LIFT_LOWEST_SCORE_TICKS;
        for (DcMotor m : arm) {
            m.setTargetPosition(targetPos);
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
