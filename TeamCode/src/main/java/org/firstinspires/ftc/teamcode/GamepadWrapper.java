package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.IllformedLocaleException;
import java.util.function.Predicate;

public class GamepadWrapper {
    private Gamepad gamepad;
    private Predicate<Gamepad> getValue;
    private boolean disabled = false;
    private boolean value;

    public GamepadWrapper(Gamepad gamepad, Predicate<Gamepad> getValue) {
        this.gamepad = gamepad;
        this.getValue = getValue;
        // this.value = getValue.test(gamepad);
        if (gamepad == null) {
            throw new IllformedLocaleException("ee");
        }
    }

    public boolean update() {
        boolean newValue = getValue.test(gamepad);
        if (newValue) {
            if (!disabled) {
                value = true;
                disabled = true;
            } else {
                value = false;
            }
        } else {
            disabled = false;
            value = false;
        }
        return value;
    }

    public boolean get() {
        return value;
    }
}
