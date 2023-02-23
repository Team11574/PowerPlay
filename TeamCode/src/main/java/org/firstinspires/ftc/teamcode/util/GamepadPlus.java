package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadPlus {
    public float last_left_stick_x = 0f;
    public float last_left_stick_y = 0f;
    public float last_right_stick_x = 0f;
    public float last_right_stick_y = 0f;
    public boolean last_dpad_up = false;
    public boolean last_dpad_down = false;
    public boolean last_dpad_left = false;
    public boolean last_dpad_right = false;
    public boolean last_a = false;
    public boolean last_b = false;
    public boolean last_x = false;
    public boolean last_y = false;
    public boolean last_left_bumper = false;
    public boolean last_right_bumper = false;
    public boolean last_left_stick_button = false;
    public boolean last_right_stick_button = false;
    public float last_left_trigger = 0f;
    public float last_right_trigger = 0f;

    public Gamepad gamepad;

    public GamepadPlus(Gamepad pad) {
        gamepad = pad;
    }

    public float get_partitioned_left_stick_y() {
        double theta = Math.atan2(gamepad.left_stick_y, gamepad.left_stick_x);
        if (Math.abs(theta) < Math.PI/6) {
            return 0;
        }
        return gamepad.left_stick_y;
    }

    public float get_partitioned_left_stick_x() {
        double theta = Math.atan2(gamepad.left_stick_y, gamepad.left_stick_x);
        if (Math.abs(theta) > Math.PI/3) {
            return 0;
        }
        return gamepad.left_stick_x;
    }

    public float get_partitioned_right_stick_y() {
        double theta = Math.atan2(gamepad.right_stick_y, gamepad.right_stick_x);
        if (Math.abs(theta) < Math.PI/6) {
            return 0;
        }
        return gamepad.right_stick_y;
    }

    public float get_partitioned_right_stick_x() {
        double theta = Math.atan2(gamepad.right_stick_y, gamepad.right_stick_x);
        if (Math.abs(theta) > Math.PI/3) {
            return 0;
        }
        return gamepad.right_stick_x;
    }

    public boolean right_trigger_pressed() { return gamepad.right_trigger > 0; };

    public boolean left_trigger_pressed() { return gamepad.left_trigger > 0; };

    public boolean dpad_up_pressed() {
        return (gamepad.dpad_up && !last_dpad_up);
    }

    public boolean dpad_down_pressed() {
        return (gamepad.dpad_down && !last_dpad_down);
    }

    public boolean dpad_left_pressed() {
        return (gamepad.dpad_left && !last_dpad_left);
    }

    public boolean dpad_right_pressed() {
        return (gamepad.dpad_right && !last_dpad_right);
    }

    public boolean a_pressed() {
        return (gamepad.a && !last_a);
    }

    public boolean b_pressed() {
        return (gamepad.b && !last_b);
    }

    public boolean x_pressed() {
        return (gamepad.x && !last_x);
    }

    public boolean y_pressed() {
        return (gamepad.y && !last_y);
    }

    public boolean left_bumper_pressed() {
        return (gamepad.left_bumper && !last_left_bumper);
    }

    public boolean right_bumper_pressed() {
        return (gamepad.right_bumper && !last_right_bumper);
    }

    public boolean left_stick_button_pressed() {
        return (gamepad.left_stick_button && !last_left_stick_button);
    }

    public boolean right_stick_button_pressed() {
        return (gamepad.right_stick_button && !last_right_stick_button);
    }

    public void update() {
        last_left_stick_x = gamepad.left_stick_x;
        last_left_stick_y = gamepad.left_stick_y;
        last_right_stick_x = gamepad.right_stick_x;
        last_right_stick_y = gamepad.right_stick_y;
        last_dpad_up = gamepad.dpad_up;
        last_dpad_down = gamepad.dpad_down;
        last_dpad_left = gamepad.dpad_left;
        last_dpad_right = gamepad.dpad_right;
        last_a = gamepad.a;
        last_b = gamepad.b;
        last_x = gamepad.x;
        last_y = gamepad.y;
        last_left_bumper = gamepad.left_bumper;
        last_right_bumper = gamepad.right_bumper;
        last_left_stick_button = gamepad.left_stick_button;
        last_right_stick_button = gamepad.right_stick_button;
        last_left_trigger = gamepad.left_trigger;
        last_right_trigger = gamepad.right_trigger;
    }

}
