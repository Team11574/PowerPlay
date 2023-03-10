package incognito.cog.control;

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

    public boolean dpad_up_pressed = false;
    public boolean dpad_down_pressed = false;
    public boolean dpad_left_pressed = false;
    public boolean dpad_right_pressed = false;
    public boolean a_pressed = false;
    public boolean b_pressed = false;
    public boolean x_pressed = false;
    public boolean y_pressed = false;
    public boolean left_bumper_pressed = false;
    public boolean right_bumper_pressed = false;
    public boolean left_stick_button_pressed = false;
    public boolean right_stick_button_pressed = false;
    public boolean left_trigger_pressed = false;
    public boolean right_trigger_pressed = false;


    public Gamepad gamepad;

    public GamepadPlus(Gamepad pad) {
        gamepad = pad;
    }

    public float get_partitioned_left_stick_y() {
        double theta = Math.atan2(gamepad.left_stick_y, gamepad.left_stick_x);
        if (Math.abs(theta) < Math.PI / 6) {
            return 0;
        }
        return gamepad.left_stick_y;
    }

    public float get_partitioned_left_stick_x() {
        double theta = Math.atan2(gamepad.left_stick_y, gamepad.left_stick_x);
        if (Math.abs(theta - Math.PI / 2) < Math.PI / 6) {
            return 0;
        }

        return gamepad.left_stick_x;
    }

    public float get_partitioned_right_stick_y() {
        double theta = Math.atan2(gamepad.right_stick_y, gamepad.right_stick_x);
        if (Math.abs(theta) < Math.PI / 6) {
            return 0;
        }
        return gamepad.right_stick_y;
    }

    public float get_partitioned_right_stick_x() {
        double theta = Math.atan2(gamepad.right_stick_y, gamepad.right_stick_x);
        if (Math.abs(theta - Math.PI / 2) < Math.PI / 6) {
            return 0;
        }
        return gamepad.right_stick_x;
    }

    public double right_stick_angle() {
        return Math.atan2(-gamepad.right_stick_y, gamepad.right_stick_x);
    }

    public double left_stick_angle() {
        return Math.atan2(-gamepad.left_stick_y, gamepad.left_stick_x);
    }

    /**
     * Returns the octant of the right stick,
     * where 0 is from -pi/8 to pi/8, 1 is from pi/8 to 3pi/8, etc.
     *
     * @return the octant of the right stick
     */
    public int right_stick_octant() {
        if (gamepad.right_stick_x == 0 && gamepad.right_stick_y == 0) {
            return -1;
        }
        double shift = Math.PI / 8;
        double theta = right_stick_angle();
        if (theta < 0) {
            theta += 2 * Math.PI;
        }
        return (int) Math.ceil((theta + shift) / (Math.PI / 4) - 1) % 8;
    }

    /**
     * Returns the octant of the left stick,
     * where 0 is from -pi/8 to pi/8, 1 is from pi/8 to 3pi/8, etc.
     *
     * @return the octant of the left stick
     */
    public int left_stick_octant() {
        if (gamepad.left_stick_x == 0 && gamepad.left_stick_y == 0) {
            return -1;
        }
        double shift = Math.PI / 8;
        double theta = left_stick_angle();
        if (theta < 0) {
            theta += 2 * Math.PI;
        }
        return (int) Math.ceil((theta + shift) / (Math.PI / 4) - 1) % 8;
    }

    public boolean right_trigger_active() {
        return gamepad.right_trigger > 0;
    }

    public boolean left_trigger_active() {
        return gamepad.left_trigger > 0;
    }

    public void update() {
        last_left_stick_x = gamepad.left_stick_x;
        last_left_stick_y = gamepad.left_stick_y;
        last_right_stick_x = gamepad.right_stick_x;
        last_right_stick_y = gamepad.right_stick_y;
        left_trigger_pressed = last_left_trigger == 0 && gamepad.left_trigger > last_left_trigger;
        last_left_trigger = gamepad.left_trigger;
        right_trigger_pressed = last_right_trigger == 0 && gamepad.right_trigger > last_right_trigger;
        last_right_trigger = gamepad.right_trigger;

        dpad_up_pressed = gamepad.dpad_up && !last_dpad_up;
        last_dpad_up = gamepad.dpad_up;

        dpad_down_pressed = gamepad.dpad_down && !last_dpad_down;
        last_dpad_down = gamepad.dpad_down;

        dpad_left_pressed = gamepad.dpad_left && !last_dpad_left;
        last_dpad_left = gamepad.dpad_left;

        dpad_right_pressed = gamepad.dpad_right && !last_dpad_right;
        last_dpad_right = gamepad.dpad_right;

        a_pressed = gamepad.a && !last_a;
        last_a = gamepad.a;

        b_pressed = gamepad.b && !last_b;
        last_b = gamepad.b;

        x_pressed = gamepad.x && !last_x;
        last_x = gamepad.x;

        y_pressed = gamepad.y && !last_y;
        last_y = gamepad.y;

        left_bumper_pressed = gamepad.left_bumper && !last_left_bumper;
        last_left_bumper = gamepad.left_bumper;

        right_bumper_pressed = gamepad.right_bumper && !last_right_bumper;
        last_right_bumper = gamepad.right_bumper;

        left_stick_button_pressed = gamepad.left_stick_button && !last_left_stick_button;
        last_left_stick_button = gamepad.left_stick_button;

        right_stick_button_pressed = gamepad.right_stick_button && !last_right_stick_button;
        last_right_stick_button = gamepad.right_stick_button;
    }

}