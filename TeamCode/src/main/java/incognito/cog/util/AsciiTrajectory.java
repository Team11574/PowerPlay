package incognito.cog.util;

import java.util.HashMap;

public class AsciiTrajectory {
    HashMap<String, String> chars = new HashMap<String, String>();
    int GRID_SIZE = 10;
    int RATIO_MODIFIER = 4;

    AsciiTrajectory() {
        chars.put("b_horizontal", "═");
        chars.put("b_vertical", "║");
        chars.put("b_top_left", "╔");
        chars.put("b_top_right", "╗");
        chars.put("b_bottom_left", "╚");
        chars.put("b_bottom_right", "╝");

        chars.put("blank", " ");
        chars.put("j_high", "H");
        chars.put("j_medium", "M");
        chars.put("j_low", "L");
        chars.put("j_ground", "G");

        chars.put("m_up_start", "╵");
        chars.put("m_up_end", "↑");

        chars.put("m_down_start", "╷");
        chars.put("m_down_end", "↓");

        chars.put("m_horizontal", "─");
        chars.put("m_vertical", "│");

        chars.put("m_left_start", "╴");
        chars.put("m_left_end", "←");

        chars.put("m_right_start", "╶");
        chars.put("m_right_end", "→");

        chars.put("m_up_left", "╮");
        chars.put("m_up_right", "╭");
        chars.put("m_down_left", "╯");
        chars.put("m_down_right", "╰");
    }

    public void printGrid() {
        // Print top border, taking into account the ratio modifier
        // (multiply width characters by ratio modifier)
        System.out.print(chars.get("b_top_left"));
        for (int i = 0; i < GRID_SIZE * RATIO_MODIFIER; i++) {
            System.out.print(chars.get("b_horizontal"));
        }
        System.out.println(chars.get("b_top_right"));


        // Print grid
        for (int i = 0; i < GRID_SIZE; i++) {
            System.out.print(chars.get("b_vertical"));
            for (int j = 0; j < GRID_SIZE * RATIO_MODIFIER; j++) {
                System.out.print(chars.get("blank"));
            }
            System.out.println(chars.get("b_vertical"));
        }

        // Print bottom border
        System.out.print(chars.get("b_bottom_left"));
        for (int i = 0; i < GRID_SIZE * RATIO_MODIFIER; i++) {
            System.out.print(chars.get("b_horizontal"));
        }
        System.out.println(chars.get("b_bottom_right"));
    }

    public static void main(String[] args) {
        AsciiTrajectory asciiTrajectory = new AsciiTrajectory();
        asciiTrajectory.printGrid();
    }
}