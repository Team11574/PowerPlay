package org.firstinspires.ftc.teamcode.util.runnable;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.function.Consumer;
import java.util.function.Function;

public class Scheduler {
    static List<Function<Void, Boolean>> globalQueries = new ArrayList<>();
    static List<Consumer<Void>> globalActions = new ArrayList<>();

    static List<Function<Void, Boolean>> linearQueries = new ArrayList<>();
    static List<Consumer<Void>> linearActions = new ArrayList<>();

    /**
     *
     * Schedule globally queued actions.
     *
     * All global scheduled actions will be checked every update cycle.
     *
     * Function and Consumer inputs are always null, they simple allow for cleaner
     * syntax by saying "when" and "then" as shown in the example below.
     *
     *
     * Scheduler.globalSchedule(
     *     when -> horizontalSlide.atSetPosition(SET_POSITION_THRESHOLD),
     *     then -> horizontalClaw.open()
     * );
     *
     * @param query The check function to query, must return a boolean.
     * @param action The action function to run once the query returns true, must return void.
     */
    public static void globalSchedule(Function<Void, Boolean> query, Consumer<Void> action) {
        globalQueries.add(query);
        globalActions.add(action);
    }

    /**
     * Schedule LIFO queued actions.
     *
     * Function and Consumer inputs are always null, they simple allow for cleaner
     * syntax by saying "when" and "then" as shown in the example below.
     *
     *
     * Scheduler.linearSchedule(
     *     when -> horizontalSlide.atSetPosition(SET_POSITION_THRESHOLD),
     *     then -> horizontalClaw.open()
     * );
     *
     * @param query The check function to query, must return a boolean
     * @param action The action function to run once the query returns true, must return void
     */
    public static void linearSchedule(Function<Void, Boolean> query, Consumer<Void> action) {
        linearQueries.add(query);
        linearActions.add(action);
    }

    public static void update() {
        // Global
        int i = 0;
        while (i < globalQueries.size()) {
            Function<Void, Boolean> check = globalQueries.get(i);
            Consumer<Void> run = globalActions.get(i);
            try {
                if (check.apply(null)) {
                    run.accept(null);
                    globalQueries.remove(i);
                    globalActions.remove(i);
                    i--;
                }
            } catch (Exception e) {
                // query or action failed
            }
            i++;
        }

        // Linear
        if (linearQueries.size() > 0) {
            Function<Void, Boolean> check = linearQueries.get(linearQueries.size() - 1);
            Consumer<Void> run = linearActions.get(linearActions.size() - 1);
            try {
                if (check.apply(null)) {
                    run.accept(null);
                    linearQueries.remove(linearQueries.size() - 1);
                    linearActions.remove(linearActions.size() - 1);
                }
            } catch (Exception e) {
                // query or action failed
            }
        }
    }
}
