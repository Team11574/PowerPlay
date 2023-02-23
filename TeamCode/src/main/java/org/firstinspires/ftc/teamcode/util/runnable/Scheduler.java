package org.firstinspires.ftc.teamcode.util.runnable;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.function.Consumer;
import java.util.function.Function;

public class Scheduler {
    static List<Function<Void, Boolean>> globalQueries = new ArrayList<>();
    static List<Consumer<Void>> globalActions = new ArrayList<>();
    static List<Double> globalWaits = new ArrayList<>();
    static List<Double> globalStartTimes = new ArrayList<>();

    static List<Function<Void, Boolean>> linearQueries = new ArrayList<>();
    static List<Consumer<Void>> linearActions = new ArrayList<>();
    static List<Double> linearWaits = new ArrayList<>();
    static List<Double> linearStartTimes = new ArrayList<>();
    static ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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
        globalSchedule(query, action, 0);
    }

    public static void globalSchedule(Function<Void, Boolean> query, Consumer<Void> action, double wait) {
        globalQueries.add(query);
        globalActions.add(action);
        globalWaits.add(wait);
        globalStartTimes.add(-1d);
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
        linearSchedule(query, action, 0);
    }

    /**
     *
     * @param query
     * @param action
     * @param wait Wait time in milliseconds
     */
    public static void linearSchedule(Function<Void, Boolean> query, Consumer<Void> action, double wait) {
        linearQueries.add(query);
        linearActions.add(action);
        linearWaits.add(wait);
        linearStartTimes.add(-1d);
    }

    public static void update() {
        // Global
        int i = 0;
        while (i < globalQueries.size()) {
            Function<Void, Boolean> check = globalQueries.get(i);
            Consumer<Void> run = globalActions.get(i);
            double wait = globalWaits.get(i);
            if (globalStartTimes.get(i) < 0)
                globalStartTimes.set(i, timer.time());
            double startTime = globalStartTimes.get(i);

            if (timer.time() >= startTime + wait) {
                try {
                    if (check.apply(null)) {
                        run.accept(null);
                        globalQueries.remove(i);
                        globalActions.remove(i);
                        globalWaits.remove(i);
                        globalStartTimes.remove(i);
                        i--;
                    }
                } catch (Exception e) {
                    // query or action failed
                }
            }
            i++;
        }

        // Linear
        if (linearQueries.size() > 0) {
            int last_index = linearActions.size() - 1;
            Function<Void, Boolean> check = linearQueries.get(last_index);
            Consumer<Void> run = linearActions.get(last_index);
            double wait = linearWaits.get(last_index);
            if (linearStartTimes.get(last_index) < 0)
                linearStartTimes.set(last_index, timer.time());
            double startTime = linearStartTimes.get(last_index);

            if (timer.time() >= startTime + wait) {
                try {
                    if (check.apply(null)) {
                        run.accept(null);
                        linearQueries.remove(last_index);
                        linearActions.remove(last_index);
                        linearWaits.remove(last_index);
                        linearStartTimes.remove(last_index);
                    }
                } catch (Exception e) {
                    // query or action failed
                }
            }
        }
    }
}
