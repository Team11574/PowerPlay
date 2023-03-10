package incognito.cog.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

public class Scheduler {
    List<Function<Void, Boolean>> globalQueries; // = new ArrayList<>();
    List<Consumer<Void>> globalActions; // = new ArrayList<>();
    List<Double> globalWaits; // = new ArrayList<>();
    List<Double> globalStartTimes; // = new ArrayList<>();

    List<Function<Void, Boolean>> linearQueries; // = new ArrayList<>();
    List<Consumer<Void>> linearActions; // = new ArrayList<>();
    List<Double> linearWaits; // = new ArrayList<>();
    List<Double> linearStartTimes; // = new ArrayList<>();
    ElapsedTime timer; // = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public Scheduler() {
        globalQueries = new ArrayList<>();
        globalActions = new ArrayList<>();
        globalWaits = new ArrayList<>();
        globalStartTimes = new ArrayList<>();

        linearQueries = new ArrayList<>();
        linearActions = new ArrayList<>();
        linearWaits = new ArrayList<>();
        linearStartTimes = new ArrayList<>();
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    /**
     * Schedule globally queued actions.
     * <p>
     * All global scheduled actions will be checked every update cycle.
     * <p>
     * Function and Consumer inputs are always null, they simple allow for cleaner
     * syntax by saying "when" and "then" as shown in the example below.
     * <p>
     * <p>
     * Scheduler.globalSchedule(
     * when -> horizontalSlide.atSetPosition(SET_POSITION_THRESHOLD),
     * then -> horizontalClaw.open()
     * );
     *
     * @param query  The check function to query, must return a boolean.
     * @param action The action function to run once the query returns true, must return void.
     */
    public void globalSchedule(Function<Void, Boolean> query, Consumer<Void> action) {
        globalSchedule(query, action, 0);
    }

    public void globalSchedule(Function<Void, Boolean> query, Consumer<Void> action, double wait) {
        globalQueries.add(query);
        globalActions.add(action);
        globalWaits.add(wait);
        globalStartTimes.add(-1d);
    }

    /**
     * Schedule LIFO queued actions.
     * <p>
     * Function and Consumer inputs are always null, they simple allow for cleaner
     * syntax by saying "when" and "then" as shown in the example below.
     * <p>
     * <p>
     * Scheduler.linearSchedule(
     * when -> horizontalSlide.atSetPosition(SET_POSITION_THRESHOLD),
     * then -> horizontalClaw.open()
     * );
     *
     * @param query  The check function to query, must return a boolean
     * @param action The action function to run once the query returns true, must return void
     */
    public void linearSchedule(Function<Void, Boolean> query, Consumer<Void> action) {
        linearSchedule(query, action, 0);
    }

    /**
     * @param query
     * @param action
     * @param wait   Wait time in milliseconds
     */
    public void linearSchedule(Function<Void, Boolean> query, Consumer<Void> action, double wait) {
        linearQueries.add(query);
        linearActions.add(action);
        linearWaits.add(wait);
        linearStartTimes.add(-1d);
    }

    public boolean hasLinearQueries() {
        return linearQueries.size() > 0;
    }

    public int linearQueryLength() {
        return linearQueries.size();
    }

    public int globalQueryLength() {
        return globalQueries.size();
    }

    public boolean hasGlobalQueries() {
        return globalQueries.size() > 0;
    }

    public void clearLinear() {
        linearQueries.clear();
        linearActions.clear();
        linearWaits.clear();
        linearStartTimes.clear();
    }

    public void clearGlobal() {
        globalQueries.clear();
        globalActions.clear();
        globalWaits.clear();
        globalStartTimes.clear();
    }

    public void update() {
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
            Function<Void, Boolean> check = linearQueries.get(0);
            Consumer<Void> run = linearActions.get(0);
            double wait = linearWaits.get(0);
            if (linearStartTimes.get(0) < 0)
                linearStartTimes.set(0, timer.time());
            double startTime = linearStartTimes.get(0);

            if (timer.time() >= startTime + wait) {
                try {
                    if (check.apply(null)) {
                        run.accept(null);
                        linearQueries.remove(0);
                        linearActions.remove(0);
                        linearWaits.remove(0);
                        linearStartTimes.remove(0);
                    }
                } catch (Exception e) {
                    // query or action failed
                }
            }
        }
    }
}