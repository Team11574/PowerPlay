package org.firstinspires.ftc.teamcode.util.runnable;

import android.os.Build;

import androidx.annotation.RequiresApi;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.function.Consumer;
import java.util.function.Function;

public class Scheduler {
    List<Function<Void, Boolean>> whens;
    List<Consumer<Void>> thens;

    public Scheduler() {
        whens = new ArrayList<>();
        thens = new ArrayList<>();
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void schedule(Function<Void, Boolean> when, Consumer<Void> then) {
        whens.add(when);
        thens.add(then);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void update() {
        int i = 0;
        while (i < whens.size()) {
            Function<Void, Boolean> check = whens.get(i);
            Consumer<Void> run = thens.get(i);
            try {
                if (check.apply(null)) {
                    run.accept(null);
                    whens.remove(i);
                    thens.remove(i);
                    i--;
                }
            } catch (Exception e) {
                // check or run didn't work
            }
            i++;
        }
    }
}
