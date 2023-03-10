package incognito.cog.actions;

import incognito.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.function.Consumer;

public class Action {
    public boolean resolved = false;
    private Consumer<Robot> action;
    public String name;

    private ArrayList<Consumer<Robot>> callbacks = new ArrayList<>();

    Action(String name, Consumer<Robot> action, Runnable completion) {
        this.name = name;
        this.action = action;
        addCallback((robot) -> completion.run());
    }

    }

    public void run(Robot robot) {
        action.accept(robot);
        resolved = true;
        dispatchCallback(robot);
    }

    // Method to see if it's completed
    public boolean resolved() {
        return resolved;
    }

    public void addCallback(Consumer<Robot> callback) {
        callbacks.add(callback);
    }

    public void dispatchCallback(Robot robot) {
        for (Consumer<Robot> callback : callbacks) {
            callback.accept(robot);
        }
    }

}