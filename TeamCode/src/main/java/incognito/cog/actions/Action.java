package incognito.cog.actions;

import incognito.cog.callbacks.CallbackManagerRunnable;

public class Action {
    public boolean resolved = false;
    private final Runnable action;
    public String name;

    private final CallbackManagerRunnable<Event> callbacks = new CallbackManagerRunnable<Event>(Event.values());

    Action(String name, Runnable action) {
        this.name = name;
        this.action = action;
    }

    public void run() {
        action.run();
        resolved = true;
        dispatchCallback(Event.RESOLVED);
    }

    // Method to see if it's completed
    public boolean resolved() {
        return resolved;
    }

    protected void setResolved(boolean resolved) {
        this.resolved = resolved;
        dispatchCallback(Event.RESOLVED);
    }

    public void registerCallback(Event event, Runnable callback) {
        callbacks.registerCallback(event, callback);
    }

    public void dispatchCallback(Event event) {
        callbacks.dispatchCallback(event);
    }

    public enum Event {
        RUN, RESOLVED
    }

}