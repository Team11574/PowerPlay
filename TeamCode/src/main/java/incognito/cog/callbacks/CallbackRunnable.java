package incognito.cog.callbacks;

public class CallbackRunnable implements Callback<Runnable> {
    private final Runnable callback;
    private boolean cancelled = false;

    private Runnable thenCallback = null;

    public CallbackRunnable(Runnable callback) {
        this.callback = callback;
    }

    public void run() {
        if (cancelled) {
            return;
        }
        callback.run();
        if (thenCallback != null) {
            thenCallback.run();
        }
    }

    public void cancel() {
        cancelled = true;
    }

    public boolean isCancelled() {
        return cancelled;
    }

    public void then(Runnable callback) {
        thenCallback = callback;
    }
}