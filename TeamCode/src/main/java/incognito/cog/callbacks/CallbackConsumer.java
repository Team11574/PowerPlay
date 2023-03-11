package incognito.cog.callbacks;

import java.util.function.Consumer;

public class CallbackConsumer<T> implements Callback<Consumer<T>> {
    private final Consumer<T> callback;
    private boolean cancelled = false;

    public CallbackConsumer(Consumer<T> callback) {
        this.callback = callback;
    }

    public void run() {
        run(null);
    }

    public void run(T value) {
        if (cancelled) {
            return;
        }
        callback.accept(value);
    }

    public void cancel() {
        cancelled = true;
    }

    public boolean isCancelled() {
        return cancelled;
    }
}