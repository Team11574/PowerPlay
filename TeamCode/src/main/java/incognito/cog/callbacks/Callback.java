package incognito.cog.callbacks;

public interface Callback<CallbackType> {
    void run();

    void cancel();

    boolean isCancelled();

    void then(CallbackType callback);
}