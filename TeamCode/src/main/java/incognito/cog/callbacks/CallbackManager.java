package incognito.cog.callbacks;

public interface CallbackManager<EventType, CallbackType> {
    public Callback<CallbackType> registerCallback(EventType event, CallbackType callback);

    public void dispatchCallback(EventType event);
}