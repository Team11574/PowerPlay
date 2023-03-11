package incognito.cog.callbacks;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class CallbackManagerRunnable<EventType> implements CallbackManager<EventType, Runnable> {
    private final HashMap<EventType, List<Callback<Runnable>>> callbacks;

    public CallbackManagerRunnable(EventType[] events) {
        callbacks = new HashMap<>();
        for (EventType event : events) {
            callbacks.put(event, new ArrayList<Callback<Runnable>>());
        }
    }

    public CallbackRunnable registerCallback(EventType event, Runnable callback) {
        if (!callbacks.containsKey(event)) {
            throw new IllegalArgumentException("Event " + event + " is not registered");
        }

        CallbackRunnable callbackWrapper = new CallbackRunnable(callback);
        List<Callback<Runnable>> callbackList = callbacks.get(event);
        if (callbackList != null) {
            callbackList.add(callbackWrapper);
        }
        return callbackWrapper;
    }

    public void dispatchCallback(EventType event) {
        if (!callbacks.containsKey(event)) {
            throw new IllegalArgumentException("Event " + event + " is not registered");
        }

        List<Callback<Runnable>> callbackList = callbacks.get(event);
        if (callbackList != null) {
            for (Callback<Runnable> callback : callbackList) {
                callback.run();
            }
        }
    }
}