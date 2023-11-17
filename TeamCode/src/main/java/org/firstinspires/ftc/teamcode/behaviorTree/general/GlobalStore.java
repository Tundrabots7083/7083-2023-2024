package org.firstinspires.ftc.teamcode.behaviorTree.general;

import java.util.HashMap;
import java.util.Map;

public class GlobalStore {
    private final Map<String, Object> data = new HashMap<String, Object>();

    public void setValue(String key, Object value) {
        data.put(key, value);
    }
    public Object removeValue(String key) {
        return data.remove(key);
    }

    public Object getValue(String key) {
        return data.get(key);
    }
}