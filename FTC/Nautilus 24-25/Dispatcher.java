package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.LinkedList;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Queue;
import java.util.HashMap;
import java.util.function.Supplier;
import java.util.function.Consumer;
import java.util.ArrayList;
import java.util.Map;

public class Dispatcher {

    public static class Sequence {
        ArrayList<Supplier<Boolean>> commandList;
        String topic;
        Dispatcher dx;

        public Sequence(String topic, Dispatcher dx) {
            this.topic = topic;
            this.dx = dx;
            this.commandList = new ArrayList<Supplier<Boolean>>();
            dx.subscribe(topic, (step) -> doSTEP((int) step));
        }
        
        public void doSTEP(int step) {
            if (step < 0) return;
            if (step >= commandList.size()) {
                dx.set(topic, -1);
                return;
            }
            Supplier<Boolean> cond = commandList.get(step);
            dx.stepWhen(cond, topic);
        }

        public Sequence set(String t, Object value) {
            commandList.add(() -> {dx.set(t, value); return true;});
            return this;
        }
        public Sequence setWhen(Supplier<Boolean> cond, String t, Object value) {
            commandList.add(() -> {dx.setWhen(cond, t, value); return true;});
            return this;
        }
        public Sequence waitFor(double delay) {
            commandList.add(() -> dx.tvMap.get(topic).age() > delay);
            return this;
        }
        public Sequence waitFor(Supplier<Boolean> cond) {
            commandList.add(cond);
            return this;
        }
        public Sequence waitFor(Supplier<Boolean> cond, double timeout) {
            commandList.add(() -> cond.get() || dx.tvMap.get(topic).age() > timeout);
            return this;
        }
        public Sequence waitFor(String t, Object value) {
            commandList.add(() -> value.equals(dx.get(t)));
            return this;
        }
        public Sequence run(Runnable command) {
            commandList.add(() -> {command.run(); return true;});
            return this;
        }
        
        public Sequence start() {
            dx.set(topic, 0);
            return this;
        }
    }

    private class SubscriberList extends ArrayList<Consumer<Object>> { }
    private class TopicValue {
        Supplier<Boolean> cond;
        String topic;
        Object value;
        double timeStamp;

        public TopicValue(Supplier<Boolean> c, String t, Object v) {
            cond = c; topic = t; value = v; timeStamp = 0;
        }

        double age() {
            return clock.seconds() - timeStamp;
        }
    }

    public ElapsedTime clock = new ElapsedTime();
    public TopicValue dxrun = new TopicValue(null, "/dx/run", 0);
    public TopicValue dxrunms = new TopicValue(null, "/dx/runms", 0.0);
    public int runCount = 0;
    public int prevRunCount = 0;
    HashMap<String, SubscriberList> subscribersMap = new HashMap<String, SubscriberList>();
    HashMap<String, TopicValue> tvMap = new HashMap<String, TopicValue>();
    HashMap<String, Sequence> seqMap = new HashMap<String, Sequence>();
    Queue<TopicValue> currQueue = new LinkedList<TopicValue>();
    Queue<TopicValue> nextQueue = null;
    public int postcount = 0;
    public int repostcount = 0;
    
    private void clearQueues(String topic) {
        if (nextQueue != null) nextQueue.removeIf(tv -> tv.topic.equals(topic));
        if (currQueue != null) currQueue.removeIf(tv -> tv.topic.equals(topic));
    }
    
    public Sequence newSequence(String topic) {
        if (seqMap.get(topic) == null) {
            subscribe(topic, (step) -> seqMap.get(topic).doSTEP((int)step));
        }
        cancelSequence(topic);
        clearQueues(topic);
        Sequence s = new Sequence(topic, this);
        seqMap.put(topic, s);
        return s;
    }
    
    public void cancelSequence(String topic) {
        clearQueues(topic);
        Object v = get(topic);
        if (v != null && (Integer)v >= 0) { post(new TopicValue(null, topic, -2)); }
    }

    public void subscribe(String topic, Consumer<Object> callback) {
        SubscriberList subList = subscribersMap.get(topic);
        if (subList == null) {
            subList = new SubscriberList();
            subscribersMap.put(topic, subList);
        }
        subList.add(callback);
    }

    private void post(TopicValue tv) {
        if (tv.topic.equals("/robot/task")) postcount++;
        TopicValue oldvalue = tvMap.get(tv.topic);
        if (oldvalue != null && tv.value != null && tv.value.equals(oldvalue.value)) return;
        repost(tv);
    }

    private void repost(TopicValue tv) {
        if (tv.topic.equals("/robot/task")) repostcount++;
        tv.timeStamp = clock.seconds();
        tvMap.put(tv.topic, tv);
        String topic = tv.topic;
        while (!topic.isEmpty()) {
            SubscriberList subList = subscribersMap.get(topic);
            if (subList != null) {
                for (Consumer<Object> callback : subList) {
                    callback.accept(tv.value);
                }
            }
            int lastslash = topic.lastIndexOf("/");
            if (lastslash <= 0) break;
            topic = topic.substring(0, lastslash);
        }
    }

    public boolean isEqual(String topic, Object v) {
        TopicValue tv = tvMap.get(topic);
        return tv != null && tv.value != null && v.equals(tv.value);
    }

    public <T> T get(String topic, java.lang.Class<? extends T> classOrInterface) {
        TopicValue tv = tvMap.get(topic);
        if (tv != null) return classOrInterface.cast(tv.value);
        return null;
    }
    public Object get(String topic) {
        TopicValue tv = tvMap.get(topic);
        if (tv != null) return tv.value;
        return null;
    }
    public double age(String topic) {
        TopicValue tv = tvMap.get(topic);
        if (tv != null) return tv.age();
        return -1;
    }

    public void setWhen(Supplier<Boolean> cond, String topic, Object value) {
        if (currQueue == null) currQueue = new LinkedList<TopicValue>();
        clearQueues(topic);
        currQueue.add(new TopicValue(cond, topic, value));
    }
    public void setWhen(double delay, String topic, Object value) {
        double alarm = clock.seconds() + delay;
        setWhen(() -> clock.seconds() >= alarm, topic, value);
    }
    public void set(String topic, Object value) {
        setWhen(() -> true, topic, value);
    }
    public void step(String topic) {
        set(topic, (int)get(topic)+1);
    }
    public void stepWhen(Supplier<Boolean> cond, String topic) {
        setWhen(cond, topic, (int)get(topic)+1);
    }
    public void stepWhen(double delay, String topic) {
        setWhen(delay, topic, (int)get(topic)+1);
    }

    public void run() {
        dxrun.value = runCount;
        runCount++;
        repost(dxrun);
        double t = dxrunms.age();
        if (t > 0.25) {
            int c = runCount - prevRunCount;
            if (c == 0) c = 1;
            dxrunms.value = t / c * 1000;
            repost(dxrunms);
            prevRunCount = runCount;
        }
        if (nextQueue != null) currQueue.addAll(nextQueue);
        nextQueue = new LinkedList<TopicValue>();
        while (!currQueue.isEmpty()) {
            TopicValue tv = currQueue.poll();
            if (tv.cond.get()) post(tv);
            else nextQueue.add(tv);
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        addTelemetry(telemetry, "");
    }
    public void addTelemetry(Telemetry telemetry, String prefix) {
        for (Map.Entry<String, TopicValue> e : tvMap.entrySet()) {
            String key = e.getKey();
            if (key.startsWith(prefix)) {
                String s = String.format(e.getValue().value + " %.2f", age(key));
                telemetry.addData(key, s);
            }
        }
    }
}