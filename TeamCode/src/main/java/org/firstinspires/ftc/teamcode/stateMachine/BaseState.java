package org.firstinspires.ftc.teamcode.stateMachine;

import java.util.ArrayList;
import java.util.List;

public class BaseState implements State {

    private List<Transition> transitions;
    private boolean running = false;
    private final String name;
    private Runnable entry, exit, during;
    private double minTime = 0, maxTime = Double.POSITIVE_INFINITY;
    private String nextState;
    private final BetterTimer timer = new BetterTimer();

    public BaseState(String name) {
        this.name = name;
        this.transitions = new ArrayList<>();
    }

    public void start() {
        timer.resetTimer();
        running = true;
        if (entry != null) {
            entry.run();
        }
        for (Transition t : transitions) {
            t.start();
        }
    }

    public void run() {
        if (!running) return;

        if (during != null) during.run();

        for (Transition t : transitions) {
            t.run();
            if (t.isFinished() && timer.getElapsedTime() >= minTime) {
                String transitionTarget = t.getNextState();
                if (exit != null) exit.run();
                nextState = transitionTarget;
                running = false;
                return;
            }
        }

        if (timer.getElapsedTime() >= maxTime) {
            if (exit != null) exit.run();
            running = false;
            //goes to fallback state
        }
    }

    public BaseState setEntry(Runnable function) {
        this.entry = function;
        return this;
    }
    public BaseState setExit(Runnable function) {
        this.exit = function;
        return this;
    }
    public BaseState setDuring(Runnable function) {
        this.during = function;
        return this;
    }
    public BaseState addTransition(Transition transition) {
        transitions.add(transition);
        return this;
    }
    public BaseState setMinTime(double minTime) {
        this.minTime = minTime;
        return this;
    }
    public BaseState setMaxTime(double maxTime) {
        this.maxTime = maxTime;
        return this;
    }
    public BaseState setFallbackState(String fallbackState) {
        if (fallbackState == null) {
            throw new IllegalArgumentException("Fallback State cannot be null");
        }
        this.nextState = fallbackState;
        return this;
    }

    public boolean isFinished() {
        return !running;
    }

    public String getName() {
        return name;
    }

    public String getNextState() {
        return nextState;
    }
}