package org.firstinspires.ftc.teamcode.stateMachine;

public class BetterTimer {

    private long startTime;

    public BetterTimer() {
        resetTimer();
    }

    public void resetTimer() {
        startTime = System.currentTimeMillis();
    }

    public long getElapsedTime() {
        return System.currentTimeMillis() - startTime;
    }

    public double getElapsedTimeSeconds() {
        return (getElapsedTime() / 1000.0);
    }
}