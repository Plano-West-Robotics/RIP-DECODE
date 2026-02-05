package org.firstinspires.ftc.teamcode.statemachine;

public interface State {
    void start();
    void run();
    boolean isFinished();
    String getName();
    String getNextState();
}
