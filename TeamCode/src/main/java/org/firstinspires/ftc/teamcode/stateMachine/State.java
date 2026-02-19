package org.firstinspires.ftc.teamcode.stateMachine;

public interface State {
    void start();
    void run();
    boolean isFinished();
    String getName();
    String getNextState();
}
