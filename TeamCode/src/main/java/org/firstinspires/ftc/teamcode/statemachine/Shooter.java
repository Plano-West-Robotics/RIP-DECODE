package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.statemachine.State;
//import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.ArrayList;

public class Shooter {
    private StateMachine state;

//    private Flywheel flywheel;
//    private Hood hood;
//    private Turret turret;
//    private Pose2d pose, goal;
//    private InterpLUT shooterLookup;

    public final static double IDLE_VEL = 4000, IDLE_HOOD = 0.5;

    private boolean requestShot;

    public Shooter (HardwareMap map) {
//        flywheel = new Flywheel(map);
//        hood = new Hood(map);
//        turret = new Turret(map);
//
//        requestShot = false;
//        pose = new Pose2d(0, 0, Math.toRadians(0));
//        goal = new Pose2d(0, 0, Math.toRadians(0));
//
//        shooterLookup = new InterpLUT();

        State[] states = createStates();
        state = new StateMachine(states);
    }

    private State[] createStates() {
        State[] states = new State[2];

        states[0] = new BaseState("IDLE")
                .setDuring(() -> {
//                    flywheel.setVelocity(IDLE_VEL);
//                    hood.setPosition(IDLE_HOOD);
                })
                .addTransition(new Transition(() -> requestShot, "ON"));

        states[1] = new BaseState("ON")
                .setEntry(() -> {
//                    turret.requestTracking();
                })
                .setDuring(() -> {
                    //profiling

                    // shooter will take from interplut
                    // hood will follow regression

//                    double dx = goal.getErrorInX(pose);
//                    double dy = goal.getErrorInY(pose);
//
//                    double tAngle = Math.toDegrees(Math.atan2(dy, dx));
//                    double offset = tAngle - Math.toDegrees(pose.getHeading());
//
//                    turret.setTargetAngle(offset);
                })
                .setExit(() -> {
//                    turret.stopTracking();
                })
                .setFallbackState("IDLE")
                .addTransition(new Transition(() -> !requestShot, "IDLE"));

        return states;
    }

    public void init() {
        state.start();
//        flywheel.setVelocity(IDLE_VEL);
//        hood.setPosition(IDLE_HOOD);
//        turret.setCurAngle(turret.getCurPosition());
//        goal = Robot.goal;
//        pose = Robot.pose;
    }

    public void update() {
//        flywheel.update();
//        hood.update();
//        turret.update();

        state.run();
    }

//    public void updatePose(Pose2d pose) {
//        this.pose = pose;
//    }
//    public void updateGoal() {
//        this.goal = Robot.goal;
//    }

//    public boolean isReady() {
//        if (!state.currentState().equals("ON")) return false;
//
////        return flywheel.atVelocity() && hood.atPosition() && turret.inPosition();
//    }

    public void requestShot() { requestShot = true; }

    public void requestIdle() {
        requestShot = false;
    }
}