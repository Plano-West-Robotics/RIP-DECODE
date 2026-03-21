package org.firstinspires.ftc.teamcode.auton.comp;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.stateMachine.BaseState;
import org.firstinspires.ftc.teamcode.stateMachine.State;
import org.firstinspires.ftc.teamcode.stateMachine.StateMachine;
import org.firstinspires.ftc.teamcode.stateMachine.Transition;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class BlueCloseNoClearStateMachineAuto extends BaseAuto
{
    public Pose[][] poses = new Pose[AutonConstants.PATH_COUNT_UPPER_BOUND][2];
    public double[][] headings = new double[AutonConstants.PATH_COUNT_UPPER_BOUND][];
    public Path[] paths = new Path[AutonConstants.PATH_COUNT_UPPER_BOUND];
    public int pathCount;

    public StateMachine fsm;

    @Override
    public void setup()
    {
        createPaths();
        fsm = new StateMachine(buildMachine());
        follower.setStartingPose(poses[0][0]);
        Drawing.init();
        webcam.toggleGoalId();
    }

    @Override
    public void setup_loop()
    {
        boolean cameraIsReady = webcam.portal.getCameraState() == VisionPortal.CameraState.STREAMING;
        telemetry.addData("Camera Is Ready?", cameraIsReady);

        // pressing GP1_Y toggles the goal color
        telemetry.addData("Goal Color", webcam.getGoalId() == AprilTagWebcam.RED_GOAL_ID ? "RED" : "BLUE");
    }

    @Override
    public void go()
    {
        fsm.start();
    }

    @Override
    public void while_running()
    {
//        Drawing.drawDebug(follower); // TODO: Check if the drawing shows up in Panels, then remove it when no longer necessary because it slows loop times
        fsm.run();

        telemetry.addData("Current State", fsm.currentState());
    }


    /*
        Additional Methods Below
    */


    public void createPaths()
    {
        pathCount = 0;

        headings[pathCount] = new double[]{AutonConstants.BLUE_START.getHeading()};
        poses[pathCount++] = new Pose[]
            {
                AutonConstants.BLUE_START, AutonConstants.BLUE_SCORE
            };

        headings[pathCount] = new double[]{AutonConstants.BLUE_SCORE.getHeading(), AutonConstants.BLUE_LINEUP_1.getHeading()};
        poses[pathCount++] = new Pose[]
            {
                AutonConstants.BLUE_SCORE, AutonConstants.BLUE_LINEUP_1
            };

        headings[pathCount] = new double[]{AutonConstants.BLUE_LINEUP_1.getHeading()};
        poses[pathCount++] = new Pose[]
            {
                AutonConstants.BLUE_LINEUP_1, AutonConstants.BLUE_INTAKE_1
            };

        headings[pathCount] = new double[]{AutonConstants.BLUE_INTAKE_1.getHeading(), AutonConstants.BLUE_SCORE.getHeading()};
        poses[pathCount++] = new Pose[]
            {
                AutonConstants.BLUE_INTAKE_1, AutonConstants.BLUE_SCORE
            };

        headings[pathCount] = new double[]{AutonConstants.BLUE_SCORE.getHeading(), AutonConstants.BLUE_LINEUP_2.getHeading()};
        poses[pathCount++] = new Pose[]
            {
                AutonConstants.BLUE_SCORE, AutonConstants.BLUE_LINEUP_2
            };

        headings[pathCount] = new double[]{AutonConstants.BLUE_LINEUP_2.getHeading()};
        poses[pathCount++] = new Pose[]
            {
                AutonConstants.BLUE_LINEUP_2, AutonConstants.BLUE_INTAKE_2
            };

        headings[pathCount] = new double[]{AutonConstants.BLUE_INTAKE_2.getHeading(), AutonConstants.BLUE_INTERMEDIATE_2.getHeading()};
        poses[pathCount++] = new Pose[]
            {
                AutonConstants.BLUE_INTAKE_2, AutonConstants.BLUE_INTERMEDIATE_2
            };

        headings[pathCount] = new double[]{AutonConstants.BLUE_INTERMEDIATE_2.getHeading(), AutonConstants.BLUE_SCORE.getHeading()};
        poses[pathCount++] = new Pose[]
            {
                AutonConstants.BLUE_INTERMEDIATE_2, AutonConstants.BLUE_SCORE
            };

        headings[pathCount] = new double[]{AutonConstants.BLUE_SCORE.getHeading(), AutonConstants.BLUE_LINEUP_3.getHeading()};
        poses[pathCount++] = new Pose[]
            {
                AutonConstants.BLUE_SCORE, AutonConstants.BLUE_LINEUP_3
            };

        headings[pathCount] = new double[]{AutonConstants.BLUE_LINEUP_3.getHeading()};
        poses[pathCount++] = new Pose[]
            {
                AutonConstants.BLUE_LINEUP_3, AutonConstants.BLUE_INTAKE_3
            };

        headings[pathCount] = new double[]{AutonConstants.BLUE_INTAKE_3.getHeading(), AutonConstants.BLUE_SCORE.getHeading()};
        poses[pathCount++] = new Pose[]
            {
                AutonConstants.BLUE_INTAKE_3, AutonConstants.BLUE_SCORE_END
            };

        for (int i = 0; i < pathCount; i++)
        {
            paths[i] = new Path(new BezierLine(poses[i][0], poses[i][1]));
            if (headings[i].length == 1)
                paths[i].setConstantHeadingInterpolation(headings[i][0]);
            else if (headings[i].length == 2)
                paths[i].setLinearHeadingInterpolation(headings[i][0], headings[i][1]);
        }

        /*
            PATH MODIFICATIONS
        */
        paths[2].setVelocityConstraint(AutonConstants.INTAKE_1_VEL_CONSTRAINT);
        paths[5].setVelocityConstraint(AutonConstants.INTAKE_1_VEL_CONSTRAINT);
        paths[9].setVelocityConstraint(AutonConstants.INTAKE_1_VEL_CONSTRAINT);
    }

    public State[] buildMachine()
    {
        State[] states = new State[17];

        states[0] = new BaseState("START")
            .setEntry(() ->
            {
                follower.followPath(paths[0]);
            })
            .setDuring(() ->
            {
                outtake.setVelocity(Outtake.AUTO_MANUAL_ANGULAR_RATE);
            })
            .addTransition(new Transition(() -> !follower.isBusy(), "AT_PRELOAD_SCORE"));

        states[1] = new BaseState("AT_PRELOAD_SCORE")
            .setEntry(() ->
            {
                pathTimer.resetTimer();
                outtake.hoodDown();
            })
            .setDuring(this::shootArtifacts)
            .setExit(() ->
            {
                intake.forwardRegular();
                intake.reverseRegularTransfer();
                follower.followPath(paths[1]);
            })
            .addTransition(new Transition(() -> pathTimer.getElapsedTimeSeconds() > AutonConstants.PRELOAD_SCORE_TIME, "TO_LINEUP1"));

        states[2] = new BaseState("TO_LINEUP1")
            .setExit(() ->
            {
                follower.followPath(paths[2]);
            })
            .addTransition(new Transition(() -> !follower.isBusy(), "TO_INTAKE1"));

        states[3] = new BaseState("TO_INTAKE1")
            .setExit(() ->
            {
                follower.followPath(paths[3]);
                pathTimer.resetTimer();
            })
            .addTransition(new Transition(() -> !follower.isBusy(), "TO_SCORE1"));

        states[4] = new BaseState("TO_SCORE1")
            .setDuring(() ->
            {
                intake.stop();
                intake.stopTransfer();
                outtake.setVelocity(Outtake.AUTO_MANUAL_ANGULAR_RATE);
            })
            .addTransition(new Transition(() -> !follower.isBusy(), "AT_SCORE1"));

        states[5] = new BaseState("AT_SCORE1")
            .setEntry(() ->
            {
                pathTimer.resetTimer();
            })
            .setDuring(this::shootArtifacts)
            .setExit(() ->
            {
                intake.forwardRegular();
                intake.reverseRegularTransfer();
                follower.followPath(paths[4]);
            })
            .addTransition(new Transition(() -> pathTimer.getElapsedTimeSeconds() > AutonConstants.FIRST_THREE_SCORE_TIME, "TO_LINEUP2"));

        states[6] = new BaseState("TO_LINEUP2")
            .setExit(() ->
            {
                follower.followPath(paths[5]);
            })
            .addTransition(new Transition(() -> !follower.isBusy(), "TO_INTAKE2"));

        states[7] = new BaseState("TO_INTAKE2")
            .setExit(() ->
            {
                follower.followPath(paths[6]);
                pathTimer.resetTimer();
            })
            .addTransition(new Transition(() -> !follower.isBusy(), "TO_INTERMEDIATE2"));

        states[8] = new BaseState("TO_INTERMEDIATE2")
            .setDuring(() -> {
                outtake.setVelocity(Outtake.AUTO_MANUAL_ANGULAR_RATE);
            })
            .setExit(() ->
            {
                follower.followPath(paths[7]);
                pathTimer.resetTimer();
            })
            .addTransition(new Transition(() -> !follower.isBusy(), "TO_SCORE2"));

        states[9] = new BaseState("TO_SCORE2")
            .setDuring(() -> {
                intake.stop();
                intake.stopTransfer();
            })
            .addTransition(new Transition(() -> !follower.isBusy(), "AT_SCORE2"));

        states[10] = new BaseState("AT_SCORE2")
            .setEntry(() ->
            {
                pathTimer.resetTimer();
            })
            .setDuring(this::shootArtifacts)
            .setExit(() ->
            {
                intake.forwardRegular();
                intake.reverseRegularTransfer();
                follower.followPath(paths[8], true);
            })
            .addTransition(new Transition(() -> pathTimer.getElapsedTimeSeconds() > AutonConstants.FIRST_THREE_SCORE_TIME, "TO_LINEUP3"));

        states[11] = new BaseState("TO_LINEUP3")
            .setExit(() ->
                {
                    follower.followPath(paths[9], true);
                }
            )
            .addTransition(new Transition(() -> !follower.isBusy(), "TO_INTAKE3"));

        states[12] = new BaseState("TO_INTAKE3")
            .setExit(() ->
            {
                follower.followPath(paths[10], true);
                pathTimer.resetTimer();
            })
            .addTransition(new Transition(() -> !follower.isBusy(), "TO_SCORE3"));

        states[13] = new BaseState("TO_SCORE3")
            .setDuring(() ->
            {
                intake.stop();
                intake.stopTransfer();
                outtake.setVelocity(Outtake.AUTO_MANUAL_ANGULAR_RATE);
            })
            .addTransition(new Transition(() -> !follower.isBusy(), "AT_SCORE3"));

        states[14] = new BaseState("AT_SCORE3")
            .setEntry(() ->
            {
                pathTimer.resetTimer();
            })
            .setDuring(this::shootArtifacts)
            .setExit(() ->
            {
                intake.stop();
                intake.stopTransfer();
                outtake.setVelocity(0);
            })
            .addTransition(new Transition(() -> pathTimer.getElapsedTimeSeconds() > AutonConstants.FIRST_THREE_SCORE_TIME, "STOP"));

        states[15] = new BaseState("STOP");

        return states;
    }

    public void shootArtifacts()
    {
        outtake.setVelocity(Outtake.AUTO_MANUAL_ANGULAR_RATE);
        double error = outtake.getAverageVelocity() - Outtake.AUTO_MANUAL_ANGULAR_RATE;
        if (Math.abs(error) < Outtake.NORMAL_ERROR_TOLERANCE)
        {
            intake.forwardLaunch();
            intake.forwardLaunchTransfer();
        }
        telemetry.addData("Error", error);
        telemetry.update();
    }
}
