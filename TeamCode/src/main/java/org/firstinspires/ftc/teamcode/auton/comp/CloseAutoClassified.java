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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class CloseAutoClassified extends BaseAuto
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
    }

    @Override
    public void setup_loop()
    {
//        if (gamepads.justPressed(Button.GP1_A))
//        {
//            webcam.toggleGoalId();
//        }
        boolean cameraIsReady = webcam.portal.getCameraState() == VisionPortal.CameraState.STREAMING;
        telemetry.addData("Camera Is Ready?", cameraIsReady);
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

        headings[pathCount] = new double[] {AutonConstants.RED_START.getHeading()};
        poses[pathCount++] = new Pose[]
        {
            AutonConstants.RED_START, AutonConstants.RED_SCORE
        };

        headings[pathCount] = new double[] {AutonConstants.RED_SCORE.getHeading(), AutonConstants.RED_LINEUP_2.getHeading()};
        poses[pathCount++] = new Pose[]
        {
            AutonConstants.RED_SCORE, AutonConstants.RED_LINEUP_2
        };

        headings[pathCount] = new double[] {AutonConstants.RED_LINEUP_2.getHeading()};
        poses[pathCount++] = new Pose[]
        {
            AutonConstants.RED_LINEUP_2, AutonConstants.RED_INTAKE_2
        };

        headings[pathCount] = new double[] {AutonConstants.RED_INTAKE_2.getHeading()};
        poses[pathCount++] = new Pose[]
        {
            AutonConstants.RED_INTAKE_2, AutonConstants.RED_INTERMEDIATE_2
        };

        headings[pathCount] = new double[] {AutonConstants.RED_INTERMEDIATE_2.getHeading()};
        poses[pathCount++] = new Pose[]
        {
                AutonConstants.RED_INTERMEDIATE_2, AutonConstants.RED_CLEAR_LINEUP
        };

        headings[pathCount] = new double[] {AutonConstants.RED_CLEAR_LINEUP.getHeading()};
        poses[pathCount++] = new Pose[]
        {
                AutonConstants.RED_CLEAR_LINEUP, AutonConstants.RED_CLEAR_GOAL
        };

        headings[pathCount] = new double[] {AutonConstants.RED_CLEAR_LINEUP.getHeading()};
        poses[pathCount++] = new Pose[]
        {
            AutonConstants.RED_CLEAR_GOAL, AutonConstants.RED_CLEAR_LINEUP
        };

        headings[pathCount] = new double[] {AutonConstants.RED_CLEAR_GOAL.getHeading(), AutonConstants.RED_SCORE.getHeading()};
        poses[pathCount++] = new Pose[]
        {
                AutonConstants.RED_CLEAR_GOAL, AutonConstants.RED_SCORE
        };

        headings[pathCount] = new double[] {AutonConstants.RED_SCORE.getHeading(), AutonConstants.RED_LINEUP_1.getHeading()};
        poses[pathCount++] = new Pose[]
                {
                        AutonConstants.RED_SCORE, AutonConstants.RED_LINEUP_1
                };

        headings[pathCount] = new double[] {AutonConstants.RED_LINEUP_1.getHeading()};
        poses[pathCount++] = new Pose[]
        {
            AutonConstants.RED_LINEUP_1, AutonConstants.RED_INTAKE_1
        };

        headings[pathCount] = new double[] {AutonConstants.RED_INTAKE_1.getHeading(), AutonConstants.RED_SCORE.getHeading()};
        poses[pathCount++] = new Pose[]
        {
            AutonConstants.RED_INTAKE_1, AutonConstants.RED_SCORE
        };

        headings[pathCount] = new double[] {AutonConstants.RED_SCORE.getHeading(), AutonConstants.RED_LINEUP_3.getHeading()};
        poses[pathCount++] = new Pose[]
        {
          AutonConstants.RED_SCORE, AutonConstants.RED_LINEUP_3
        };

        headings[pathCount] = new double[] {AutonConstants.RED_LINEUP_3.getHeading()};
        poses[pathCount++] = new Pose[]
        {
                AutonConstants.RED_LINEUP_3, AutonConstants.RED_INTAKE_3
        };

        headings[pathCount] = new double[] {AutonConstants.RED_INTAKE_3.getHeading(), AutonConstants.RED_SCORE.getHeading()};
        poses[pathCount++] = new Pose[]
        {
            AutonConstants.RED_INTAKE_3, AutonConstants.RED_SCORE
        };

        headings[pathCount] = new double[] {AutonConstants.RED_SCORE.getHeading(), AutonConstants.RED_LEAVE_1.getHeading()};
        poses[pathCount++] = new Pose[]
        {
            AutonConstants.RED_SCORE, AutonConstants.RED_LEAVE_1
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
        paths[8].setVelocityConstraint(AutonConstants.INTAKE_1_VEL_CONSTRAINT);
        paths[11].setVelocityConstraint(AutonConstants.INTAKE_1_VEL_CONSTRAINT);
    }

    public State[] buildMachine()
    {
        State[] states = new State[20];

        states[0] = new BaseState("START")
                .setEntry(() -> {
                    follower.followPath(paths[0]);
                })
                .setDuring(() -> {
                    outtake.setVelocity(Outtake.MANUAL_ANGULAR_RATE);
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "AT_PRELOAD_SCORE"));

        states[1] = new BaseState("AT_PRELOAD_SCORE")
                .setEntry(() -> {
                    pathTimer.resetTimer();
                    outtake.hoodDown();
                })
                .setDuring(this::shootArtifacts)
                .setExit(() -> {
                    intake.forwardRegular();
                    outtake.setVelocity(-800);
                    follower.followPath(paths[1]);
                })
                .addTransition(new Transition(() -> pathTimer.getElapsedTimeSeconds() > AutonConstants.PRELOAD_SCORE_TIME, "TO_LINEUP2"));

        states[2] = new BaseState("TO_LINEUP2")
                .setExit(() -> {
                    follower.followPath(paths[2]);
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "TO_INTAKE2"));

        states[3] = new BaseState("TO_INTAKE2")
                .setExit(() -> {
                    follower.followPath(paths[3]);
                    pathTimer.resetTimer();
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "TO_INTERMEDIATE2"));
        states[4] = new BaseState("TO_INTERMEDIATE2")
                .setExit(() -> {
                    follower.followPath(paths[4]);
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "TO_CLEAR_LINEUP"));

        states[5] = new BaseState("TO_CLEAR_LINEUP")
                .setExit(() -> {
                    follower.followPath(paths[5]);
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "CLEAR_LINEUP"));

        states[6] = new BaseState("CLEAR_LINEUP")
                .setExit(() -> {
                    follower.followPath(paths[6]);
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "AVOID"));

        states[7] = new BaseState("AVOID")
                .setExit(() -> {
                    follower.followPath(paths[7]);
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "TO_SCORE2"));

        states[8] = new BaseState("TO_SCORE2")
                .setDuring(() -> {
                    intake.stop();
                    outtake.setVelocity(Outtake.MANUAL_ANGULAR_RATE);
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "AT_SCORE2"));

        states[9] = new BaseState("AT_SCORE2")
                .setEntry(() -> {
                    pathTimer.resetTimer();
                })
                .setDuring(this::shootArtifacts)
                .setExit(() -> {
                    intake.forwardRegular();
                    outtake.setVelocity(-800);
                    follower.followPath(paths[8], true);
                })
                .addTransition(new Transition(() -> pathTimer.getElapsedTimeSeconds() > AutonConstants.FIRST_THREE_SCORE_TIME, "TO_LINEUP1"));

        states[10] = new BaseState("TO_LINEUP1")
                .setExit(() -> {
                    follower.followPath(paths[9], true);
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "TO_INTAKE1"));


        states[11] = new BaseState("TO_INTAKE1")
                .setExit(() -> {
                    follower.followPath(paths[10], true);
                    pathTimer.resetTimer();
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "TO_SCORE1"));

        states[12] = new BaseState("TO_SCORE1")
                .setDuring(() -> {
                    intake.stop();
                    outtake.setVelocity(Outtake.MANUAL_ANGULAR_RATE);
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "AT_SCORE1"));

        states[13] = new BaseState("AT_SCORE1")
                .setEntry(() -> {
                    pathTimer.resetTimer();
                })
                .setDuring(this::shootArtifacts)
                .setExit(() -> {
                    intake.forwardRegular();
                    outtake.setVelocity(-800);
                    follower.followPath(paths[11]);
                })
                .addTransition(new Transition(() -> pathTimer.getElapsedTimeSeconds() > AutonConstants.FIRST_THREE_SCORE_TIME, "TO_LINEUP3"));

        states[14] = new BaseState("TO_LINEUP3")
                .setExit(() -> {
                    follower.followPath(paths[12]);
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "TO_INTAKE3"));

        states[15] = new BaseState("TO_INTAKE3")
                .setExit(() -> {
                    follower.followPath(paths[13]);
                    pathTimer.resetTimer();
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "TO_SCORE3"));

        states[16] = new BaseState("TO_SCORE_3")
                .setDuring(() -> {
                    intake.stop();
                    outtake.setVelocity(Outtake.MANUAL_ANGULAR_RATE);
                })
                .addTransition(new Transition(() -> !follower.isBusy(), "AT_SCORE3"));

        states[17] = new BaseState("AT_SCORE3")
                .setEntry(() -> {
                    pathTimer.resetTimer();
                })
                .setDuring(this::shootArtifacts)
                .setExit(() -> {
                    intake.forwardRegular();
                    outtake.setVelocity(-800);
                    follower.followPath(paths[14]);
                })
                .addTransition(new Transition(() -> pathTimer.getElapsedTimeSeconds() > AutonConstants.FIRST_THREE_SCORE_TIME, "LEAVE_LINE"));

        states[18] = new BaseState("LEAVE_LINE")
                .addTransition(new Transition(() -> !follower.isBusy(), "STOP"));

        states[19] = new BaseState("STOP");

        return states;
    }

    public void shootArtifacts(double toleranceTPS)
    {
        AprilTagDetection detection = webcam.getGoalDetection();
        if (detection != null)
        {
            webcam.updateRange(detection.ftcPose.range);
            double range = webcam.getRange();

            double targetAngularRate = Outtake.toAngularRate(
                    Outtake.calculateIdealFlywheelTangentialVelocity(range));

            outtake.setVelocity(targetAngularRate);

            double error = outtake.getAverageVelocity() - targetAngularRate;

            if (Math.abs(error) < toleranceTPS)
                intake.forwardLaunch();
            else
                intake.stop();

            telemetry.addData("Range", range);
            telemetry.addData("Target Angular Rate", targetAngularRate);
            telemetry.addData("Error", error);
        }
    }

    public void shootArtifacts()
    {
        AprilTagDetection detection = webcam.getGoalDetection();
        if (detection != null)
        {
            webcam.updateRange(detection.ftcPose.range);
            double range = webcam.getRange();

            double targetAngularRate = Outtake.toAngularRate(
                    Outtake.calculateIdealFlywheelTangentialVelocity(range));

            outtake.setVelocity(Outtake.MANUAL_ANGULAR_RATE);

            double error = outtake.getAverageVelocity() - Outtake.MANUAL_ANGULAR_RATE;

            if (Math.abs(error) < Outtake.NORMAL_ERROR_TOLERANCE)
                intake.forwardLaunch();
            //else
                //.intake.stop();

            telemetry.addData("Range", range);
            telemetry.addData("Target Angular Rate", targetAngularRate);
            telemetry.addData("Error", error);
        }
    }
}
