package org.firstinspires.ftc.teamcode.auton.comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class RedAuto extends OpMode
{
    public Hardware hardware;
    public Gamepads gamepads;
    public Intake intake;
    public Outtake outtake;
    public AprilTagWebcam webcam;

    public Timer pathTimer;

    public Follower follower;
    public Pose startPose = new Pose(122.1927409261577, 124.35544430538174, Math.toRadians(37));
    public Pose scorePose = new Pose(95.77570093457945, 104.74766355140187, Math.toRadians(45));
    public Pose lineUp1Pose = new Pose(93.29085681426106, 68, Math.toRadians(180));
    public Pose intake1Pose = new Pose(135, 68, Math.toRadians(180));

    public Path preloadPath, lineUp1Path, intake1Path, score1Path;

    public PathState pathState = PathState.START;

    public int numShot = 0;

    public enum PathState
    {
        START,
        TO_PRELOAD_SCORE,
        AT_PRELOAD_SCORE,
        PRELOAD_LAUNCH,
        TO_LINEUP1,
        TO_INTAKE1,
        TO_SCORE1,
        AT_SCORE1,
        TO_LINEUP2,
        TO_INTAKE2,
        TO_SCORE2,
        AT_SCORE2,
    }

    @Override
    public void init()
    {
        hardware = new Hardware(hardwareMap);
        gamepads = new Gamepads(gamepad1, gamepad2);
        intake = new Intake(hardware);
        outtake = new Outtake(hardware);
        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);

        pathTimer = new Timer();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop()
    {
        if (gamepads.justPressed(Button.GP1_A)) webcam.toggleGoalId();

        boolean cameraIsReady = webcam.portal.getCameraState() == VisionPortal.CameraState.STREAMING;
        telemetry.addData("Camera Is Ready?", cameraIsReady);
        telemetry.addData("Goal Color", webcam.getGoalId() == AprilTagWebcam.RED_GOAL_ID ? "RED" : "BLUE");

        gamepads.update(gamepad1, gamepad2);
        telemetry.update();
    }

    @Override
    public void loop()
    {
        follower.update();
        autonomousPaths();
    }

    public void buildPaths()
    {
        preloadPath = new Path(new BezierLine(startPose, scorePose));
        preloadPath.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        lineUp1Path = new Path(new BezierLine(scorePose, lineUp1Pose));
        lineUp1Path.setLinearHeadingInterpolation(scorePose.getHeading(), lineUp1Pose.getHeading());

        intake1Path = new Path(new BezierLine(lineUp1Pose, intake1Pose));
        intake1Path.setConstantHeadingInterpolation(Math.toRadians(180));
        intake1Path.setVelocityConstraint(7.5);


        score1Path = new Path(new BezierLine(intake1Pose, scorePose));
        score1Path.setLinearHeadingInterpolation(intake1Pose.getHeading(), scorePose.getHeading());
    }

    public void autonomousPaths()
    {
        telemetry.addData("Path State", pathState);

        switch (pathState)
        {
            case START:
                follower.followPath(preloadPath);
                pathState = PathState.TO_PRELOAD_SCORE;
                break;
            case TO_PRELOAD_SCORE:
                if (!follower.isBusy())
                {
                    ((DcMotorEx) outtake.motor.motor).setVelocity(Outtake.MANUAL_ANGULAR_RATE);
                    pathState = PathState.AT_PRELOAD_SCORE;
                    pathTimer.resetTimer();
                }
                break;
            case AT_PRELOAD_SCORE:
                AprilTagDetection detection = webcam.getGoalDetection();
                if (detection != null)
                {
                    webcam.updateRange(detection.ftcPose.range);
                    double targetAngularRate = Outtake.toAngularRate(Outtake.calculateIdealFlywheelTangentialVelocity(webcam.getRange()));
                    ((DcMotorEx) outtake.motor.motor).setVelocity(targetAngularRate);
                    double error = ((DcMotorEx) outtake.motor.motor).getVelocity() - targetAngularRate;

                    if (Math.abs(error) < Outtake.ANGULAR_RATE_ERROR_TOLERANCE)
                    {
                        intake.forwardLaunch();
                    }
                    else
                    {
                        intake.stop();
                    }

                    telemetry.addData("Range", webcam.getRange());
                    telemetry.addData("Target Angular Rate", targetAngularRate);
                    telemetry.addData("Error", error);
                    telemetry.addData("Time ", pathTimer.getElapsedTimeSeconds());
                }
                if (pathTimer.getElapsedTimeSeconds() > 8)
                {
                    intake.forwardRegular();
                    outtake.motor.setPower(0);
                    follower.followPath(lineUp1Path);
                    pathState = PathState.TO_LINEUP1;
                }
                break;
            case TO_LINEUP1:
                if (!follower.isBusy())
                {
                    outtake.motor.setPower(0);
                    follower.followPath(intake1Path);
                    pathState = PathState.TO_INTAKE1;
                }
                break;
            case TO_INTAKE1:
                if (!follower.isBusy())
                {
                    intake.stop();
                    follower.followPath(score1Path);
                    pathState = PathState.TO_SCORE1;
                }
                break;
            case TO_SCORE1:
                if (!follower.isBusy()) pathState = PathState.AT_SCORE1;
                break;
            case AT_SCORE1:
                shoot();
                break;
            case TO_LINEUP2:
                break;
        }
    }

    public boolean shoot()
    {
        boolean successfullyShot = false;

        AprilTagDetection detection = webcam.getGoalDetection();
        if (detection != null)
        {
            webcam.updateRange(detection.ftcPose.range);
            double targetAngularRate = Outtake.toAngularRate(Outtake.calculateIdealFlywheelTangentialVelocity(webcam.getRange()));
            ((DcMotorEx) outtake.motor.motor).setVelocity(targetAngularRate);
            double error = ((DcMotorEx) outtake.motor.motor).getVelocity() - targetAngularRate;

            if (Math.abs(error) < Outtake.ANGULAR_RATE_ERROR_TOLERANCE)
            {
                intake.forwardLaunch();
            }
            else
            {
                intake.stop();
            }

            telemetry.addData("Range", webcam.getRange());
            telemetry.addData("Target Angular Rate", targetAngularRate);
            telemetry.addData("Error", error);
        }

        return successfullyShot;
    }
}
