//package org.firstinspires.ftc.teamcode.auton.comp;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.core.control.Button;
//import org.firstinspires.ftc.teamcode.core.control.Gamepads;
//import org.firstinspires.ftc.teamcode.hardware.Hardware;
//import org.firstinspires.ftc.teamcode.hardware.RightStopper;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//@Autonomous(preselectTeleOp = "MainComp", group = "Comp")
//public class FarAuto extends OpMode
//{
//    public Pose startPose = AutonConstants.FAR_RED_START;
//    public Pose scorePose = AutonConstants.FAR_RED_SCORE;
//    public Pose lineUp1Pose = AutonConstants.FAR_RED_LINEUP;
//    public Pose intake1Pose = AutonConstants.FAR_RED_INTAKE;
//    public Pose backToScorePose = AutonConstants.FAR_RED_SCORE_1;
//
//
//    public Hardware hardware;
//    public RightStopper rightStopper;
//    public Gamepads gamepads;
//    public Intake intake;
//    public Outtake outtake;
//    public AprilTagWebcam webcam;
//
//    public Timer pathTimer;
//
//    public Follower follower;
//
//    public Path preloadPath, lineUp1Path, intake1Path, score1Path;
//
//    public PathState pathState;
//
//    public int numShot = 0;
//
//    public boolean isRed = true;
//    public boolean score1ReverseLaunchDone = false;
//    public boolean disableAuto = false;
//
//    public enum PathState
//    {
//        START,
//        TO_PRELOAD_SCORE,
//        AT_PRELOAD_SCORE,
//        TO_LINEUP,
//        TO_INTAKE1,
//        TO_SCORE1,
//        AT_SCORE1,
//        STOP
//    }
//
//    @Override
//    public void init()
//    {
//        hardware = new Hardware(hardwareMap);
//        rightStopper = hardware.rightStopper;
//        gamepads = new Gamepads(gamepad1, gamepad2);
//        intake = new Intake(hardware);
//        outtake = new Outtake(hardware);
//        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);
//
//        pathTimer = new Timer();
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        follower = Constants.createFollower(hardwareMap);
//
//        pathState = PathState.START;
//    }
//
//    @Override
//    public void init_loop()
//    {
//        if (gamepads.justPressed(Button.GP1_A))
//        {
//            webcam.toggleGoalId();
//
//            if (isRed)
//            {
//                startPose = AutonConstants.FAR_BLUE_START;
//                scorePose = AutonConstants.FAR_BLUE_SCORE;
//                lineUp1Pose = AutonConstants.FAR_BLUE_LINEUP;
//                intake1Pose = AutonConstants.FAR_BLUE_INTAKE;
//                backToScorePose = AutonConstants.FAR_BLUE_SCORE_1;
//            }
//            else
//            {
//                startPose = AutonConstants.FAR_RED_START;
//                scorePose = AutonConstants.FAR_RED_SCORE;
//                lineUp1Pose = AutonConstants.FAR_RED_LINEUP;
//                intake1Pose = AutonConstants.FAR_RED_INTAKE;
//                backToScorePose = AutonConstants.FAR_RED_SCORE_1;
//            }
//
//            isRed = !isRed;
//        }
//
//        if (gamepads.justPressed(Button.GP1_B))
//        {
//            disableAuto = !disableAuto;
//        }
//
//        boolean cameraIsReady = webcam.portal.getCameraState() == VisionPortal.CameraState.STREAMING;
//        telemetry.addData("Camera Is Ready?", cameraIsReady);
//        telemetry.addData("Goal Color", webcam.getGoalId() == AprilTagWebcam.RED_GOAL_ID ? "RED" : "BLUE");
//        telemetry.addData("Auto Disabled", disableAuto);
//
//        gamepads.update(gamepad1, gamepad2);
//        telemetry.update();
//    }
//
//    @Override
//    public void start()
//    {
//        follower.setStartingPose(startPose);
//        buildPaths();
//    }
//
//    @Override
//    public void loop()
//    {
//        follower.update();
//        autonomousPaths();
//    }
//
//    public void buildPaths()
//    {
//        preloadPath = new Path(new BezierLine(startPose, scorePose));
//        preloadPath.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//        lineUp1Path = new Path(new BezierLine(scorePose, lineUp1Pose));
//        lineUp1Path.setLinearHeadingInterpolation(scorePose.getHeading(), lineUp1Pose.getHeading());
//
//        intake1Path = new Path(new BezierLine(lineUp1Pose, intake1Pose));
//        intake1Path.setConstantHeadingInterpolation(intake1Pose.getHeading());
//        intake1Path.setVelocityConstraint(AutonConstants.INTAKE_1_VEL_CONSTRAINT);
//
////        intermediatePath = new Path(new BezierLine(intake1Pose, lineUp1Pose));
////        intermediatePath.setConstantHeadingInterpolation(intake1Pose.getHeading());
////
////        score1Path = new Path(new BezierLine(lineUp1Pose, scorePose));
////        score1Path.setLinearHeadingInterpolation(lineUp1Pose.getHeading(), scorePose.getHeading());
//
//        score1Path = new Path(new BezierLine(intake1Pose, backToScorePose));
//        score1Path.setLinearHeadingInterpolation(intake1Pose.getHeading(), scorePose.getHeading());
//
//    }
//
//    public void autonomousPaths()
//    {
//        telemetry.addData("Path State", pathState);
//
//        switch (pathState)
//        {
//            case START:
//                if (disableAuto)
//                {
//                    pathState = PathState.STOP;
//                    break;
//                }
//                else
//                {
//                    rightStopper.go();
//                    follower.followPath(preloadPath);
//                    pathState = PathState.TO_PRELOAD_SCORE;
//                    break;
//                }
//            case TO_PRELOAD_SCORE:
//                ((DcMotorEx) outtake.motor.motor).setVelocity(Outtake.MANUAL_ANGULAR_RATE);
//                if (!follower.isBusy())
//                {
//                    pathState = PathState.AT_PRELOAD_SCORE;
//                    pathTimer.resetTimer();
//                }
//                break;
//            case AT_PRELOAD_SCORE:
//                shoot(AutonConstants.FAR_TPS, Outtake.FAR_ERROR_TOLERANCE);
//                if (pathTimer.getElapsedTimeSeconds() > AutonConstants.FAR_PRELOAD_SCORE_TIME)
//                {
//                    intake.forwardRegular();
////                    ((DcMotorEx) outtake.motor.motor).setVelocity(0);
//                    ((DcMotorEx) outtake.motor.motor).setVelocity(-800);
//                    follower.followPath(lineUp1Path);
//                    pathState = PathState.TO_LINEUP;
//                }
//                break;
//            case TO_LINEUP:
//                if (!follower.isBusy())
//                {
////                    ((DcMotorEx) outtake.motor.motor).setVelocity(0);
//                    follower.followPath(intake1Path);
//                    rightStopper.stop();
//                    pathState = PathState.TO_INTAKE1;
//                }
//                break;
//            case TO_INTAKE1:
//                if (!follower.isBusy())
//                {
//                    follower.followPath(score1Path);
//                    pathState = PathState.TO_SCORE1;
//                    pathTimer.resetTimer();
//                }
//                break;
////            case TO_INTERMEDIATE1:
////                if (pathTimer.getElapsedTimeSeconds() >= AutonConstants.DISABLE_INTAKE_SECONDS) intake.stop();
////                if (!follower.isBusy())
////                {
////                    follower.followPath(score1Path);
////                    pathState = PathState.TO_SCORE1;
////                }
////                break;
//            case TO_SCORE1:
//                if (pathTimer.getElapsedTimeSeconds() >= AutonConstants.DISABLE_INTAKE_SECONDS)
//                {
//                    ((DcMotorEx) outtake.motor.motor).setVelocity(Outtake.MANUAL_ANGULAR_RATE);
//                    pathTimer.resetTimer();
//                    intake.reverseLaunch();
//                    rightStopper.go();
//                }
//                if (pathTimer.getElapsedTimeSeconds() >= AutonConstants.REVERSE_INTAKE_SECONDS)
//                {
//                    pathTimer.resetTimer();
//                    intake.stop();
//                }
//                if (!follower.isBusy())
//                {
//                    intake.reverseLaunch();
//                    pathState = PathState.AT_SCORE1;
//                    pathTimer.resetTimer();
//                }
//                break;
//            case AT_SCORE1:
//                if (!score1ReverseLaunchDone && pathTimer.getElapsedTimeSeconds() >= AutonConstants.REVERSE_INTAKE_SECONDS)
//                {
//                    pathTimer.resetTimer();
//                    intake.stop();
//                    score1ReverseLaunchDone = true;
//                }
//                rightStopper.go();
//                shoot(AutonConstants.FAR_TPS, Outtake.FAR_ERROR_TOLERANCE);
//                if (pathTimer.getElapsedTimeSeconds() > AutonConstants.FAR_PRELOAD_SCORE_TIME)
//                {
//                    intake.stop();
//                    ((DcMotorEx) outtake.motor.motor).setVelocity(0);
//                    follower.followPath(score1Path, true);
//                    pathState = PathState.STOP;
//                }
//                break;
//            case STOP:
//                intake.stop();
//                ((DcMotorEx) outtake.motor.motor).setVelocity(0);
//                break;
//        }
//    }
//
//    public void shoot(double ticksPerSecond, double tolerance)
//    {
//        ((DcMotorEx) outtake.motor.motor).setVelocity(ticksPerSecond);
//        double error = ((DcMotorEx) outtake.motor.motor).getVelocity() - ticksPerSecond;
//
//        if (Math.abs(error) < tolerance)
//        {
//            intake.forwardLaunch();
//        }
//        else
//        {
//            intake.stop();
//        }
//
//        telemetry.addData("Error", error);
//    }
//}
