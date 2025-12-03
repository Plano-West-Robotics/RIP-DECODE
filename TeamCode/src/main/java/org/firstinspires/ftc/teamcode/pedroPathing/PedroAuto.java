//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.hardware.Hardware;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake;
//
//@Autonomous
//public class PedroAuto extends OpMode {
//    public Hardware hardware;
//    public Outtake outtake;
//    public Intake intake;
//    private Follower follower;
//    private ElapsedTime pathTimer, actionTimer, opModeTimer;
//
//    private final Pose startPose = new Pose(120.22429906542055, 130.76635514018693, Math.toRadians(37));
//    private final Pose scorePose = new Pose(96, 111.25233644859813, Math.toRadians(45));
//    //private final Pose park = new Pose(38.57943925233644, 33.86915887850467, Math.toRadians(90));
//
//    private enum PathState
//    {
//        START,
//        START_TO_SCORE,
//        AT_SCORE,
//        SCORED
//    }
//    private PathState pathState = PathState.START;
//
//    private Path scorePreload;
//    public void buildPaths()
//    {
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//        //scoreToPark = new Path(new BezierLine(scorePose, park));
//        //scoreToPark.setLinearHeadingInterpolation(scorePose.getHeading(), park.getHeading());
//    }
//
//    public void autonomousPaths()
//    {
//        switch (pathState)
//        {
//            case START:
//                follower.followPath(scorePreload);
//                pathState = PathState.START_TO_SCORE;
//                intake.motor.setPower(Intake.REGULAR_POWER);
//                intake.transfer.setPower(1);
//                outtake.motor.setPower(0.55);
//                outtake.paddles.standby();
//                break;
//            case START_TO_SCORE:
//                if (!follower.isBusy()) {
//                    pathState = PathState.AT_SCORE;
//                    pathTimer.reset();
//                }
//                break;
//            case AT_SCORE:
//                outtake.paddles.ready();
//                if (pathTimer.milliseconds() > 1500)
//                {
//                    pathState = PathState.SCORED;
//                }
//                break;
//            case SCORED:
//                break;
//        }
//    }
//    @Override
//    public void init() {
//        hardware = new Hardware(hardwareMap);
//        outtake = new Outtake(hardware);
//        intake = new Intake(hardware);
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        autonomousPaths();
//    }
//}
