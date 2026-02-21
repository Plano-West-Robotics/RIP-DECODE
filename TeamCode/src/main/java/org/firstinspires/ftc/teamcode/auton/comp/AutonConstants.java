package org.firstinspires.ftc.teamcode.auton.comp;

import com.pedropathing.geometry.Pose;

public class AutonConstants
{
    public static final Pose RED_START = new Pose(122.1927409261577, 124.35544430538174, Math.toRadians(37));
    public static final Pose RED_SCORE = new Pose(94.77570093457945, 104.74766355140187, Math.toRadians(37));
    public static final Pose RED_LINEUP_1 = new Pose(93.29085681426106, AutonConstants.PICKUP_Y_POS_1, Math.toRadians(0));
    public static final Pose RED_INTAKE_1 = new Pose(AutonConstants.PICKUP_X_POS, AutonConstants.PICKUP_Y_POS_1, Math.toRadians(0));
    public static final Pose RED_LEAVE_1 = new Pose(120, 80, Math.toRadians(0));

    public static final Pose BLUE_START = new Pose(21.807259073842303, 124.35544430538174, Math.toRadians(143));
    public static final Pose BLUE_SCORE = new Pose(49.22429906542055, 104.74766355140187, Math.toRadians(143));
    public static final Pose BLUE_LINEUP_1 = new Pose(50.70914318573894, AutonConstants.BLUE_PICKUP_Y_POS_1, Math.toRadians(180));
    public static final Pose BLUE_INTAKE_1 = new Pose(AutonConstants.BLUE_PICKUP_X_POS, AutonConstants.BLUE_PICKUP_Y_POS_1, Math.toRadians(180));
    public static final Pose BLUE_LEAVE_1 = new Pose(24, 80, Math.toRadians(180));

    public static final double INTAKE_1_VEL_CONSTRAINT = 2.0;
    public static final double PRELOAD_SCORE_TIME = 4;
    public static final double FIRST_THREE_SCORE_TIME = 5.5;
    public static final double DISABLE_INTAKE_SECONDS = 0.30;
    public static final double PICKUP_Y_POS_1 = 84.5;
    public static final double PICKUP_X_POS = 131;
    public static final double REVERSE_INTAKE_SECONDS = 0.21;

    public static final double BLUE_PICKUP_Y_POS_1 = 84.5;
    public static final double BLUE_PICKUP_X_POS = 13;

    public static final double FAR_PRELOAD_SCORE_TIME = 15;
    public static final double FAR_TPS = 1600;
    public static final double PICKUP_Y_POS_FAR = 34;
    public static final double START_X_POS_FAR = 88;
    public static final double SCORE_X_POS_FAR = 88;
    public static final double SCORE_Y_POS_FAR = 20;
    public static final double SCORE_ANGLE_DEG = 69.94;


    public static final Pose FAR_RED_START = new Pose(START_X_POS_FAR, 9, Math.toRadians(90));
    public static final Pose FAR_RED_SCORE = new Pose(START_X_POS_FAR, SCORE_Y_POS_FAR, Math.toRadians(SCORE_ANGLE_DEG));
    public static final Pose FAR_RED_LINEUP = new Pose(START_X_POS_FAR, PICKUP_Y_POS_FAR, Math.toRadians(0));
    public static final Pose FAR_RED_INTAKE = new Pose(136, PICKUP_Y_POS_FAR, Math.toRadians(0));
    public static final Pose FAR_RED_LEAVE = new Pose(106, 34, Math.toRadians(0));

    //public static final Pose FAR_RED_SCORE_1 = new Pose(SCORE_X_POS_FAR, PICKUP_Y_POS_FAR, Math.toRadians(SCORE_ANGLE_DEG));



    public static final Pose FAR_BLUE_START = new Pose(56, 9, Math.toRadians(90));
    public static final Pose FAR_BLUE_SCORE = new Pose(56, SCORE_Y_POS_FAR, Math.toRadians(110.06));
    public static final Pose FAR_BLUE_LINEUP = new Pose(56, PICKUP_Y_POS_FAR, Math.toRadians(180));
    public static final Pose FAR_BLUE_INTAKE = new Pose(8, PICKUP_Y_POS_FAR, Math.toRadians(180));
    public static final Pose FAR_BLUE_LEAVE = new Pose(38, 34, Math.toRadians(180));

    public static final int PATH_COUNT_UPPER_BOUND = 20;

    public static Pose mirror(Pose p)
    {
        double heading = p.getHeading();
        heading = Math.toRadians(180) - heading;
        return new Pose(144 - p.getX(),
                p.getY() /* + 20 */,
                heading);
    }

    public static Pose mirrorShift(Pose p, double x, double y)
    {
        double heading = p.getHeading();
        heading = Math.toRadians(180) - heading;
        return new Pose((144 - p.getX()) - x, // shift is relative to previous incorrect blue position
                p.getY() + y,
                heading);
    }
}
