package org.firstinspires.ftc.teamcode.auton.comp;

import com.pedropathing.geometry.Pose;

public class AutonConstants
{
    public static final Pose RED_START = new Pose(122.1927409261577, 124.35544430538174, Math.toRadians(37));
    public static final Pose RED_SCORE = new Pose(94.77570093457945, 104.74766355140187, Math.toRadians(37));
    public static final Pose RED_LINEUP_1 = new Pose(93.29085681426106, AutonConstants.PICKUP_Y_POS_1, Math.toRadians(180));
    public static final Pose RED_INTAKE_1 = new Pose(AutonConstants.PICKUP_X_POS, AutonConstants.PICKUP_Y_POS_1, Math.toRadians(180));
    public static final Pose RED_LEAVE_1 = new Pose(120, 80, Math.toRadians(45));

    public static final Pose BLUE_START = AutonConstants.mirror(RED_START);
    public static final Pose BLUE_SCORE = AutonConstants.mirror(RED_SCORE);
    public static final Pose BLUE_LINEUP_1 = AutonConstants.mirrorShift(RED_LINEUP_1, 0, 8);
    public static final Pose BLUE_INTAKE_1 = AutonConstants.mirrorShift(RED_INTAKE_1, 0, 8);
    public static final Pose BLUE_LEAVE_1 = AutonConstants.mirror(RED_LEAVE_1);

    public static final double INTAKE_1_VEL_CONSTRAINT = 2.0;
    public static final double PRELOAD_SCORE_TIME = 5.25;
    public static final double FIRST_THREE_SCORE_TIME = 7.3;
    public static final double DISABLE_INTAKE_SECONDS = 0.30;
    public static final double PICKUP_Y_POS_1 = 70.5;
    public static final double PICKUP_X_POS = 132.5;
    public static final double REVERSE_INTAKE_SECONDS = 0.21;

    public static final double FAR_PRELOAD_SCORE_TIME = 15;
    public static final double FAR_TPS = 1700;
    public static final double PICKUP_Y_POS_FAR = 35.5;
    public static final double SCORE_X_POS_FAR = 80;
    public static final double SCORE_ANGLE_DEG = 63;


    public static final Pose FAR_RED_START = new Pose(SCORE_X_POS_FAR, 9, Math.toRadians(90));
    public static final Pose FAR_RED_SCORE = new Pose(SCORE_X_POS_FAR, PICKUP_Y_POS_FAR, Math.toRadians(SCORE_ANGLE_DEG));
    public static final Pose FAR_RED_LINEUP = new Pose(100, PICKUP_Y_POS_FAR, Math.toRadians(180));
    public static final Pose FAR_RED_INTAKE = new Pose(135, PICKUP_Y_POS_FAR, Math.toRadians(180));
    public static final Pose FAR_RED_SCORE_1 = new Pose(SCORE_X_POS_FAR, PICKUP_Y_POS_FAR, Math.toRadians(SCORE_ANGLE_DEG));

    public static final Pose FAR_BLUE_START = mirror(FAR_RED_START);
    public static final Pose FAR_BLUE_SCORE = mirror(FAR_RED_SCORE);
    public static final Pose FAR_BLUE_LINEUP = mirror(FAR_RED_LINEUP);
    public static final Pose FAR_BLUE_INTAKE = mirror(FAR_RED_INTAKE);
    public static final Pose FAR_BLUE_SCORE_1 = mirror(FAR_RED_SCORE_1);

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
