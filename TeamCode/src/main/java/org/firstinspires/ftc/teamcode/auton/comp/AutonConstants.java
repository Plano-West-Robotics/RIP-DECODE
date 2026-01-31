package org.firstinspires.ftc.teamcode.auton.comp;

import com.pedropathing.geometry.Pose;

public class AutonConstants
{
    public static final double INTAKE_1_VEL_CONSTRAINT = 3.1;
    public static final double PRELOAD_SCORE_TIME = 4.8;
    public static final double FIRST_THREE_SCORE_TIME = 6.3;
    public static final double DISABLE_INTAKE_SECONDS = 0.35;
    public static final double PICKUP_Y_POS_1 = 70.5;
    public static final double REVERSE_INTAKE_SECONDS = 0.2;

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
