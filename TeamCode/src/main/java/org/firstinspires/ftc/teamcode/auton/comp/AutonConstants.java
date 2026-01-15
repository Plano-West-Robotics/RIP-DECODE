package org.firstinspires.ftc.teamcode.auton.comp;

import com.pedropathing.geometry.Pose;

public class AutonConstants
{
    public static final double INTAKE_1_VEL_CONSTRAINT = 4;
    public static final double SHOOT_THREE_BALLS_SECONDS = 5.8;
    public static final double DISABLE_INTAKE_SECONDS = 0.325;

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
