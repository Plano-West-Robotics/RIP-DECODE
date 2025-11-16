package org.firstinspires.ftc.teamcode.core.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoPairWrapper
{
    public ServoWrapper left, right;
    public double positionDiff;

    public ServoPairWrapper(HardwareMap hardwareMap, String leftServoName, String rightServoName,
                            double positionDiff)
    {
        left = new ServoWrapper(hardwareMap, leftServoName);
        right = new ServoWrapper(hardwareMap, rightServoName);

        /*
        This value means the servo pair is synced up when [left=positionDiff, right=1], or
        [left=1, right=positionDiff].
         */
        this.positionDiff = positionDiff;
    }

    /**
     * Remember that servos do not have encoders!
     * @return the arguments of the left and right servos' last setPosition() call
     */
    public double[] getPosition()
    {
        return new double[] {left.getPosition(), right.getPosition()};
    }

    public void setPosition(double position)
    {
        setPosition(position, 1 + positionDiff - position);
    }

    public void setPosition(double leftPosition, double rightPosition)
    {
        left.setPosition(leftPosition);
        right.setPosition(rightPosition);
    }

    public ServoWrapper getLeft()
    {
        return left;
    }

    public ServoWrapper getRight()
    {
        return right;
    }

    public double getPositionDiff()
    {
        return positionDiff;
    }

    public void setPositionDiff(double positionDiff)
    {
        this.positionDiff = positionDiff;
    }
}
