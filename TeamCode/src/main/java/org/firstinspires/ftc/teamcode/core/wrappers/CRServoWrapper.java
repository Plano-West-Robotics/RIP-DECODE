package org.firstinspires.ftc.teamcode.core.wrappers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CRServoWrapper
{
    public com.qualcomm.robotcore.hardware.CRServo servo;

    public CRServoWrapper(HardwareMap hardwareMap, String name)
    {
        servo = hardwareMap.get(CRServo.class, name);
    }

    public void forward()
    {
        setDirection(CRServo.Direction.FORWARD);
    }

    public void reverse()
    {
        setDirection(CRServo.Direction.REVERSE);
    }

    public void setPower(double power)
    {
        servo.setPower(power);
    }

    public double getPower()
    {
        return servo.getPower();
    }

    public CRServo.Direction getDirection()
    {
        return servo.getDirection();
    }

    public void setDirection(CRServo.Direction direction)
    {
        servo.setDirection(direction);
    }

    public String getName()
    {
        return servo.getDeviceName();
    }
}
