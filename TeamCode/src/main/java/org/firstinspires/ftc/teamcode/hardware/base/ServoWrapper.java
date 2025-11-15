package org.firstinspires.ftc.teamcode.hardware.base;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoWrapper {
    public com.qualcomm.robotcore.hardware.Servo servo;

    public ServoWrapper(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.get(Servo.class, name);
    }

    public void forward() {
        setDirection(Servo.Direction.FORWARD);
    }

    public void reverse() {
        setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Remember that servos do not have encoders!
     *
     * @return the argument of the last {@code setPosition()} call
     */
    public double getPosition() {
        return servo.getPosition();
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public Servo.Direction getDirection() {
        return servo.getDirection();
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    public String getName() {
        return servo.getDeviceName();
    }
}
