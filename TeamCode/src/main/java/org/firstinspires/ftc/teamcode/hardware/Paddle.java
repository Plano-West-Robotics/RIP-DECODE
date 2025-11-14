package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.templates.StageServoPair;

public class Paddle extends StageServoPair<Paddle.Stage>
{
    public static final double POSITION_DIFF = 0.01;

    public enum Stage
    {
        A,
        B,
    }

    public Paddle(HardwareMap hardwareMap)
    {
        super(
            new StageServoPairBuilder<>(
            hardwareMap,
            "paddleL",
            "paddleR",
            Stage.class,
            POSITION_DIFF
            )
            .add(Stage.A, 0.01)
            .add(Stage.B, 0.02)
        );
    }

    public void A()
    {
        setStage(Stage.A);
    }

    public void B()
    {
        setStage(Stage.B);
    }
}
