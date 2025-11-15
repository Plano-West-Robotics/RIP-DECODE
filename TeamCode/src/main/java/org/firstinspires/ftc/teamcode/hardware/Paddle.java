package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.templates.StageServoPair;

public class Paddle extends StageServoPair<Paddle.Stage>
{
    public static final double POSITION_DIFF = 0.01;

    public enum Stage
    {
        STANDBY,
        READY,
    }

    public Paddle(HardwareMap hardwareMap)
    {
        super(
            new StageServoPairBuilder<>(
            hardwareMap,
            "pL",
            "pR",
            Stage.class,
            POSITION_DIFF
            )
            .add(Stage.STANDBY, 0.01)
            .add(Stage.READY, 0.02)
        );
    }

    public void standby()
    {
        setStage(Stage.STANDBY);
    }

    public void ready()
    {
        setStage(Stage.READY);
    }
}
