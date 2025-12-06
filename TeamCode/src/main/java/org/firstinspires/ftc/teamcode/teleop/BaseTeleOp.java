/**
 * Subclasses of BaseTeleOp should override the setup() and run() to run code during init() and
 * loop(); init() and loop() should not be overridden, but init_loop() and start() can be.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

public abstract class BaseTeleOp extends OpMode
{
    public Hardware hardware;
    public Gamepads gamepads;

    @Override
    public final void init()
    {
        hardware = new Hardware(hardwareMap);
        gamepads = new Gamepads(gamepad1, gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        setup();
        telemetry.update();
    }

    public void setup() {}

    @Override
    public final void loop()
    {
        gamepads.update(gamepad1, gamepad2);
        run();
        telemetry.update();
    }

    public void run() {}
}
