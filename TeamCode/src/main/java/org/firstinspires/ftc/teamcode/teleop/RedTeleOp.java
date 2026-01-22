package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DECODERobot;

@TeleOp(name = "RedTeleOp", group = "TeleOp")
public class RedTeleOp extends OpMode {
    DECODERobot robot = new DECODERobot(telemetry);

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.curSide = DECODERobot.side.RED;
    }

    @Override
    public void loop() {
        robot.gamePadPower(gamepad1, gamepad2);
    }
}
