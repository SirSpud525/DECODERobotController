package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MashedPotatoRobot;

@TeleOp(name = "DECODETeleOp", group = "TeleOp")
public class DECODETeleOp extends OpMode{

MashedPotatoRobot robot = new MashedPotatoRobot();
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.gamePadPower(gamepad1, gamepad2, telemetry);
    }


}
