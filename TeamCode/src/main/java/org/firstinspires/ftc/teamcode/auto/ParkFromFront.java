package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Camera;
import org.firstinspires.ftc.teamcode.DECODERobot;

@Autonomous(name = "ParkFromFront", group = "Auto")
public class ParkFromFront extends LinearOpMode {
    DECODERobot robot = new DECODERobot(telemetry);

    Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        robot.driveTo(100);
        //to shoot if running
        robot.turnToFromHere(-45);
        robot.shootControl(0.7);
        robot.updateMotorPowers(3000);
        robot.shootControl(0.0);
        robot.updateMotorPowers();
        //go back and block
        robot.turnToFromHere(45);
        robot.driveTo(-50);
        //park if we can
        robot.strafeTo(50);

        stop();
    }
}
