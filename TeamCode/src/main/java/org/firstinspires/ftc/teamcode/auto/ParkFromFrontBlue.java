package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Camera;
import org.firstinspires.ftc.teamcode.DECODERobot;

@Autonomous(name = "ParkFromFrontBlue", group = "Auto")
public class ParkFromFrontBlue extends LinearOpMode {
    DECODERobot robot = new DECODERobot(telemetry);

    Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        robot.driveTo(300, 2);
        //to shoot if running
        robot.turnToFromHere(-155);
        robot.shootControl(0.75);
        robot.updateMotorPowers(5000);
        robot.spinnerControl(1.0);
        robot.intakeControl(1.0);
        robot.updateMotorPowers(3000);
        robot.intakeControl(0.0);
        robot.intakeControl(0.0);
        robot.shootControl(0.0);
        robot.updateMotorPowers();
        robot.turnToFromHere(0);
        robot.driveTo(300, 2);

        stop();
    }
}
