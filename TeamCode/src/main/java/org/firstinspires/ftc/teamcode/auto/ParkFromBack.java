package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Camera;
import org.firstinspires.ftc.teamcode.DECODERobot;

@Autonomous(name = "ParkFromBack", group = "Auto")
public class ParkFromBack extends LinearOpMode {
    DECODERobot robot = new DECODERobot(telemetry);

    Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        robot.driveTo(1000, 2);
        robot.turnTo(180);
        robot.shootControl(0.5);
        robot.updateMotorPowers(5000);
        robot.intakeControl(1.0);
        robot.spinnerControl(1.0);
        robot.updateMotorPowers(5000);
        robot.spinnerControl(0.0);
        robot.intakeControl(0.0);
        robot.shootControl(0.0);
        robot.updateMotorPowers();

        stop();
    }
}
