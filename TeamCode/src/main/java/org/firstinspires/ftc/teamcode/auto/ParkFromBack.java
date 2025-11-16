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

        robot.slideControl(0.2);
        robot.updateMotorPowers(2000);
        robot.driveTo(2150, 2);
        robot.shootControl(0.71);
        robot.updateMotorPowers(7000);
        robot.intakeControl(1.0);
        robot.spinnerControl(1.0);
        robot.updateMotorPowers(5000);

        // move
        double power = -1.0;
        for (int i = 0; i < 4; i++) {
            robot.spinnerControl(power);
            robot.updateMotorPowers(200);
            power *= -1;
        }

        robot.intakeControl(1.0);
        robot.spinnerControl(1.0);
        robot.updateMotorPowers(4000);

        robot.spinnerControl(0.0);
        robot.intakeControl(0.0);
        robot.shootControl(0.0);
        robot.updateMotorPowers();

        robot.strafeTo(-1300);
        //- for red, + for blue

        stop();
    }
}
