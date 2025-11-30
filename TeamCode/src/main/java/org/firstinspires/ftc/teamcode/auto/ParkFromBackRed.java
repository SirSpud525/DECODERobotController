package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Camera;
import org.firstinspires.ftc.teamcode.DECODERobot;

@Autonomous(name = "ParkFromBackRed", group = "Auto")
public class ParkFromBackRed extends LinearOpMode {
    DECODERobot robot = new DECODERobot(telemetry);

    Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

//        robot.slideControl(0.2);
        robot.updateMotorPowers(500);
        robot.driveTo(2300, 2);
        robot.shootControl(0.38);
        robot.updateMotorPowers(5500);
        robot.intakeControl(1.0);
        robot.spinnerControl(1.0);
        robot.updateMotorPowers(600);

        robot.intakeControl(0.0);
        robot.spinnerControl(0.0);
        robot.updateMotorPowers(2500);

        robot.intakeControl(1.0);
        robot.spinnerControl(1.0);
        robot.updateMotorPowers(1000);

// move
        double power = -1.0;
        for (int i = 0; i < 10; i++) {
            robot.spinnerControl(power);
            robot.updateMotorPowers(100);
            power *= -1;
        }

        robot.intakeControl(0.0);
        robot.spinnerControl(0.0);
        robot.updateMotorPowers(1500);

        robot.intakeControl(1.0);
        robot.spinnerControl(1.0);
        robot.updateMotorPowers(2500);

        robot.intakeControl(0.0);
        robot.intakeControl(0.0);
        robot.shootControl(0.0);
        robot.updateMotorPowers();

        robot.strafeTo(-1300);

        stop();
    }
}
