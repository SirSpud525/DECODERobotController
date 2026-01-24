package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DECODERobot;
import org.firstinspires.ftc.teamcode.utils.Camera;

@Autonomous(name = "BigTriangleBlue", group = "Auto")
public class BigTriangleBlue extends LinearOpMode {
    DECODERobot robot = new DECODERobot(telemetry);

    Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.setOffsets(580, 3560, -133);

        waitForStart();

//        robot.slideControl(0.2);
        robot.updateMotorPowers(250);
        robot.driveTo(2050, 2);

        shoot();

        robot.strafeTo(1300);

        stop();
    }

    public void shoot(){
        robot.shootControl(0.4225);
        robot.updateMotorPowers(6500);
        robot.intakeControl(1.0);
        robot.spinnerControl(1.0);
        robot.updateMotorPowers(600);

        robot.intakeControl(0.0);
        robot.spinnerControl(0.0);
        robot.updateMotorPowers(1750);

        robot.intakeControl(1.0);
        robot.spinnerControl(1.0);
        robot.updateMotorPowers(1800);

// move
//        double power = -1.0;
//        for (int i = 0; i < 10; i++) {
//            robot.spinnerControl(power);
//            robot.updateMotorPowers(100);
//            power *= -1;
//        }

        robot.intakeControl(0.0);
        robot.spinnerControl(0.0);
        robot.updateMotorPowers(2700);

        robot.intakeControl(1.0);
        robot.spinnerControl(1.0);
        robot.updateMotorPowers(4000);

        robot.intakeControl(0.0);
        robot.intakeControl(0.0);
        robot.shootControl(0.0);
        robot.updateMotorPowers();
    }
//potato
}
