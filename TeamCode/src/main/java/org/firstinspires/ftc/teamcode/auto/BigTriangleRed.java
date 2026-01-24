package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Camera;
import org.firstinspires.ftc.teamcode.DECODERobot;

@Autonomous(name = "BigTriangleRed", group = "Auto")
public class BigTriangleRed extends LinearOpMode {
    DECODERobot robot = new DECODERobot(telemetry);

    Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.setOffsets(3080, 3560, 128);

        waitForStart();

        robot.updateMotorPowers(250);
        robot.driveTo(2050, 2);

        shoot();

        robot.strafeTo(-1300);

//        shootAgain();

        stop();
    }
    public void shootAgain() // after the robot shoots its already equipped 3 artifacts, it picks up 3 more that are nearby
    {
        robot.turnTo(135);
        robot.intakeControl(1.0);
        robot.updateMotorPowers();
        robot.driveTo(1200,2);
        robot.sleep(500);
        robot.driveTo(-1200, 1.5);
        robot.turnTo(-135);
        robot.intakeControl(0);
        robot.updateMotorPowers();
//        shoot();

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



    // move
//        double power = -1.0;
//        for (int i = 0; i < 10; i++) {
//            robot.spinnerControl(power);
//            robot.updateMotorPowers(100);
//            power *= -1;
//        }



}
