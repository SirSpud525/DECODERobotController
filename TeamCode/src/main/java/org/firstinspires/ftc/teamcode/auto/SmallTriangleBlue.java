package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Camera;
import org.firstinspires.ftc.teamcode.DECODERobot;

@Autonomous(name = "SmallTriangleBlue", group = "Auto")
public class SmallTriangleBlue extends LinearOpMode {
    DECODERobot robot = new DECODERobot(telemetry);

    Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.setOffsets(1640, 425, 180);

        waitForStart();
        robot.updateMotorPowers(250);
        robot.driveTo(-310, 2);
        robot.turnToFromHere(25);

        shoot();

        robot.turnToFromHere(0);
        robot.driveTo(-800, 2);

        stop();
    }

    public void shoot(){

        robot.shootControl(0.45);
        robot.updateMotorPowers(7000);
        robot.spinnerControl(1.0);
        robot.intakeControl(1.0);
        robot.updateMotorPowers(600);

        robot.spinnerControl(0.0);
        robot.intakeControl(0.0);
        robot.updateMotorPowers(2800);

        robot.spinnerControl(1.0);
        robot.intakeControl(1.0);
        robot.updateMotorPowers(1800);

        // move
//        double power = -1.0;
//        for (int i = 0; i < 10; i++) {
//            robot.spinnerControl(power);
//                robot.updateMotorPowers(100);
//            power *= -1;
//        }

        robot.intakeControl(0.0);
        robot.spinnerControl(0.0);
        robot.updateMotorPowers(2000);

        robot.intakeControl(1.0);
        robot.spinnerControl(1.0);
        robot.updateMotorPowers(6000);

        robot.intakeControl(0.0);
        robot.intakeControl(0.0);
        robot.shootControl(0.0);
        robot.updateMotorPowers();
    }


}

