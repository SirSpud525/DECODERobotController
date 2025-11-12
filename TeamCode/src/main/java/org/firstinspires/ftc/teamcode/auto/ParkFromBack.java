package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.Camera;
import org.firstinspires.ftc.teamcode.DECODERobot;

@Autonomous(name = "ParkFromBack", group = "Auto")
public class ParkFromBack extends LinearOpMode {
    DECODERobot robot = new DECODERobot(telemetry);

    Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        robot.driveTo(100);
        //to shoot if running
        robot.turnToFromHere(180);
        robot.shootControl(0.0);
        //
        robot.turnToFromHere(-135);
        robot.strafe(50);
        robot.driveTo(50);
        //if can park
        robot.strafe(-50);
        
        stop();
    }
}
