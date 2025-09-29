package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class MashedPotatoRobot {

//GLOBAL VARIABLES AND INIT

final String robotName = "Potato";
    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    public IMU imu;

    //potato
    //mashed potatoes
    //var
    //12345betA

    public void init(final HardwareMap hardwareMap) {
        // Initialize hardware map - declares code version of physical motors using the driver/control hub
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set reverse motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        //encoders
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);
        frontRight.setTargetPosition(0);

        //zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the IMU (gyro/angle sensor)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(imuParameters);

        this.imu.resetYaw();
    }

    //UTILITY FUNCTIONS

    public double distToAngle(double dist){
        //Gives an angle based on distance
        double v0 = (2 * 125 * 0.02) / (0.075); //initial velocity
        double constant = 9.8 / v0;
        double height = 0.98;
        double discriminant = 1 - dist * dist * constant * constant - 2 * height * constant;
        double theta = Math.atan( v0 * (1 + Math.sqrt(discriminant)));
        return theta;
    }
    
    private void setDriveMode(final DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void drive(final double pow){
        //Sets all motors to pow
        frontLeft.setPower(pow);
        backLeft.setPower(pow);
        frontRight.setPower(pow);
        backRight.setPower(pow);
    }

    //TELEOP FUNCTION

    public void gamePadPower(Gamepad gp1, Gamepad gp2, Telemetry telemetry) {
            //Driving(gp1, telemetry);
            potatoDrive(gp1, telemetry);
    }

    public void Driving(Gamepad gp1, Telemetry telemetry){
        //uses driver input to speedup/slowdon as needed
        double multiplier = Math.max(0.3, 1 - gp1.right_trigger);
        double speedMode = (gp1.left_trigger);

        //gets joystick input
        double slowdown = 1.5;
        final double drive = (-gp1.left_stick_y);
        final double turn = (gp1.right_stick_x);
        final double strafe = (gp1.left_stick_x);

        //math to determine power - uses mecanum wheel equation
        flDrivePower = (drive + strafe + turn);
        frDrivePower = (drive - strafe - turn);
        blDrivePower = (drive - strafe + turn);
        brDrivePower = (drive + strafe - turn);

        telemetry.addData("IMU-X", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).firstAngle);
        telemetry.addData("IMU-Y", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).secondAngle);
        telemetry.addData("IMU-Z", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).thirdAngle);

        double imuPos = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
        telemetry.addData("IMU-Angle", imuPos);
        telemetry.update();

        if (speedMode > 0.1){
            slowdown = 1;
        }

        //factor in slowdown and speedup
        frontLeft.setPower(flDrivePower * multiplier / slowdown);
        frontRight.setPower(frDrivePower * multiplier / slowdown);
        backLeft.setPower(blDrivePower * multiplier / slowdown);
        backRight.setPower(brDrivePower * multiplier / slowdown);
    }

    public void potatoDrive (Gamepad gp1, Telemetry telemetry){
        //gyro driving function
        //IMU setup
        telemetry.addData("IMU-X", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).firstAngle);
        telemetry.addData("IMU-Y", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).secondAngle);
        telemetry.addData("IMU-Z", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).thirdAngle);

        //reset button - zeroes the gyro
        if (gp1.x == true){
            imu.resetYaw();
        }

        //get IMU
        double imuPos = (imu.getRobotYawPitchRollAngles().getYaw(RADIANS));
        telemetry.addData("IMU-Angle", imuPos);

        //math - uses trig functions to find the power for each motor
        double change = Math.cos(-imuPos);
        double sine = Math.sin(-imuPos);

        double drive = (-gp1.left_stick_y);
        double strafe = (gp1.left_stick_x);
        double turn = (gp1.right_stick_x);

        double driveCos = change * drive;
        double driveSin = sine * drive;
        double strafeSin = sine * strafe;
        double strafeCos = change * strafe;

        //divided by 1.5 is to account for strafing being slower than driving forward
        double actualDrive = (driveCos + strafeSin) / 1.5;
        double actualStrafe = -driveSin + strafeCos;

        //set power
        flDrivePower = (actualDrive + actualStrafe + turn);
        frDrivePower = (actualDrive - actualStrafe - turn);
        blDrivePower = (actualDrive - actualStrafe + turn);
        brDrivePower = (actualDrive + actualStrafe - turn);

        double slowdown = 0.85;

        //adds or removes to speed based on user input
        if (gp1.right_trigger >= 0.1){
            slowdown = 0.25;
        } else if (gp1.left_trigger >= 0.1) {
            slowdown = 1.25;
        }

        frontLeft.setPower(flDrivePower * slowdown);
        frontRight.setPower(frDrivePower * slowdown);
        backLeft.setPower(blDrivePower * slowdown);
        backRight.setPower(brDrivePower * slowdown);

    }


    //AUTO FUNCTIONS

    private void enableAllMotors(final double p1, final double p2){
        //abstraction, sets power to the TL & BR motors for p1, and TR & BL motors for p2
        frontRight.setPower(p2);
        backRight.setPower(p1);
        frontLeft.setPower(p1);
        backLeft.setPower(p2);
    }

    public void angularStrafe(int degrees, int distance) {
        //Strafes the robot distance in "degrees" direction

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double xDist = 1000;
        double yDist = 1000;
        final double error = 100;
        final double slowdown = 1.8;

        while (Math.abs(yDist - frontRight.getCurrentPosition()) > error || Math.abs(xDist - frontLeft.getCurrentPosition()) > error) {

            double Pitch = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES); //makes sure that it strafes relative to the starting IMU pos
            double angleInRadians = Math.toRadians(degrees + 315 - Pitch); //0 deg, would strafe right, 90 deg would strafe up, etc.
            double cosValue = Math.cos(angleInRadians);
            double sinValue = Math.sin(angleInRadians);
            yDist = distance * sinValue;
            xDist = distance * cosValue;

            double x = cosValue;
            double y = sinValue;

            enableAllMotors(x / slowdown, y / slowdown);
        }
            drive(0.0);

    }

    public void strafe(int pos) {
        //strafes the robot left and right
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int flBrPos = pos;
        int frBlPos = -pos;
        double extra = 0.1;
        final double tickDistance = 40;

        if (pos > 0){
            extra = 0.1;
        } else {
            extra = -0.1;
        }

        while (Math.abs(flBrPos - backRight.getCurrentPosition()) > tickDistance || Math.abs(flBrPos - frontLeft.getCurrentPosition()) > tickDistance) {
            int flDistance = flBrPos - frontLeft.getCurrentPosition();
            int frDistance = frBlPos - frontRight.getCurrentPosition();
            int blDistance = frBlPos - backLeft.getCurrentPosition();
            int brDistance = flBrPos - backRight.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(flBrPos);
            blDrivePower = (double)blDistance / (double)Math.abs(frBlPos);
            frDrivePower = (double)frDistance / (double)Math.abs(frBlPos);
            brDrivePower = (double)brDistance / (double)Math.abs(flBrPos);

            frontLeft.setPower(flDrivePower / 2 + extra);
            frontRight.setPower(frDrivePower / 2 - extra);
            backLeft.setPower(blDrivePower / 2 - extra);
            backRight.setPower(brDrivePower / 2 + extra);

        }

        drive(0.0);

        try {Thread.sleep(50);} catch (InterruptedException e) {}
    }

    public void driveTo(final int pos) {
        //Drives pos distance
        if (pos == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final int delay = 20;
        final int THRESHOLD = 35;
        double ADDITIONAL_SPEED;

        if (pos >= 0){
            ADDITIONAL_SPEED = 0.1;
        }else {
            ADDITIONAL_SPEED = -0.1;
        }

        while (Math.abs(pos - frontLeft.getCurrentPosition()) > THRESHOLD || Math.abs(pos - frontRight.getCurrentPosition()) > THRESHOLD) {
            int flDistance = pos - frontLeft.getCurrentPosition();
            int frDistance = pos - frontRight.getCurrentPosition();
            int blDistance = pos - backLeft.getCurrentPosition();
            int brDistance = pos - backRight.getCurrentPosition();

            ADDITIONAL_SPEED = (flDistance / Math.abs(flDistance)) * 0.1;


            flDrivePower = (double) flDistance / (double) Math.abs(pos);
            blDrivePower = (double) blDistance / (double) Math.abs(pos);
            frDrivePower = (double) frDistance / (double) Math.abs(pos);
            brDrivePower = (double) brDistance / (double) Math.abs(pos);

            frontLeft.setPower(flDrivePower / 3 + ADDITIONAL_SPEED);
            frontRight.setPower(frDrivePower / 3 + ADDITIONAL_SPEED);
            backLeft.setPower(blDrivePower / 3 + ADDITIONAL_SPEED);
            backRight.setPower(brDrivePower / 3 + ADDITIONAL_SPEED);

            try {
                Thread.sleep(delay);
            } catch (InterruptedException e) {
            }
        }

        drive(0.0);
    }

    public void turn(double target) {
        //Target is how many degrees, turns slowly but accurately
        this.imu.resetYaw();

        if (target == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
        double error = target - currentPosition;

        double kp = 1;

        final int DELAY = 50;

        while (Math.abs(error) > 2) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180 * kp);
            final double mult = 2.4;

            flDrivePower = -turn * mult;
            frDrivePower = turn * mult;
            blDrivePower = -turn * mult;
            brDrivePower = turn * mult;

            frontLeft.setPower(flDrivePower / 2);
            frontRight.setPower(frDrivePower / 2);
            backLeft.setPower(blDrivePower / 2);
            backRight.setPower(brDrivePower / 2);

            try {Thread.sleep(DELAY);} catch (InterruptedException e) {}
        }

        drive(0.0);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {Thread.sleep(50);} catch (InterruptedException e) {}
    }

        public void fastTurn(int deg){
        //a faster(but less accurate) version of the turn function
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        this.imu.resetYaw();
        double pos = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);

        //get within 15 degrees of target
        while(Math.abs(deg - pos) > 15){
            pos = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);

            if (deg - pos >= 0){
                frontLeft.setPower(0.8);
                frontRight.setPower(-0.8);
                backRight.setPower(-0.8);
                backLeft.setPower(0.8);
            } else {
                frontLeft.setPower(-0.8);
                frontRight.setPower(0.8);
                backRight.setPower(0.8);
                backLeft.setPower(-0.8);
            }

        }

        //finishing turning with the more accurate function
        turn(deg - pos);

    }
    
}
