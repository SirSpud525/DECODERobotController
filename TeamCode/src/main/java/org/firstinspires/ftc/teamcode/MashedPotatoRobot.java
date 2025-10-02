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
    public ElapsedTime driveModeDebounce = new ElapsedTime();
    private ElapsedTime pushUpDebounce = new ElapsedTime();
    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    public boolean gyroDrive = true;
    public boolean pushing = false;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor slide1;
    private DcMotor slide2;
    public IMU imu;

    //potato
    //mashed potatoes
    //var
    //12345betA

    public void init(final HardwareMap hardwareMap) {
        // Initialize hardware map - declares code version of physical motors using the driver/control hub
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft"); //4
        frontRight = hardwareMap.get(DcMotor.class, "frontRight"); //3
        backLeft = hardwareMap.get(DcMotor.class, "backLeft"); //1
        backRight = hardwareMap.get(DcMotor.class, "backRight"); //2
        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide2 = hardwareMap.get(DcMotor.class, "slide2");

        // Set reverse motors

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
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void handleMotorPower(){

        frontLeft.setPower(flDrivePower);
        frontRight.setPower(frDrivePower);
        backLeft.setPower(blDrivePower);
        backRight.setPower(brDrivePower);

    }
    public void brake() {
        flDrivePower = 0.0;
        blDrivePower = 0.0;
        frDrivePower = 0.0;
        brDrivePower = 0.0;

        frontLeft.setPower(0.0);
        backLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backRight.setPower(0.0);
    }
    public void sleep(int milliseconds) {
        try {Thread.sleep(milliseconds);} catch (InterruptedException ignored) {}
    }
    public double distToAngle(double dist){
        //Gives an angle based on distance
        double v0 = (2 * 125 * 0.02) / (0.075); //initial velocity
        double constant = 9.8 / v0;
        double height = 0.9845;
        double discriminant = 1 - dist * dist * constant * constant - 2 * height * constant;
        double theta = Math.atan( v0 * (1 + Math.sqrt(discriminant)) / (9.8 * dist)); 
        return theta; //Returns angle
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

    //TELEOP FUNCTIONS

    public void gamePadPower(Gamepad gp1, Gamepad gp2, Telemetry telemetry) {
            if (gyroDrive){
                gyroDrive(gp1, telemetry);
            }else{
                Driving(gp1, telemetry);
            }
            toggleGyro(gp1);
            pushUp(gp2);
    }

    public void toggleGyro(Gamepad gp1){
        //Toggles gyro on or off

        //toggles GyroDrive boolean
        if (gp1.x){
            if (driveModeDebounce.milliseconds() < 500) return;
            driveModeDebounce.reset();

            gyroDrive = !gyroDrive;
        }
    }
    public void pushUp(Gamepad gp2){
        if (gp2.x){
            if (pushUpDebounce.milliseconds() < 500) return;

            pushUpDebounce.reset();
            pushing = !pushing;
        }

        if (pushing){
            slide1.setPower(0.1);
            slide2.setPower(0.1);
        } else {
            slide1.setPower(0.0);
            slide2.setPower(0.0);
        }

    }
    public void Driving(Gamepad gp1, Telemetry telemetry){
        //uses driver input to speedup/slowdown as needed
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

    public void gyroDrive (Gamepad gp1, Telemetry telemetry){
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
        frDrivePower = p2;
        brDrivePower = p1;
        flDrivePower = p1;
        blDrivePower = p2;
    }

    public void angularStrafe(int degrees, int distance) {
        //Strafes the robot distance in "degrees" direction

        //changes here should be fine, but the commented old version stays until it's tested, then delete it
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            handleMotorPower();
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

            flDrivePower = flDrivePower / 2 + extra;
            frDrivePower = frDrivePower / 2 - extra;
            blDrivePower = blDrivePower / 2 - extra;
            brDrivePower = brDrivePower / 2 + extra;

            handleMotorPower();

        }

        drive(0.0);

        try {Thread.sleep(50);} catch (InterruptedException e) {}
    }
    public void accurateStrafe(int pos) {
        double targetAngle = 0;

        double currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double errorAngle = targetAngle - currentAngle;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;

        int flBrPos = pos;
        int frBlPos = -pos;

        while (Math.abs((Math.abs(flBrPos - frontLeft.getCurrentPosition()) + Math.abs(frBlPos - frontRight.getCurrentPosition())) / 2) > 20) {
            int flDistance = flBrPos - frontLeft.getCurrentPosition();
            int frDistance = frBlPos - frontRight.getCurrentPosition();
            int blDistance = frBlPos - backLeft.getCurrentPosition();
            int brDistance = flBrPos - backRight.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(flBrPos);
            blDrivePower = (double)blDistance / (double)Math.abs(frBlPos);
            frDrivePower = (double)frDistance / (double)Math.abs(frBlPos);
            brDrivePower = (double)brDistance / (double)Math.abs(flBrPos);

            if (Math.abs(flBrPos - backRight.getCurrentPosition()) > 100 || Math.abs(flBrPos - frontLeft.getCurrentPosition()) > 100) {
                flDrivePower = Math.signum(flDrivePower);
                frDrivePower = Math.signum(frDrivePower);
                blDrivePower = Math.signum(blDrivePower);
                brDrivePower = Math.signum(brDrivePower);
            } else {
                flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.15);
                frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.15);
                blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.15);
                brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.15);
            }

            // Correct angle
            currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            errorAngle = targetAngle - currentAngle;

            double kp = 1;

            double proportional = errorAngle * kp;

            double turn = proportional / (180 * kp);

            double flDrivePowerCorrection = -turn;
            double frDrivePowerCorrection = turn;
            double blDrivePowerCorrection = -turn;
            double brDrivePowerCorrection = turn;

            flDrivePowerCorrection = (flDrivePowerCorrection / 2) + (Math.signum(flDrivePowerCorrection) * 0.1);
            frDrivePowerCorrection = (frDrivePowerCorrection / 2) + (Math.signum(frDrivePowerCorrection) * 0.1);
            blDrivePowerCorrection = (blDrivePowerCorrection / 2) + (Math.signum(blDrivePowerCorrection) * 0.1);
            brDrivePowerCorrection = (brDrivePowerCorrection / 2) + (Math.signum(brDrivePowerCorrection) * 0.1);

            flDrivePower += flDrivePowerCorrection;
            frDrivePower += frDrivePowerCorrection;
            blDrivePower += blDrivePowerCorrection;
            brDrivePower += brDrivePowerCorrection;

            // there was a slowdown here, I removed it cus not responsible, could be an issue later

            handleMotorPower();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

            flDrivePower = flDrivePower / 3 + ADDITIONAL_SPEED;
            frDrivePower = frDrivePower / 3 + ADDITIONAL_SPEED;
            blDrivePower = blDrivePower / 3 + ADDITIONAL_SPEED;
            brDrivePower = brDrivePower / 3 + ADDITIONAL_SPEED;

            handleMotorPower();

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

            flDrivePower = flDrivePower / 2;
            frDrivePower = frDrivePower / 2;
            blDrivePower = blDrivePower / 2;
            brDrivePower = brDrivePower / 2;

            handleMotorPower();

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
                flDrivePower = 0.8;
                frDrivePower = -0.8;
                brDrivePower = -0.8;
                blDrivePower = 0.8;
            } else {
                flDrivePower = -0.8;
                frDrivePower = 0.8;
                brDrivePower = 0.8;
                blDrivePower = -0.8;
            }

            handleMotorPower();

        }

        //finishing turning with the more accurate function
        turn(deg - pos);

    }

    public void turnToFromHere(double target) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - currentPosition;

        double kp = 1;

        final int DELAY = 50;

        while (Math.abs(error) > 3) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180 * kp);

            flDrivePower = -turn;
            frDrivePower = turn;
            blDrivePower = -turn;
            brDrivePower = turn;

            flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.1);
            frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.1);
            blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.1);
            brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.1);

            //removed slowdown stuff

            handleMotorPower();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
