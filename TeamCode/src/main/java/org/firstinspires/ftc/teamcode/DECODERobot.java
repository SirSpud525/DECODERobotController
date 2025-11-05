package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class DECODERobot {
    Telemetry telemetry;

    private final ElapsedTime driveModeDebounce = new ElapsedTime();
    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    public double driveSlowdown = 2;
    public DriveMode driveMode = DriveMode.NORMAL;
    private DcMotor flDrive;
    private DcMotor frDrive;
    private DcMotor blDrive;
    private DcMotor brDrive;

    public double slidePower;
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    public double intakePower;
    private DcMotor intake;

    public double shootPower;
    private DcMotor shoot;

    public double spinnerPower = 0.0;
    private CRServo tlSpinner;
    private CRServo trSpinner;
    private CRServo blSpinner;
    private CRServo brSpinner;

    private final ElapsedTime lockDebounce = new ElapsedTime();
    public boolean locked = false;

    public IMU imu;
    public GoBildaPinpointDriver odometry;

    public DECODERobot(Telemetry robotTelemetry) {
        telemetry = robotTelemetry;
    }

    public void init(HardwareMap hardwareMap) {
        // Initialize hardware map - declares code version of physical motors as mapped in the driver hub
        flDrive = hardwareMap.get(DcMotor.class, "flDrive"); // Port
        frDrive = hardwareMap.get(DcMotor.class, "frDrive"); // Port
        blDrive = hardwareMap.get(DcMotor.class, "blDrive"); // Port
        brDrive = hardwareMap.get(DcMotor.class, "brDrive"); // Port
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide"); // Port
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide"); // Port
        intake = hardwareMap.get(DcMotor.class, "intake"); // Port
        shoot = hardwareMap.get(DcMotor.class, "shoot"); // Port

        tlSpinner = hardwareMap.get(CRServo.class, "tlSpinner");
        trSpinner = hardwareMap.get(CRServo.class, "trSpinner");
        blSpinner = hardwareMap.get(CRServo.class, "blSpinner");
        brSpinner = hardwareMap.get(CRServo.class, "brSpinner");

        // Reverse motors
        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        brDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        trSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        brSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset all motors
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Brake motors when they have zero power
        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the IMU (gyro/angle sensor)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(imuParameters);

        this.imu.resetYaw();

        //Initializing odometry
//        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//
//        odo.setOffsets(37.6, 53.0, DistanceUnit.MM);
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//
//        odo.resetPosAndIMU();
        //odo.setPosition(new Pose2D(DistanceUnit.MM, -923.925, 1601.47, RADIANS, 0));
    }

    public void updateMotorPowers(int waitAfter) {
        updateMotorPowers();

        // Wait (to allow the action done to be performed)
        sleep(waitAfter);
    }

    public void updateMotorPowers() {
        flDrive.setPower(flDrivePower / driveSlowdown);
        frDrive.setPower(frDrivePower / driveSlowdown);
        blDrive.setPower(blDrivePower / driveSlowdown);
        brDrive.setPower(brDrivePower / driveSlowdown);

        leftSlide.setPower(slidePower);
        rightSlide.setPower(slidePower);

        intake.setPower(intakePower);

        shoot.setPower(shootPower);

        tlSpinner.setPower(spinnerPower);
        trSpinner.setPower(spinnerPower);
        blSpinner.setPower(spinnerPower);
        brSpinner.setPower(spinnerPower);
    }

    public void updateTelemetry() {
        telemetry.addData("FL Drive Power: ", flDrivePower);
        telemetry.addData("FR Drive Power: ", frDrivePower);
        telemetry.addData("BL Drive Power: ", blDrivePower);
        telemetry.addData("BR Drive Power: ", brDrivePower);
        telemetry.addData("Drive Slowdown: ", driveSlowdown);
        telemetry.addData("Drive Mode: ", driveMode);
        telemetry.addData("Angle: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Slide Power: ", slidePower);
        telemetry.addData("Intake Power: ", intakePower);
        telemetry.addData("Shoot Power: ", shootPower);
        telemetry.addData("Spinner Power: ", spinnerPower);

        telemetry.update();
    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2) {
        setLock(gp2);

        if (locked) {
            mainDrive(0, 0, 0, 1, 1);
            spinnerControl(0.0);
            slideControl(-0.2);
            intakeControl(0.0);
            shootControl(0.0);
        } else {
            setDrivingMode(gp1);

            if (driveMode == DriveMode.NORMAL) {
                mainDrive(gp1);
            } else {
                mainGyroDrive(gp1);
            }

            spinnerControl(0.0);

            slideControl(gp2);
            intakeControl(gp2);
            shootControl(gp2);
        }

        // Update state
        updateMotorPowers();

        // Update telemetry
        updateTelemetry();
    }

    public void setDrivingMode(Gamepad gp) {
        if (gp.x) {
            if (driveModeDebounce.milliseconds() < 500) return;

            driveModeDebounce.reset();

            if (driveMode == DriveMode.NORMAL) {
                setDrivingMode(DriveMode.GYRO);
            } else {
                setDrivingMode(DriveMode.NORMAL);
            }
        }
    }

    public void setDrivingMode(DriveMode mode) {
        driveMode = mode;
    }

    public void mainDrive(Gamepad gp) {
        double drive = -gp.left_stick_y;
        double turn = gp.right_stick_x;
        double strafe = gp.left_stick_x;
        double slowDownFactor = gp.right_trigger;
        double speedUpFactor = gp.left_trigger;

        mainDrive(drive, turn, strafe, slowDownFactor, speedUpFactor);
    }

    public void mainDrive(double drive, double turn, double strafe, double slowDownFactor, double speedUpFactor) {
        flDrivePower = drive + strafe + turn;
        frDrivePower = drive - strafe - turn;
        blDrivePower = drive - strafe + turn;
        brDrivePower = drive + strafe - turn;

        if (slowDownFactor > 0.5 && speedUpFactor > 0.5) {
            driveSlowdown = 2;
        } else if (slowDownFactor > 0.5) {
            driveSlowdown = 4;
        } else if (speedUpFactor > 0.5) {
            driveSlowdown = 1;
        } else {
            driveSlowdown = 2;
        }
    }

    public void mainGyroDrive(Gamepad gp) {
        double drive = -gp.left_stick_y;
        double turn = gp.right_stick_x;
        double strafe = gp.left_stick_x;
        double slowDownFactor = gp.right_trigger;
        double speedUpFactor = gp.left_trigger;

        if (gp.a) {
            imu.resetYaw();
        }

        mainGyroDrive(drive, turn, strafe, slowDownFactor, speedUpFactor);
    }

    public void mainGyroDrive(double drive, double turn, double strafe, double slowDownFactor, double speedUpFactor) {
        double imuPos = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double cosine = Math.cos(-imuPos);
        double sine = Math.sin(-imuPos);

        double driveCos = cosine * drive;
        double driveSin = sine * drive;
        double strafeCos = cosine * strafe;
        double strafeSin = sine * strafe;

        double actualDrive = (driveCos + strafeSin) / 1.5;
        double actualStrafe = strafeCos - driveSin;

        mainDrive(actualDrive, turn, actualStrafe, slowDownFactor, speedUpFactor);
    }

    public void slideControl(Gamepad gp) {
        boolean contract = gp.left_bumper && !gp.right_bumper;
        boolean extend = gp.right_bumper && !gp.left_bumper;

        if (contract) {
            slideControl(1.0);
        } else if (extend) {
            slideControl(-1.0);
        } else {
            slideControl(0.0);
        }
    }

    public void slideControl(double power) {
        double clampedPower;

        if ((power < 0 && leftSlide.getCurrentPosition() > -2140) || (power > 0 && leftSlide.getCurrentPosition() < 0)) {
            clampedPower = power;
        } else {
            clampedPower = 0.0;
        }
        
        double shiftedPower = clampedPower + 0.2; // some contractive power always necessary

        slidePower = Math.min(Math.max(shiftedPower, -1.0), 1.0); // clamp between -1 to 1
    }

    public void intakeControl(Gamepad gp) {
        if (gp.a) {
            intakeControl(1.0);
            spinnerControl(1.0);
        } else {
            intakeControl(0.0);
        }
    }

    public void intakeControl(double power) {
        intakePower = power;
    }

    public void shootControl(Gamepad gp) {
        shootControl(gp.right_trigger - gp.left_trigger);

        if (Math.abs(gp.right_trigger - gp.left_trigger) > 0.3) {
            spinnerControl(gp.right_trigger - gp.left_trigger);
        }
    }

    public void shootControl(double power) {
        shootPower = power;
    }

    public void spinnerControl(double power) {
        spinnerPower = power;
    }

    public void setLock(Gamepad gp) {
        if (gp.y) {
            if (lockDebounce.milliseconds() < 500) return;

            lockDebounce.reset();

            locked = !locked;
        }
    }

    public void driveTo(int pos) {
        if (pos == 0) return;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;

        while ((Math.abs(pos - flDrive.getCurrentPosition()) + Math.abs(pos - frDrive.getCurrentPosition()) / 2) > 20) {
            // Drive
            int flDistance = pos - flDrive.getCurrentPosition();
            int frDistance = pos - frDrive.getCurrentPosition();
            int blDistance = pos - blDrive.getCurrentPosition();
            int brDistance = pos - brDrive.getCurrentPosition();

            flDrivePower = (double) flDistance / (double) Math.abs(pos);
            blDrivePower = (double) blDistance / (double) Math.abs(pos);
            frDrivePower = (double) frDistance / (double) Math.abs(pos);
            brDrivePower = (double) brDistance / (double) Math.abs(pos);

            if (Math.abs(pos - flDrive.getCurrentPosition()) > 150 && Math.abs(pos - frDrive.getCurrentPosition()) > 150) {
                flDrivePower = Math.signum(flDrivePower) * 0.9;
                frDrivePower = Math.signum(frDrivePower) * 0.9;
                blDrivePower = Math.signum(blDrivePower) * 0.9;
                brDrivePower = Math.signum(brDrivePower) * 0.9;
            } else {
                flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.1);
                frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.1);
                blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.1);
                brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.1);
            }

            // Slowdown
            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveTo(int pos, double slowdown) {
        if (pos == 0) return;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;

        while (((Math.abs(pos - flDrive.getCurrentPosition()) + Math.abs(pos - frDrive.getCurrentPosition())) / 2) > 20) {
            // Drive
            int flDistance = pos - flDrive.getCurrentPosition();
            int frDistance = pos - frDrive.getCurrentPosition();
            int blDistance = pos - blDrive.getCurrentPosition();
            int brDistance = pos - brDrive.getCurrentPosition();

            flDrivePower = (double) flDistance / (double) Math.abs(pos);
            blDrivePower = (double) blDistance / (double) Math.abs(pos);
            frDrivePower = (double) frDistance / (double) Math.abs(pos);
            brDrivePower = (double) brDistance / (double) Math.abs(pos);

            flDrivePower = (flDrivePower / slowdown) + (Math.signum(flDrivePower) * 0.1);
            frDrivePower = (frDrivePower / slowdown) + (Math.signum(frDrivePower) * 0.1);
            blDrivePower = (blDrivePower / slowdown) + (Math.signum(blDrivePower) * 0.1);
            brDrivePower = (brDrivePower / slowdown) + (Math.signum(brDrivePower) * 0.1);

            // Slowdown
            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeTo(int pos) {
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

        while (Math.abs((Math.abs(flBrPos - flDrive.getCurrentPosition()) + Math.abs(frBlPos - frDrive.getCurrentPosition())) / 2) > 20) {
            int flDistance = flBrPos - flDrive.getCurrentPosition();
            int frDistance = frBlPos - frDrive.getCurrentPosition();
            int blDistance = frBlPos - blDrive.getCurrentPosition();
            int brDistance = flBrPos - brDrive.getCurrentPosition();

            flDrivePower = (double) flDistance / (double) Math.abs(flBrPos);
            blDrivePower = (double) blDistance / (double) Math.abs(frBlPos);
            frDrivePower = (double) frDistance / (double) Math.abs(frBlPos);
            brDrivePower = (double) brDistance / (double) Math.abs(flBrPos);

            if (Math.abs(flBrPos - brDrive.getCurrentPosition()) > 100 || Math.abs(flBrPos - flDrive.getCurrentPosition()) > 100) {
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

            // Slowdown
            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveAndStrafe(int drivePos, int strafePos) {
        double targetAngle = 0;

        double currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double errorAngle = targetAngle - currentAngle;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;

        int flBrPos = (int) (1.5 * (double) strafePos + (double) drivePos);
        int frBlPos = (int) (-1.5 * (double) strafePos + (double) drivePos);

        while (Math.abs((Math.abs(flBrPos - flDrive.getCurrentPosition()) + Math.abs(frBlPos - frDrive.getCurrentPosition())) / 2) > 15) {
            int flDistance = flBrPos - flDrive.getCurrentPosition();
            int frDistance = frBlPos - frDrive.getCurrentPosition();
            int blDistance = frBlPos - blDrive.getCurrentPosition();
            int brDistance = flBrPos - brDrive.getCurrentPosition();

            flDrivePower = (double) flDistance / (double) Math.abs(flBrPos);
            blDrivePower = (double) blDistance / (double) Math.abs(frBlPos);
            frDrivePower = (double) frDistance / (double) Math.abs(frBlPos);
            brDrivePower = (double) brDistance / (double) Math.abs(flBrPos);

            flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.15);
            frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.15);
            blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.15);
            brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.15);

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

            // Slowdown
            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnTo(double target) {
        this.imu.resetYaw();

        if (target == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - currentPosition;

        double kp = 1;

        final int DELAY = 50;

        while (Math.abs(error) > 1) {
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

            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.imu.resetYaw();
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        flDrive.setMode(mode);
        frDrive.setMode(mode);
        blDrive.setMode(mode);
        brDrive.setMode(mode);
    }

    public void resetGyro() {
        this.imu.resetYaw();
    }

    public void brake() {
        flDrivePower = 0.0;
        blDrivePower = 0.0;
        frDrivePower = 0.0;
        brDrivePower = 0.0;

        flDrive.setPower(0.0);
        blDrive.setPower(0.0);
        frDrive.setPower(0.0);
        brDrive.setPower(0.0);
    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException _ignored) {}
    }


    public double distToVelocity(double theta) {
        //Given theta (The angle at which the artifact shoots at) return the velocity needed to reach the basket
        double x = odometry.getPosX(DistanceUnit.MM);
        double y = odometry.getPosY(DistanceUnit.MM);
        double dist = Math.sqrt(x * x + y * y);
        double secant = 1 / Math.cos(Math.toRadians(theta));
        double numerator = -9.8 * dist * dist;
        double denominator = 2 * (.9845 - dist * Math.tan(Math.toRadians(theta)));
        return secant * Math.sqrt(numerator / denominator);
    }

    public enum DriveMode {
        NORMAL,
        GYRO
    }
}
