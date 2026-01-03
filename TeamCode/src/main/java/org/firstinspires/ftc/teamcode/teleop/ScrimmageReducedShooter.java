package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp

public class ScrimmageReducedShooter extends OpMode {
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor passthroughRight;
    public DcMotor passthroughLeft;
    public DcMotor shooterRight;
    public DcMotor shooterLeft;
    public CRServo intakeLeft;
    public CRServo intakeRight;
    public IMU imu;

    @Override
    public void init() {
        //drive the bot thingy

        frontRight = hardwareMap.get(DcMotor.class,"rightFrontDriveMotor");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFrontDriveMotor");
        backLeft = hardwareMap.get(DcMotor.class, "leftRearDriveMotor");
        backRight = hardwareMap.get(DcMotor.class, "rightRearDriveMotor");

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        //shoot the ball thingy

        shooterRight = hardwareMap.get(DcMotor.class, "rightShooterMotor");
        shooterLeft = hardwareMap.get(DcMotor.class, "leftShooterMotor");

        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterLeft.setDirection(DcMotor.Direction.REVERSE);

        //move the ball thingy

        passthroughRight = hardwareMap.get(DcMotor.class,"rightPassthroughMotor");
        passthroughLeft = hardwareMap.get(DcMotor.class,"leftPassthroughMotor");

        passthroughRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        passthroughLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        passthroughLeft.setDirection(DcMotor.Direction.REVERSE);

        //bring in ball thingy

        intakeLeft = hardwareMap.get(CRServo.class, "leftIntakeServo");
        intakeRight = hardwareMap.get(CRServo.class, "rightIntakeServo");

        intakeLeft.setDirection(CRServo.Direction.REVERSE);

        //The where am i thingy

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot
                (RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }


    @Override
    public void loop() {

        //now i know where i am thingy
        if (gamepad1.a){
            imu.resetYaw();
        }

        //go to the place thingy
        double forward = gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);


        double frontLeftPower = newForward + newRight + rotate;
        double frontRightPower = newForward - newRight - rotate;
        double backRightPower = newForward + newRight - rotate;
        double backLeftPower = newForward - newRight + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        backRight.setPower(maxSpeed * (backRightPower / maxPower));

        //Slurp it up thingy
        if (gamepad2.a){
            intakeLeft.setPower(-1.0);
            intakeRight.setPower(-1.0);
        }
        else if (gamepad2.b) {
            intakeRight.setPower(1.0);
            intakeLeft.setPower(1.0);
        }
        else {
            intakeRight.setPower(0.0);
            intakeLeft.setPower(0.0);
        }

        //here take this thingy
        double passthroughForwardPower = gamepad2.left_trigger;
        double passthroughReversePower = -gamepad2.left_stick_y;

        if (gamepad2.left_trigger > 0.05){
            passthroughLeft.setPower(passthroughForwardPower/2);
            passthroughRight.setPower(passthroughForwardPower/2);
        }
        else if (gamepad2.left_stick_y > 0.05){
            passthroughLeft.setPower(passthroughReversePower/2);
            passthroughRight.setPower(passthroughReversePower/2);
        }
        else {
            passthroughLeft.setPower(0.0);
            passthroughRight.setPower(0.0);
        }

        //kwa-pow thingy
        double shooterForwardPower = gamepad2.right_trigger;
        double shooterReversePower = gamepad2.right_stick_y;

        if (gamepad2.right_trigger > 0.05) {
            shooterLeft.setPower(shooterForwardPower-0.07); // lower speed for shooter; higher decimal = less speed
            shooterRight.setPower(shooterForwardPower-0.07);
        }
        else if (gamepad2.right_stick_y > 0.05) {
            shooterLeft.setPower(-shooterReversePower/2);
            shooterRight.setPower(-shooterReversePower/2);
        }
        else{
            shooterLeft.setPower(0.0);
            shooterRight.setPower(0.0);
        }

        //tell me whats wrong stuff
        telemetry.addData("Y axis", gamepad1.left_stick_y);
        telemetry.addData("leftFront", frontLeft.getPower());
        telemetry.addData("rightFront", frontRight.getPower());
        telemetry.addData("leftRear", backLeft.getPower());
        telemetry.addData("rightRear", backRight.getPower());
        telemetry.addData("Shooter Power", shooterLeft.getPower());
        telemetry.update();


    }
}