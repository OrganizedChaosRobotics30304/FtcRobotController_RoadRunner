package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class SimpleLeave extends OpMode {
    // Declares 4 drive motors and IMU for the mecanum drive
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor rightRear;
    public IMU imu;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {

        // Mapping declared motors to the correct ports on the control or expansion
        // hub based on the configuration settings, deviceName must match configuration
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontDriveMotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontDriveMotor");
        leftRear = hardwareMap.get(DcMotor.class, "leftRearDriveMotor");
        rightRear = hardwareMap.get(DcMotor.class, "rightRearDriveMotor");

        // Sets the motor to run via encoder to ensure all motors will rotate
        // at the same speed based on encoder feedback
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Sets the left motors to run the reverse direction from their command
        // Motors mounted on the left must be reversed to match motors on the right so
        // that forward is forward on both sides of the robot
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);


        // Maps the declared imu variable to the internal IMU within the REV control hub
        // deviceName imu is preset in the configuration
        imu = hardwareMap.get(IMU.class, "imu");

        // Sets the orientation of the IMU based on how the control hub is mounted on
        // the robot
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

        // Sets the direction of the robot on the field when INIT is pressed
        // This will be the orientation of the robot for drive
        imu.initialize(new IMU.Parameters(RevOrientation));

    }
    @Override
    public void start(){
        runtime.reset();
    }
    @Override
    public void loop() {
        double elapsed = runtime.seconds();

        if (elapsed < 5.0){
            rightFront.setPower(-0.5);
            leftRear.setPower(-0.5);
        }
        else {
            rightFront.setPower(0.0);
            rightRear.setPower(0.0);
            leftRear.setPower(0.0);
            rightRear.setPower(0.0);
        }
    }
}