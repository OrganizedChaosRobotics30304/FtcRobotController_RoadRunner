package org.firstinspires.ftc.teamcode.autonomous.disabled;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
//@Autonomous
@Disabled
public class RedAllianceAuto6Piece extends LinearOpMode {

public class TimedAction implements Action {

    private final Runnable start;
    private final Runnable end;
    private final double duration;
    private double startTime = -1.0;

    public TimedAction (Runnable start, Runnable end, double duration){

        this.start = start;
        this.end = end;
        this.duration = duration;

    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (startTime <0) {
            start.run();
            startTime = System.currentTimeMillis();
        }

        double elapsed = (System.currentTimeMillis() - startTime) / 1000.0;

        packet.put("elapsedTime", elapsed);

        if (elapsed < duration) {
            return true;
        }
        else {
            end.run();
            return false;
        }
    }
}

public class ShooterClass {

    private DcMotor shooterLeft, shooterRight;

    public ShooterClass(HardwareMap hardwareMap) {

        shooterLeft = hardwareMap.get(DcMotor.class, "leftShooterMotor");
        shooterRight = hardwareMap.get(DcMotor.class, "rightShooterMotor");

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Action runShooter(double seconds){

        return new TimedAction(
        ()-> {
            shooterLeft.setPower(0.82);
            shooterRight.setPower(0.82);
        },
                ()-> {

            shooterLeft.setPower(0.0);
            shooterRight.setPower(0.0);
                },
        seconds);

    }
}
public class PassthroughClass{

    private DcMotor passthroughRight, passthroughLeft;

    public PassthroughClass(HardwareMap hardwareMap){

        passthroughLeft = hardwareMap.get(DcMotor.class, "leftPassthroughMotor");
        passthroughRight = hardwareMap.get(DcMotor.class, "rightPassthroughMotor");

        passthroughLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        passthroughRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        passthroughLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

   public Action runPassthrough(double seconds) {

        return new TimedAction(
                ()->{
                    passthroughRight.setPower(1.0);
                    passthroughLeft.setPower(1.0);
                },
                ()->{
                    passthroughRight.setPower(0.0);
                    passthroughLeft.setPower(0.0);
                },
                seconds
        );
   }
}

public class IntakeClass{
    private CRServo intakeRight, intakeLeft;

    public IntakeClass(HardwareMap hardwareMap){
        intakeRight = hardwareMap.get(CRServo.class, "rightIntakeServo");
        intakeLeft = hardwareMap.get(CRServo.class, "leftIntakeServo");

        intakeLeft.setDirection(CRServo.Direction.REVERSE);
    }

    public Action runIntake(double seconds) {

        return new TimedAction(
                ()->{
                    intakeLeft.setPower(-1.0);
                    intakeRight.setPower(-1.0);
                },
                ()->{
                    intakeLeft.setPower(0.0);
                    intakeRight.setPower(0.0);
                },
                seconds
        );
    }
}

@Override
  public void runOpMode(){
    Pose2d initialPose = new Pose2d(63, 16, Math.toRadians(0));

    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
    IntakeClass intake = new IntakeClass(hardwareMap);
    ShooterClass shooter = new ShooterClass(hardwareMap);
    PassthroughClass passthrough = new PassthroughClass(hardwareMap);

    TrajectoryActionBuilder moveToShootPreload = drive.actionBuilder(initialPose)
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(
                     new Pose2d(60, 12 ,Math.toRadians(345)),Math.PI / 2);

    Action moveToShootPreloadAction = moveToShootPreload.build();

    TrajectoryActionBuilder moveToAlign = moveToShootPreload.endTrajectory().fresh()
            .setTangent(Math.toRadians(180))
            .lineToX(34)
            .turnTo(Math.toRadians(82));
            //.lineToXLinearHeading(26,Math.toRadians(278));
            //.lineToXSplineHeading(24,Math.toRadians(270));

    Action moveToAlignAction = moveToAlign.build();

    TrajectoryActionBuilder moveToIntake = moveToAlign.endTrajectory().fresh()
            .setTangent(Math.toRadians(90))
            .lineToY(61);

    Action moveToIntakeAction = moveToIntake.build();

    TrajectoryActionBuilder moveToShootFinal = moveToIntake.endTrajectory().fresh()
            .setTangent(Math.toRadians(270))
            .splineToLinearHeading(new Pose2d(55, 9, Math.toRadians(344)),Math.PI / 2);

    Action moveToShootFinalAction = moveToShootFinal.build();

    TrajectoryActionBuilder moveToLeave = moveToShootFinal.endTrajectory().fresh()
            //.setTangent(Math.toRadians(180))
            .setTangent(Math.toRadians(90))
            //.lineToYSplineHeading (-36, Math.toRadians(279.5));
            .lineToYSplineHeading (28, Math.toRadians(83));

    Action moveToLeaveAction = moveToLeave.build();

    Action shooterAndPassthrough = new ParallelAction(
            shooter.runShooter(3.0),//3.0
            new SequentialAction(
                    new SleepAction(1.0),//1.0
                    passthrough.runPassthrough(2.0)),//3.0
            new SequentialAction(
            new SleepAction(1.0),//1.0
            intake.runIntake(2.0)//3.0
    )
    );

    Action shooterAndPassthroughAgain = new ParallelAction(
            shooter.runShooter(3.0),//3.0
            new SequentialAction(
                    new SleepAction(1.0),//1.0
                    passthrough.runPassthrough(2.0)//2.5
            ),
            new SequentialAction(
                   new SleepAction(1.0),//1.0
                   intake.runIntake(2.0)//2.5
            )
    );

     Action intakeAndMovement = new ParallelAction(
            moveToIntakeAction,
            new SequentialAction(
                    //new SleepAction(0.3),
                    //new SleepAction(0.2),
                    intake.runIntake(2.0)

            ),
            new SequentialAction(
                    new SleepAction(0.86),
                    passthrough.runPassthrough(0.467)
            )
    );


    waitForStart();

    Actions.runBlocking(
            new SequentialAction(
                    moveToShootPreloadAction,
                    shooterAndPassthrough,
                    moveToAlignAction,
                    intakeAndMovement,
                    moveToShootFinalAction,
                    shooterAndPassthroughAgain,
                    moveToLeaveAction

            )
    );

}
}
