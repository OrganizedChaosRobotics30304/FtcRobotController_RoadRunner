package org.firstinspires.ftc.teamcode.autonomous;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class BlueAllianceAuto9Piece extends LinearOpMode {

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

    private DcMotorEx shooterLeft, shooterRight;

    public ShooterClass(HardwareMap hardwareMap) {

        shooterLeft = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        shooterRight = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");

        shooterLeft.setDirection(DcMotorEx.Direction.REVERSE);

        shooterLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public Action runShooterPreload(double seconds) {

        return new TimedAction(
                () -> {
                    shooterLeft.setVelocity(1925);
                    shooterRight.setVelocity(1925);
                },
                () -> {

                    shooterLeft.setVelocity(0.0);
                    shooterRight.setVelocity(0.0);
                },
                seconds);

    }

    public Action runShooterPickUps(double seconds) {

        return new TimedAction(
                () -> {
                    shooterLeft.setVelocity(1910);
                    shooterRight.setVelocity(1910);
                },
                () -> {

                    shooterLeft.setVelocity(0.0);
                    shooterRight.setVelocity(0.0);
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
    Pose2d initialPose = new Pose2d(63, -16, Math.toRadians(0));

    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
    IntakeClass intake = new IntakeClass(hardwareMap);
    ShooterClass shooter = new ShooterClass(hardwareMap);
    PassthroughClass passthrough = new PassthroughClass(hardwareMap);

    TrajectoryActionBuilder moveToShootPreload = drive.actionBuilder(initialPose)
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(
                     new Pose2d(55, -12, Math.toRadians(15)),Math.PI / 2);//y-12

    Action moveToShootPreloadAction = moveToShootPreload.build();

    TrajectoryActionBuilder moveToAlignFirstRow = moveToShootPreload.endTrajectory().fresh()
            .setTangent(Math.toRadians(180))
            .lineToX(32)
            .turnTo(Math.toRadians(278));
            //.lineToXLinearHeading(26,Math.toRadians(278));
            //.lineToXSplineHeading(24,Math.toRadians(270));

    Action moveToAlignFirstRowAction = moveToAlignFirstRow.build();

    TrajectoryActionBuilder moveToIntakeFirstRow = moveToAlignFirstRow.endTrajectory().fresh()
            .setTangent(Math.toRadians(270))
            .lineToY(-59);

    Action moveToIntakeFirstRowAction = moveToIntakeFirstRow.build();

    TrajectoryActionBuilder moveToShootFirstRow = moveToIntakeFirstRow.endTrajectory().fresh()
            .setTangent(Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(55, -9, Math.toRadians(13)),Math.PI / 2);//y-9

    Action moveToShootFirstRowAction = moveToShootFirstRow.build();

    TrajectoryActionBuilder moveToAlignSecondRow = moveToShootFirstRow.endTrajectory().fresh()
            .setTangent(180)
            .turnTo(Math.toRadians(0))
            .lineToX(6)
            .turnTo(Math.toRadians(278));
            //.lineToXLinearHeading(8, Math.toRadians(278));
            //.turnTo(Math.toRadians(278));

    Action moveToAlignSecondRowAction = moveToAlignSecondRow.build();

    TrajectoryActionBuilder moveToIntakeSecondRow = moveToAlignSecondRow.endTrajectory().fresh()
            .setTangent(Math.toRadians(270))
            .lineToY(-56);

    Action moveToIntakeSecondRowAction = moveToIntakeSecondRow.build();

    TrajectoryActionBuilder moveToShootSecondRow = moveToIntakeSecondRow.endTrajectory().fresh()
            .setTangent(Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(55, -3, Math.toRadians(13)),Math.PI / 2);

    Action moveToShootSecondRowAction = moveToShootSecondRow.build();

    TrajectoryActionBuilder moveToLeave = moveToShootSecondRow.endTrajectory().fresh()
            //.setTangent(Math.toRadians(180))
            .setTangent(Math.toRadians(270))
            //.lineToYSplineHeading (-36, Math.toRadians(279.5));
            .lineToYLinearHeading (-13, Math.toRadians(285));

    Action moveToLeaveAction = moveToLeave.build();

    Action shooterAndPassthroughPreLoad = new ParallelAction(
            shooter.runShooterPreload(2.6),//3.0
            new SequentialAction(
                    new SleepAction(0.7),//1.0
                    passthrough.runPassthrough(1.9)),//2.0
            new SequentialAction(
            new SleepAction(0.7),//1.0
            intake.runIntake(1.9)//2.0
    )
    );

    Action shooterAndPassthroughFirstRow = new ParallelAction(
            shooter.runShooterPickUps(2.6),//3.0
            new SequentialAction(
                    new SleepAction(0.7),//1.0
                    passthrough.runPassthrough(1.9)//2.0
            ),
            new SequentialAction(
                   new SleepAction(0.7),//1.0
                   intake.runIntake(1.9)//2.0
            )
    );

    Action shooterAndPassthroughSecondRow = new ParallelAction(
            shooter.runShooterPickUps(2.6),//3.0
            new SequentialAction(
                    new SleepAction(0.7),//1.0
                    passthrough.runPassthrough(1.9)//2.0
            ),
            new SequentialAction(
                    new SleepAction(0.7),//1.0
                    intake.runIntake(1.9)//2.0
            )
    );

     Action intakeAndMovementFirstRow = new ParallelAction(
            moveToIntakeFirstRowAction,
            new SequentialAction(
                    //new SleepAction(0.3),
                    //new SleepAction(0.2),
                    intake.runIntake(2.0)

            ),
            new SequentialAction(
                    new SleepAction(0.86),
                    passthrough.runPassthrough(0.47)
            )
    );

    Action intakeAndMovementSecondRow = new ParallelAction(
            moveToIntakeSecondRowAction,
            new SequentialAction(
                    //new SleepAction(0.3),
                    //new SleepAction(0.2),
                    intake.runIntake(2.0)

            ),
            new SequentialAction(
                    new SleepAction(0.86),
                    passthrough.runPassthrough(0.47)
            )
    );

    waitForStart();

    Actions.runBlocking(
            new SequentialAction(
                    moveToShootPreloadAction,
                    shooterAndPassthroughPreLoad,
                    moveToAlignFirstRowAction,
                    intakeAndMovementFirstRow,
                    moveToShootFirstRowAction,
                    shooterAndPassthroughFirstRow,
                    moveToAlignSecondRowAction,
                    intakeAndMovementSecondRow,
                    moveToShootSecondRowAction,
                    shooterAndPassthroughSecondRow,
                    moveToLeaveAction

            )
    );

}
}
