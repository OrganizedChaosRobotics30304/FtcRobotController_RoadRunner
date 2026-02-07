package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class RedAllianceSimpleLeave extends LinearOpMode {

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

@Override
  public void runOpMode(){
    Pose2d initialPose = new Pose2d(-55, 48, Math.toRadians(180));

    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

    TrajectoryActionBuilder moveToLeave = drive.actionBuilder(initialPose)
            .lineToX(-35)
            .waitSeconds(0.5)
            .turnTo(Math.toRadians(111));

    Action actionMoveToLeave = moveToLeave.build();

    waitForStart();

    Actions.runBlocking(
            new SequentialAction(
                    actionMoveToLeave
            )
    );

}
}
