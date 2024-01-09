package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Blue Far V0", group = "RoadRunner 1.0")
public class BlueFarV0 extends LinearOpMode {

    double HALF_ROBO_LEN = 9;

    // Start position blue far
    Pose2d BLUE_FAR_START_POSE = new Pose2d(-36, 72-HALF_ROBO_LEN, -Math.PI/2.0);
    Pose2d BLUE_CENTER_PARK = new Pose2d(56, 12 , 0);


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_FAR_START_POSE);

        Action TrajectoryBlueFarToPark = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-36, 12))
                .turn(Math.PI/2)
                .strafeTo(BLUE_CENTER_PARK.position)
                .build();

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        TrajectoryBlueFarToPark,
                        (telemetryPacket) -> {
                            telemetry.addData("x", drive.pose.position.x);
                            telemetry.addData("y", drive.pose.position.y);
                            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                            telemetry.update();
                            return false; // Returning true causes the action to run again, returning false causes it to cease
                        }
                )
        );

    }

}