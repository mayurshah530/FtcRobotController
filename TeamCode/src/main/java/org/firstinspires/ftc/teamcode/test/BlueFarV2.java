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
@Autonomous(name = "Blue Far V2", group = "RoadRunner 1.0")
public class BlueFarV2 extends LinearOpMode {

    double HALF_ROBO_LEN = 9;
    double HALF_ROBO_WIDTH = 9;

    // Start position blue far; GE: Center
    Pose2d BLUE_FAR_START_POSE = new Pose2d(-36, 72-HALF_ROBO_LEN, -Math.PI/2.0);
    Pose2d BLUE_FAR_CENTER_SPIKE = new Pose2d(-36, 24.5+HALF_ROBO_LEN, -Math.PI/2.0);
    Pose2d BLUE_CENTER_PARK = new Pose2d(58 - HALF_ROBO_LEN, 2 + HALF_ROBO_LEN , 0);
    Pose2d BLUE_BOARD_CENTER_TAG = new Pose2d(58 - HALF_ROBO_LEN, 36, 0);


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_FAR_START_POSE);



        // 1. Start position: blue far; Game Element location: Center.
        Pose2d BLUE_BOARD_CENTER_TAG = new Pose2d(58 - HALF_ROBO_LEN, 36, 0);
        

        while(!isStopRequested() && !opModeIsActive()) {

        }

        // Declare trajectory
        Action TrajectoryBlueFarGeCenter = drive.actionBuilder(drive.pose)
                .lineToY(BLUE_FAR_CENTER_SPIKE.position.y)
                .waitSeconds(1)
                .setReversed(true)
                .lineToY(BLUE_FAR_CENTER_SPIKE.position.y + 6)
                .setReversed(false)
                .strafeTo(new Vector2d(-48, BLUE_FAR_CENTER_SPIKE.position.y + 6))
                .strafeTo(new Vector2d(-48, 12))
                .turnTo(0)
                .lineToX(BLUE_BOARD_CENTER_TAG.position.x)
                .strafeTo(BLUE_BOARD_CENTER_TAG.position)
                .waitSeconds(2)
                .strafeTo(BLUE_CENTER_PARK.position)
                .turn(Math.PI +1e-6)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        TrajectoryBlueFarGeCenter, // Example of a drive action
                        // Only that this action uses a Lambda expression to reduce complexity
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