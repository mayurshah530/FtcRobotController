package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Red Near V2", group = "RoadRunner 1.0")
@Disabled
public class RedNearV2 extends LinearOpMode {

    public static double HALF_ROBO_LEN = 9;
    public static double RED_NEAR_LEFT_SPIKE_X = 10;
    public static double RED_NEAR_LEFT_SPIKE_Y = -30;


    // Start position red far

    // START POSITIONS
    Pose2d BLUE_NEAR_START_POSE = new Pose2d(12, 72-HALF_ROBO_LEN, -Math.PI/2.0);
    Pose2d RED_NEAR_START_POSE = new Pose2d(12, -(72-HALF_ROBO_LEN), Math.PI/2.0);

    Pose2d RED_FAR_START_POSE = new Pose2d(-36, -(72-HALF_ROBO_LEN), Math.PI/2.0);


    // PARK POSITIONS
    Pose2d BLUE_LEFT_PARK = new Pose2d(58 - HALF_ROBO_LEN, 56, 0);
    Pose2d BLUE_LEFT_PARK_REVERSE = new Pose2d(58 - HALF_ROBO_LEN, 56, -Math.PI);
    Pose2d RED_RIGHT_PARK = new Pose2d(58 - HALF_ROBO_LEN, -60, 0);
    Pose2d RED_CENTER_PARK = new Pose2d(56, -12 , 0);

    // SPIKE Locations
    Pose2d RED_NEAR_CENTER_SPIKE = new Pose2d(12, -(24.5+HALF_ROBO_LEN), Math.PI/2.0);
    public static double RED_NEAR_RIGHT_SPIKE_X = 23.5-HALF_ROBO_LEN;
    public static double RED_NEAR_RIGHT_SPIKE_Y = -30;
    public static double RED_NEAR_RIGHT_SPIKE_BACK_X = 6;

    Pose2d RED_NEAR_RIGHT_SPIKE = new Pose2d(RED_NEAR_RIGHT_SPIKE_X, RED_NEAR_RIGHT_SPIKE_Y, 0);
    Pose2d RED_NEAR_LEFT_SPIKE = new Pose2d(RED_NEAR_LEFT_SPIKE_X, RED_NEAR_LEFT_SPIKE_Y, -Math.PI);

    Pose2d RED_FAR_CENTER_SPIKE = new Pose2d(-36, -(24.5+HALF_ROBO_LEN), -Math.PI/2.0);
    Pose2d RED_FAR_RIGHT_SPIKE = new Pose2d(23.5-HALF_ROBO_LEN, -(30), 0);
    Pose2d RED_FAR_LEFT_SPIKE = new Pose2d(9, -(19.5)-HALF_ROBO_LEN, -(30));


    Pose2d BLUE_NEAR_CENTER_SPIKE = new Pose2d(12, 24.5+HALF_ROBO_LEN, -Math.PI/2.0);
    Pose2d BLUE_NEAR_RIGHT_SPIKE = new Pose2d(0.5+HALF_ROBO_LEN, 30+HALF_ROBO_LEN, -Math.PI);

    // TAG locations
    Pose2d BLUE_BOARD_CENTER_TAG = new Pose2d(58 - HALF_ROBO_LEN, 36, 0);

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_NEAR_START_POSE);



        Action v3RedNearGeCenterPark = drive.actionBuilder(RED_NEAR_START_POSE)
                .strafeTo(RED_NEAR_CENTER_SPIKE.position)
                .waitSeconds(1.0)
                .strafeTo(new Vector2d(12, -40))
                .turn(Math.toRadians(-120))
                .strafeTo(RED_RIGHT_PARK.position)
                .build();

        Action v3RedNearGeLeftPark = drive.actionBuilder(RED_NEAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(RED_NEAR_LEFT_SPIKE.position.x + 5, RED_NEAR_LEFT_SPIKE.position.y), Math.toRadians(180))
                .strafeTo(new Vector2d(RED_NEAR_LEFT_SPIKE.position.x -5, RED_NEAR_LEFT_SPIKE_Y))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_NEAR_LEFT_SPIKE.position.x + 3, RED_NEAR_LEFT_SPIKE.position.y))
                .strafeToLinearHeading(RED_RIGHT_PARK.position, Math.toRadians(0)) // Move to parking position
                .build();

        Action v3RedNearGeRightPark = drive.actionBuilder(RED_NEAR_START_POSE)
                .strafeToLinearHeading(RED_NEAR_RIGHT_SPIKE.position, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_NEAR_RIGHT_SPIKE.position.x - RED_NEAR_RIGHT_SPIKE_BACK_X, RED_NEAR_RIGHT_SPIKE.position.y))
                .strafeTo(new Vector2d(12, -58))
                .strafeTo(RED_RIGHT_PARK.position) // Move to parking position
                .build();

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        v3RedNearGeRightPark, // Example of a drive action
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