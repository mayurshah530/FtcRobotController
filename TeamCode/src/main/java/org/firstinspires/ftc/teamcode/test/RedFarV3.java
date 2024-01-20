package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.ScoringElementLocation;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@Autonomous(name = "Red Far V3", group = "RoadRunner 1.0")
public class RedFarV3 extends LinearOpMode {

    double HALF_ROBO_LEN = 9;

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
    Pose2d RED_NEAR_CENTER_SPIKE = new Pose2d(12, -(24.5+HALF_ROBO_LEN), -Math.PI/2.0);
    Pose2d RED_NEAR_RIGHT_SPIKE = new Pose2d(23.5-HALF_ROBO_LEN, -(30), 0);
    Pose2d RED_NEAR_LEFT_SPIKE = new Pose2d(9, -(19.5)-HALF_ROBO_LEN, -(30));

    Pose2d RED_FAR_CENTER_SPIKE = new Pose2d(-36, -(24.5+HALF_ROBO_LEN), -Math.PI/2.0);
    Pose2d RED_FAR_LEFT_SPIKE = new Pose2d(-(46 -HALF_ROBO_LEN), -30, Math.PI);
    Pose2d RED_FAR_RIGHT_SPIKE = new Pose2d(-(23.5+HALF_ROBO_LEN), -(30), 0);


    Pose2d BLUE_NEAR_CENTER_SPIKE = new Pose2d(12, 24.5+HALF_ROBO_LEN, -Math.PI/2.0);
    Pose2d BLUE_NEAR_RIGHT_SPIKE = new Pose2d(0.5+HALF_ROBO_LEN, 30+HALF_ROBO_LEN, -Math.PI);

    // TAG locations
    Pose2d BLUE_BOARD_CENTER_TAG = new Pose2d(58 - HALF_ROBO_LEN, 36, 0);

    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ScoringElementLocation selectedSide = ScoringElementLocation.UNKNOWN;


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_FAR_START_POSE);


        initVisionPortal();
        visionProcessor.SetAlliance(Alliance.RED);


        Action v3RedFarGeCenterPark = drive.actionBuilder(RED_FAR_START_POSE)
                .strafeTo(RED_FAR_CENTER_SPIKE.position)
                .waitSeconds(1)
                .strafeTo(new Vector2d(RED_FAR_CENTER_SPIKE.position.x, RED_FAR_CENTER_SPIKE.position.y - 10)) // come back
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-52, RED_FAR_CENTER_SPIKE.position.y - 10)) // slide left
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-52, -12)) // move fwd
                .turn(-Math.PI/2)
                .strafeTo(RED_CENTER_PARK.position) // park
                .build();


        Action v3RedFarGeLeftPark = drive.actionBuilder(RED_FAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(RED_FAR_LEFT_SPIKE.position.x-2, RED_FAR_LEFT_SPIKE.position.y), Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_FAR_LEFT_SPIKE.position.x + 0, RED_FAR_LEFT_SPIKE.position.y))
                .strafeTo(new Vector2d(RED_FAR_LEFT_SPIKE.position.x + 0, -12))
                .turn(Math.PI+1e-6)
                .strafeTo(RED_CENTER_PARK.position) // park
                .build();

        Action v3RedfarGeRightPark = drive.actionBuilder(RED_FAR_START_POSE)
                .splineToLinearHeading(RED_FAR_RIGHT_SPIKE, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_FAR_RIGHT_SPIKE.position.x - 5, RED_FAR_RIGHT_SPIKE.position.y))
                .strafeTo(new Vector2d(RED_FAR_RIGHT_SPIKE.position.x - 5, -8))
                .strafeTo(RED_CENTER_PARK.position) // park
                .build();


        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        selectedSide = visionProcessor.getSelection();
        visionPortal.stopStreaming();
        visionPortal.setProcessorEnabled(visionProcessor, false);
        telemetry.addData("selectedSide: ", selectedSide.toString());
        telemetry.addData("Identified", visionProcessor.getTelemetry());

        telemetry.update();


        Action trajectoryToRun = v3RedFarGeCenterPark;
        switch (selectedSide){
            case LEFT:
                trajectoryToRun = v3RedFarGeLeftPark;
                break;
            case CENTER:
                trajectoryToRun = v3RedFarGeCenterPark;
                break;
            case RIGHT:
                trajectoryToRun = v3RedfarGeRightPark;
                break;
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryToRun
                )
        );

    }

    private void initVisionPortal() {
        visionProcessor = new FirstVisionProcessor();
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(visionProcessor, aprilTag)
                .build();
        // At the beginning, vision processor on and aprilTag processor off
        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(visionProcessor, true);
    }


}