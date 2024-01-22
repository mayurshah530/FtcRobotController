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
@Autonomous(name = "Blue Near V3", group = "RoadRunner 1.0")
public class BlueNearV3 extends LinearOpMode {

    double HALF_ROBO_LEN = 9;

    // START POSITIONS
    Pose2d BLUE_NEAR_START_POSE = new Pose2d(12, 72-HALF_ROBO_LEN, -Math.PI/2.0);


    // PARK POSITIONS
    Pose2d BLUE_LEFT_PARK = new Pose2d(58 - HALF_ROBO_LEN, 56, 0);

    // SPIKE POSITIONS
    Pose2d BLUE_NEAR_CENTER_SPIKE = new Pose2d(12, 24.5+HALF_ROBO_LEN, -Math.PI/2.0);
    Pose2d BLUE_NEAR_LEFT_SPIKE = new Pose2d(23.5-HALF_ROBO_LEN, (30), 0);
    Pose2d BLUE_NEAR_RIGHT_SPIKE = new Pose2d(12, 30,   Math.PI);



    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ScoringElementLocation selectedSide = ScoringElementLocation.UNKNOWN;


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_NEAR_START_POSE);


        initVisionPortal();
        visionProcessor.SetAlliance(Alliance.BLUE);


        Action v3BlueNearGeCenterPark = drive.actionBuilder(BLUE_NEAR_START_POSE)
                .strafeTo(BLUE_NEAR_CENTER_SPIKE.position)
                .waitSeconds(1.0)
                .strafeTo(new Vector2d(12, 40))
                .turn(Math.toRadians(120))
                .splineToLinearHeading(BLUE_LEFT_PARK, 0)
                .build();


        Action v3BlueNearGeLeftPark = drive.actionBuilder(BLUE_NEAR_START_POSE)
                .strafeToLinearHeading(BLUE_NEAR_LEFT_SPIKE.position, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_NEAR_LEFT_SPIKE.position.x - 2, BLUE_NEAR_LEFT_SPIKE.position.y))
                .strafeTo(new Vector2d(12, 58))
                .strafeTo(BLUE_LEFT_PARK.position) // Move to parking position
                .build();


        Action v3BlueNearGeRightPark = drive.actionBuilder(BLUE_NEAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(BLUE_NEAR_RIGHT_SPIKE.position.x + 5, BLUE_NEAR_RIGHT_SPIKE.position.y), Math.PI)
                .strafeTo(new Vector2d(BLUE_NEAR_RIGHT_SPIKE.position.x -2, BLUE_NEAR_RIGHT_SPIKE.position.y))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_NEAR_RIGHT_SPIKE.position.x + 0, BLUE_NEAR_RIGHT_SPIKE.position.y))
                .strafeToLinearHeading(BLUE_LEFT_PARK.position, Math.toRadians(0)) // Move to parking position
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


        Action trajectoryToRun = v3BlueNearGeCenterPark;
        switch (selectedSide){
            case LEFT:
                trajectoryToRun = v3BlueNearGeLeftPark;
                break;
            case CENTER:
                trajectoryToRun = v3BlueNearGeCenterPark;
                break;
            case RIGHT:
                trajectoryToRun = v3BlueNearGeRightPark;
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
