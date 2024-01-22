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
@Autonomous(name = "Blue Far V3", group = "RoadRunner 1.0")
public class BlueFarV3 extends LinearOpMode {

    double HALF_ROBO_LEN = 9;

    // START POSITIONS
    Pose2d BLUE_FAR_START_POSE = new Pose2d(-36, (72-HALF_ROBO_LEN), -Math.PI/2.0);
    // PARK POSITIONS
    Pose2d BLUE_CENTER_PARK = new Pose2d(56, 12 , 0);
    // SPIKE POSITIONS
    Pose2d BLUE_FAR_CENTER_SPIKE = new Pose2d(-36, (24.5+HALF_ROBO_LEN), Math.PI/2.0);
    Pose2d BLUE_FAR_RIGHT_SPIKE = new Pose2d(-(46 -HALF_ROBO_LEN), 30, -Math.PI);
    Pose2d BLUE_FAR_LEFT_SPIKE = new Pose2d(-(23.5+HALF_ROBO_LEN), (30), 0);

    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ScoringElementLocation selectedSide = ScoringElementLocation.UNKNOWN;


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_FAR_START_POSE);


        initVisionPortal();
        visionProcessor.SetAlliance(Alliance.BLUE);


        Action v3BlueFarGeCenterPark = drive.actionBuilder(BLUE_FAR_START_POSE)
                .strafeTo(BLUE_FAR_CENTER_SPIKE.position)
                .waitSeconds(1)
                .strafeTo(new Vector2d(BLUE_FAR_CENTER_SPIKE.position.x, BLUE_FAR_CENTER_SPIKE.position.y + 10)) // come back
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-52, BLUE_FAR_CENTER_SPIKE.position.y + 10)) // slide left
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-52, 10)) // move fwd
                .turn(Math.PI/2)
                .strafeTo(BLUE_CENTER_PARK.position) // park
                .build();


        Action v3BlueFarGeLeftPark = drive.actionBuilder(BLUE_FAR_START_POSE)
                .splineToLinearHeading(BLUE_FAR_LEFT_SPIKE, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_FAR_LEFT_SPIKE.position.x - 5, BLUE_FAR_LEFT_SPIKE.position.y))
                .strafeTo(new Vector2d(BLUE_FAR_LEFT_SPIKE.position.x - 5, 10))
                .strafeTo(BLUE_CENTER_PARK.position) // park
                .build();

        Action v3BluefarGeRightPark = drive.actionBuilder(BLUE_FAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(BLUE_FAR_RIGHT_SPIKE.position.x, BLUE_FAR_RIGHT_SPIKE.position.y), -Math.PI)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_FAR_RIGHT_SPIKE.position.x + 2, BLUE_FAR_RIGHT_SPIKE.position.y))
                .strafeTo(new Vector2d(BLUE_FAR_RIGHT_SPIKE.position.x + 2, 10))
                .turn(-(Math.PI+1e-6))
                .strafeTo(BLUE_CENTER_PARK.position) // park
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


        Action trajectoryToRun = v3BlueFarGeCenterPark;
        switch (selectedSide){
            case LEFT:
                trajectoryToRun = v3BlueFarGeLeftPark;
                break;
            case CENTER:
                trajectoryToRun = v3BlueFarGeCenterPark;
                break;
            case RIGHT:
                trajectoryToRun = v3BluefarGeRightPark;
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
