package org.firstinspires.ftc.teamcode.test;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.ScoringElementLocation;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@Autonomous(name = "Blue Near V2", group = "RoadRunner 1.0")
public class BlueNearV2 extends LinearOpMode {

    double HALF_ROBO_LEN = 9;
    double HALF_ROBO_WIDTH = 9;


    // Blue near start pose
    Pose2d BLUE_NEAR_START_POSE = new Pose2d(12, 72-HALF_ROBO_LEN, -Math.PI/2.0);
    Pose2d BLUE_LEFT_PARK = new Pose2d(58 - HALF_ROBO_LEN, 56, 0);
    Pose2d BLUE_NEAR_CENTER_SPIKE = new Pose2d(12, 24.5+HALF_ROBO_LEN, -Math.PI/2.0);
    Pose2d BLUE_BOARD_CENTER_TAG = new Pose2d(58 - HALF_ROBO_LEN, 36, 0);

    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ScoringElementLocation selectedSide = ScoringElementLocation.UNKNOWN;

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_NEAR_START_POSE);

        initVisionPortal();
        visionProcessor.SetAlliance(Alliance.BLUE);

        // 1. Start position: blue near; Game Element location: Center.
        // Declare trajectory
        Action TrajectoryBlueNearGeCenter = drive.actionBuilder(drive.pose)
                .lineToY(BLUE_NEAR_CENTER_SPIKE.position.y)
                .waitSeconds(1)
                .setReversed(true)
                .lineToY(BLUE_NEAR_CENTER_SPIKE.position.y + 18)
                .setReversed(false)
                .splineToLinearHeading(BLUE_BOARD_CENTER_TAG, 0)
                .waitSeconds(2)
                .strafeTo(BLUE_LEFT_PARK.position)
                .turn(Math.toRadians(-135)) // Optional: Turn so that you are ready to grab pixels in manual mode.
                .waitSeconds(2)
                .build();
        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        selectedSide = visionProcessor.getSelection();
        visionPortal.stopStreaming();
        visionPortal.setProcessorEnabled(visionProcessor, false);
        telemetry.addData("selectedSide: ", selectedSide.toString());
        telemetry.update();

        if (isStopRequested()) return;

        Action trajectoryToRun = TrajectoryBlueNearGeCenter;
        switch (selectedSide){
            case LEFT:
                // TODO: Define left trajectory
                trajectoryToRun = TrajectoryBlueNearGeCenter;
                break;
            case CENTER:
                trajectoryToRun = TrajectoryBlueNearGeCenter;
                break;
            case RIGHT:
                // TODO: Define left trajectory
                trajectoryToRun = TrajectoryBlueNearGeCenter;
                break;
        }

        // General plan
        // Identify game element location (left/center/right)
        // Go place pixel
        // [later] Go towards scoring position
        // [later] Enable AprilTag processor to find absolute location
        // [later] Move to precise scoring position
        // [later] Score
        // Park

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryToRun, 
                        // This action uses a Lambda expression to reduce complexity
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