package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.ScoringElementLocation;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.teamcode.common.Utils;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name = "Red Near V4", group = "RoadRunner 1.0")
public class RedNearV4 extends LinearOpMode {

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
    Pose2d RED_RIGHT_PARK = new Pose2d(62 - HALF_ROBO_LEN, -66, 0);
    Pose2d RED_CENTER_PARK = new Pose2d(56, -12 , 0);

    // SPIKE Locations
    Pose2d RED_NEAR_CENTER_SPIKE = new Pose2d(12, -(24.5+HALF_ROBO_LEN), Math.PI/2.0);
    public static double RED_NEAR_RIGHT_SPIKE_X = 23.5-HALF_ROBO_LEN;
    public static double RED_NEAR_RIGHT_SPIKE_Y = -25;
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

    // wait location
    Pose2d RED_BOARD_WAIT = new Pose2d(40, -36, 0);


    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ScoringElementLocation selectedSide = ScoringElementLocation.UNKNOWN;

    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_NEAR_START_POSE);
        Outtake outtake = new Outtake(hardwareMap);

        initVisionPortal();
        visionProcessor.SetAlliance(Alliance.RED);

        Action v3RedNearGeCenterScore = drive.actionBuilder(RED_NEAR_START_POSE)
                .strafeTo(RED_NEAR_CENTER_SPIKE.position)
                .waitSeconds(1.0)
                .strafeTo(new Vector2d(12, -40))
                .turn(Math.toRadians(-90))
                .strafeTo(RED_RIGHT_PARK.position)
                .build();

        Action v3RedNearGeLeftPark = drive.actionBuilder(RED_NEAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(RED_NEAR_LEFT_SPIKE.position.x + 5, RED_NEAR_LEFT_SPIKE.position.y), Math.toRadians(180))
                .strafeTo(new Vector2d(RED_NEAR_LEFT_SPIKE.position.x -5, RED_NEAR_LEFT_SPIKE_Y))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_NEAR_LEFT_SPIKE.position.x + 0, RED_NEAR_LEFT_SPIKE.position.y))
                .strafeToLinearHeading(RED_RIGHT_PARK.position, Math.toRadians(0)) // Move to parking position
                .build();

        Action v3RedNearGeRightPark = drive.actionBuilder(RED_NEAR_START_POSE)
                .strafeToLinearHeading(RED_NEAR_RIGHT_SPIKE.position, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_NEAR_RIGHT_SPIKE.position.x - RED_NEAR_RIGHT_SPIKE_BACK_X, RED_NEAR_RIGHT_SPIKE.position.y))
                .strafeTo(new Vector2d(12, -58))
                .strafeTo(RED_RIGHT_PARK.position) // Move to parking position
                .build();

        Action trajectoryActionCloseOut = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(48, 12))
                .build();
        // Set to true when an AprilTag target is detected
        boolean targetFound = false;

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;


        selectedSide = visionProcessor.getSelection();
//        visionPortal.stopStreaming();
        visionPortal.setProcessorEnabled(visionProcessor, false);
        telemetry.addData("selectedSide: ", selectedSide.toString());
        telemetry.addData("Identified", visionProcessor.getTelemetry());

        telemetry.update();

        int desiredTagId = Utils.GetDesiredTagId(Alliance.RED, selectedSide);
        visionPortal.setProcessorEnabled(aprilTag, true);


        Action trajectoryToRun = null;
        switch (selectedSide){
            case LEFT:
                trajectoryToRun = v3RedNearGeLeftPark;
                break;
            case CENTER:
                trajectoryToRun = v3RedNearGeCenterScore;
                break;
            case RIGHT:
                trajectoryToRun = v3RedNearGeRightPark;
                break;
            default:
                trajectoryToRun = v3RedNearGeCenterScore;
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryToRun
                )
        );

        desiredTag  = null;
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == desiredTagId) ){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);

            drive.alignToAprilTag(desiredTag);
        }


        // (TBD) - move linear actuator forward.
        // tilt elbow up
        // turn wrist 180
        // open box
        Actions.runBlocking(
                new SequentialAction(
                        outtake.wristUp(),
                        outtake.elbowScore(),
                        outtake.openBox(),
                        trajectoryActionCloseOut
                )
        );
    } // runOpMode


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