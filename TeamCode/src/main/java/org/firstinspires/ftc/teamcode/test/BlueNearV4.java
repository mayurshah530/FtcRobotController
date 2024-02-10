package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.ScoringElementLocation;
import org.firstinspires.ftc.teamcode.common.Utils;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name = "\uD83D\uDD35 - Blue Near V4", group = "RoadRunner 1.0")
public class BlueNearV4 extends LinearOpMode {
    public static boolean ENABLE_APRIL_TAG_CORRECTION = true;

    public static boolean ENABLE_APRIL_TAG_GLOBAL_POSE = false;
    double HALF_ROBO_LEN = 9;

    // START POSITIONS
    Pose2d BLUE_NEAR_START_POSE = new Pose2d(12, 72-HALF_ROBO_LEN, -Math.PI/2.0);


    // PARK POSITIONS
    Pose2d BLUE_LEFT_PARK = new Pose2d(58 - HALF_ROBO_LEN, 63, 0);

    // SPIKE POSITIONS
    Pose2d BLUE_NEAR_CENTER_SPIKE = new Pose2d(12, 24.5+HALF_ROBO_LEN, -Math.PI/2.0);
    Pose2d BLUE_NEAR_LEFT_SPIKE = new Pose2d(23.5-HALF_ROBO_LEN, (30), 0);
    Pose2d BLUE_NEAR_RIGHT_SPIKE = new Pose2d(12, 30,   Math.PI);

    public static double TAG_BOT_OFFSET = 14.0;
    Pose2d BLUE_ALLIANCE_LEFT_TAG = new Pose2d(60.25 - TAG_BOT_OFFSET, 41.41, 0);
    Pose2d BLUE_ALLIANCE_CENTER_TAG = new Pose2d(60.25 - TAG_BOT_OFFSET, 35.41, 0);
    Pose2d BLUE_ALLIANCE_RIGHT_TAG = new Pose2d(60.25 - TAG_BOT_OFFSET, 29.41, 0);

    Pose2d BLUE_ALLIANCE_LEFT_TAG_REF = new Pose2d(60.25 - TAG_BOT_OFFSET, 41.41, 0);
    Pose2d BLUE_ALLIANCE_CENTER_TAG_REF = new Pose2d(60.25 - TAG_BOT_OFFSET, 35.41, 0);
    Pose2d BLUE_ALLIANCE_RIGHT_TAG_REF = new Pose2d(60.25 - TAG_BOT_OFFSET, 29.41, 0);


    public static double CLOSEOUT_BOT_OFFSET = 20;
    Pose2d BLUE_ALLIANCE_LEFT_TAG_CLOSEOUT = new Pose2d(60.25 - CLOSEOUT_BOT_OFFSET, 41.41, 0);
    Pose2d BLUE_ALLIANCE_CENTER_TAG_CLOSEOUT = new Pose2d(60.25 - CLOSEOUT_BOT_OFFSET, 35.41, 0);
    Pose2d BLUE_ALLIANCE_RIGHT_TAG_CLOSEOUT = new Pose2d(60.25 - CLOSEOUT_BOT_OFFSET, 29.41, 0);


    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ScoringElementLocation selectedSide = ScoringElementLocation.CENTER;

    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private final ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_NEAR_START_POSE);
        Outtake outtake = new Outtake(hardwareMap);


        initVisionPortal();
        visionProcessor.SetAlliance(Alliance.BLUE);

        Action v4BlueNearGeCenterScore = drive.actionBuilder(BLUE_NEAR_START_POSE)
                .strafeTo(BLUE_NEAR_CENTER_SPIKE.position)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(12, 40))
                .turn(Math.toRadians(90))
                .strafeTo(BLUE_ALLIANCE_CENTER_TAG.position)
                .build();

        Action v4BlueNearGeLeftScore = drive.actionBuilder(BLUE_NEAR_START_POSE)
                .strafeToLinearHeading(BLUE_NEAR_LEFT_SPIKE.position, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_NEAR_LEFT_SPIKE.position.x - 2, BLUE_NEAR_LEFT_SPIKE.position.y))
                .strafeTo(new Vector2d(12, 54))
                .strafeToLinearHeading(BLUE_ALLIANCE_LEFT_TAG.position, 0)
                .build();

        Action v4BlueNearGeRightScore = drive.actionBuilder(BLUE_NEAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(BLUE_NEAR_RIGHT_SPIKE.position.x + 5, BLUE_NEAR_RIGHT_SPIKE.position.y), Math.PI)
                .strafeTo(new Vector2d(BLUE_NEAR_RIGHT_SPIKE.position.x -2, BLUE_NEAR_RIGHT_SPIKE.position.y))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_NEAR_RIGHT_SPIKE.position.x + 5, BLUE_NEAR_RIGHT_SPIKE.position.y))
                .turn(-Math.PI+1e-6)
                .strafeToLinearHeading(BLUE_ALLIANCE_RIGHT_TAG.position, 0)
                .build();

        Action trajectoryActionCloseOutCenter = drive.actionBuilder(BLUE_ALLIANCE_CENTER_TAG_CLOSEOUT)
                .strafeTo(BLUE_LEFT_PARK.position)
                .build();
        Action trajectoryActionCloseOutLeft = drive.actionBuilder(BLUE_ALLIANCE_LEFT_TAG_CLOSEOUT)
                .strafeTo(BLUE_LEFT_PARK.position)
                .build();
        Action trajectoryActionCloseOutRight = drive.actionBuilder(BLUE_ALLIANCE_RIGHT_TAG_CLOSEOUT)
                .strafeTo(BLUE_LEFT_PARK.position)
                .build();

        // Set to true when an AprilTag target is detected
        boolean targetFound = false;
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
        int desiredTagId = Utils.GetDesiredTagId(Alliance.RED, selectedSide);

        Action trajectoryToRun = null;
        Action trajectoryActionCloseOut = null;
        switch (selectedSide){
            case LEFT:
                trajectoryToRun = v4BlueNearGeLeftScore;
                trajectoryActionCloseOut = trajectoryActionCloseOutLeft;
                break;
            case CENTER:
                trajectoryToRun = v4BlueNearGeCenterScore;
                trajectoryActionCloseOut = trajectoryActionCloseOutCenter;
                break;
            case RIGHT:
                trajectoryToRun = v4BlueNearGeRightScore;
                trajectoryActionCloseOut = trajectoryActionCloseOutRight;
                break;
            default:
                trajectoryToRun = v4BlueNearGeCenterScore;
                trajectoryActionCloseOut = trajectoryActionCloseOutCenter;
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryToRun
                )
        );


        if(ENABLE_APRIL_TAG_CORRECTION){
            boolean aprilTagConverged = false;
            desiredTag = null;
            runtime.reset();
            while(runtime.seconds() < 3.0 && !aprilTagConverged) {
                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if ((detection.metadata != null) && (detection.id == desiredTagId)) {
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
                    telemetry.addData("x", "%5.1f inches", desiredTag.ftcPose.x);
                    telemetry.addData("y", "%5.1f inches", desiredTag.ftcPose.y);
                    telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                    telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                    telemetry.addData("Robot Pose", "%3.1f, %3.1f", drive.pose.position.x, drive.pose.position.y);
                    telemetry.addData("runtime ", runtime.seconds());
                    telemetry.update();
                    if (ENABLE_APRIL_TAG_GLOBAL_POSE){
                        double cameraX = BLUE_ALLIANCE_CENTER_TAG_REF.position.x - desiredTag.ftcPose.y;
                        double cameraY = BLUE_ALLIANCE_CENTER_TAG_REF.position.y + desiredTag.ftcPose.x;
                        double robotX = cameraX - MecanumDrive.CALIBRATION.CHASSIS_FROM_CAMERA_X;
                        double robotY = cameraY - MecanumDrive.CALIBRATION.CHASSIS_FROM_CAMERA_Y;
                        // YAW sign?
                        Pose2d newRobotPose = new Pose2d(robotX, robotY, desiredTag.ftcPose.yaw);
                        telemetry.addData("Odom Pose", "%3.1f, %3.1f %3.1f", drive.pose.position.x, drive.pose.position.y, drive.pose.heading.log());
                        telemetry.addData("April Pose", "%3.1f, %3.1f %3.1f", newRobotPose.position.x, newRobotPose.position.y, newRobotPose.heading.log());
                        telemetry.update();
                        drive.setPose(newRobotPose);

                        Action v4StrafeToCenterScorePose = drive.actionBuilder(newRobotPose)
                                .strafeToLinearHeading(BLUE_ALLIANCE_RIGHT_TAG.position, 0)
                                .build();
                        Actions.runBlocking(v4StrafeToCenterScorePose);
                        break;

                    } else {
                        aprilTagConverged = drive.alignToAprilTag(desiredTag);
                    }

                } else {
                    telemetry.addLine("Target not found");
                    telemetry.update();
                }
            }
            // set powers to 0
            drive.moveRobot(0,0,0);

            telemetry.addData("AprilTagConverged? ", aprilTagConverged);
            telemetry.addData("runtime: ", runtime.seconds());
            telemetry.update();
        }

        Actions.runBlocking(
                new SequentialAction(
                        outtake.pixelDropAction,
                        trajectoryActionCloseOut)
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
