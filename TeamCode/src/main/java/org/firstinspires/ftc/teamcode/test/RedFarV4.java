package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
@Autonomous(name = "\uD83D\uDD34 --- Red Far V4", group = "RoadRunner 1.0")
public class RedFarV4 extends LinearOpMode {

    public static boolean ENABLE_APRIL_TAG_CORRECTION = true;

    public static boolean ENABLE_APRIL_TAG_GLOBAL_POSE = false;

    public static double HALF_ROBO_LEN = 9;

    // Start position red far

    // START POSITIONS
    Pose2d RED_FAR_START_POSE = new Pose2d(-36, -(72-HALF_ROBO_LEN), Math.PI/2.0);


    // PARK POSITIONS
    Pose2d RED_CENTER_PARK = new Pose2d(48, -12 , 0);

    // SPIKE Locations
    Pose2d RED_FAR_CENTER_SPIKE = new Pose2d(-36, -(24.5+HALF_ROBO_LEN), -Math.PI/2.0);
    Pose2d RED_FAR_LEFT_SPIKE = new Pose2d(-(46 -HALF_ROBO_LEN), -30, Math.PI);
    Pose2d RED_FAR_RIGHT_SPIKE = new Pose2d(-(21.5+HALF_ROBO_LEN), -(33), 0);

    // TAG locations
    public static double TAG_BOT_X_OFFSET = 19.75;
    public static double TAG_BOT_Y_OFFSET = 0.0;

    Pose2d RED_ALLIANCE_LEFT_TAG = new Pose2d(60.25 - TAG_BOT_X_OFFSET, -29.41 - TAG_BOT_Y_OFFSET, 0);
    Pose2d RED_ALLIANCE_CENTER_TAG = new Pose2d(60.25 - TAG_BOT_X_OFFSET, -38.41- TAG_BOT_Y_OFFSET, 0);
    Pose2d RED_ALLIANCE_RIGHT_TAG = new Pose2d(60.25 - TAG_BOT_X_OFFSET, -41.41- TAG_BOT_Y_OFFSET, 0);
    // wait location
    Pose2d RED_ALLIANCE_LEFT_TAG_REF = new Pose2d(60.25, -29.41, 0);
    Pose2d RED_ALLIANCE_CENTER_TAG_REF = new Pose2d(60.25, -35.41, 0);
    Pose2d RED_ALLIANCE_RIGHT_TAG_REF = new Pose2d(60.25, -41.41, 0);


    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ScoringElementLocation selectedSide = ScoringElementLocation.CENTER;

    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_FAR_START_POSE);
        Outtake outtake = new Outtake(hardwareMap);

        initVisionPortal();
        visionProcessor.SetAlliance(Alliance.RED);


        Action v4RedFarGeCenterScore = drive.actionBuilder(RED_FAR_START_POSE)
                .strafeTo(RED_FAR_CENTER_SPIKE.position)
                .waitSeconds(1)
                .strafeTo(new Vector2d(RED_FAR_CENTER_SPIKE.position.x, RED_FAR_CENTER_SPIKE.position.y - 10)) // come back
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-52, RED_FAR_CENTER_SPIKE.position.y - 10)) // slide left
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-52, -12)) // move fwd
                .turn(-Math.PI/2)
                .strafeToLinearHeading(new Vector2d(RED_ALLIANCE_CENTER_TAG.position.x, -12), 0 )
                .strafeToLinearHeading(RED_ALLIANCE_CENTER_TAG.position, 0 )
                .build();


        Action v4RedFarGeLeftScore = drive.actionBuilder(RED_FAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(RED_FAR_LEFT_SPIKE.position.x-2, RED_FAR_LEFT_SPIKE.position.y), Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_FAR_LEFT_SPIKE.position.x + 0, RED_FAR_LEFT_SPIKE.position.y))
                .strafeTo(new Vector2d(RED_FAR_LEFT_SPIKE.position.x + 0, -12))
                .turn(Math.PI+1e-6)
                .strafeToLinearHeading(new Vector2d(20, -12), 0 )
                .strafeToLinearHeading(RED_ALLIANCE_LEFT_TAG.position, 0 )
                .build();

        Action v4RedfarGeRightScore = drive.actionBuilder(RED_FAR_START_POSE)
                .strafeTo(new Vector2d(RED_FAR_START_POSE.position.x - 6, RED_FAR_START_POSE.position.y+3))
                .splineToLinearHeading(RED_FAR_RIGHT_SPIKE, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_FAR_RIGHT_SPIKE.position.x - 5, RED_FAR_RIGHT_SPIKE.position.y))
                .strafeTo(new Vector2d(RED_FAR_RIGHT_SPIKE.position.x - 5, -10))
                .strafeToLinearHeading(new Vector2d(RED_ALLIANCE_RIGHT_TAG.position.x, -12), 0 )
                .strafeToLinearHeading(RED_ALLIANCE_RIGHT_TAG.position, 0 )
                .build();

        Action trajectoryActionCloseOutCenter = drive.actionBuilder(RED_ALLIANCE_CENTER_TAG)
                .strafeTo(RED_CENTER_PARK.position)
                .build();
        Action trajectoryActionCloseOutLeft = drive.actionBuilder(RED_ALLIANCE_LEFT_TAG)
                .strafeTo(RED_CENTER_PARK.position)
                .build();
        Action trajectoryActionCloseOutRight = drive.actionBuilder(RED_ALLIANCE_RIGHT_TAG)
                .strafeTo(RED_CENTER_PARK.position)
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
        Action trajectoryActionCloseOut = null;
        switch (selectedSide){
            case LEFT:
                trajectoryToRun = v4RedFarGeLeftScore;
                trajectoryActionCloseOut = trajectoryActionCloseOutLeft;
                break;
            case CENTER:
                trajectoryToRun = v4RedFarGeCenterScore;
                trajectoryActionCloseOut = trajectoryActionCloseOutCenter;
                break;
            case RIGHT:
                trajectoryToRun = v4RedfarGeRightScore;
                trajectoryActionCloseOut = trajectoryActionCloseOutRight;
                break;
            default:
                trajectoryToRun = v4RedFarGeCenterScore;
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
                        double robotX = RED_ALLIANCE_CENTER_TAG_REF.position.x - desiredTag.ftcPose.y;
                        double robotY = RED_ALLIANCE_CENTER_TAG_REF.position.y + desiredTag.ftcPose.x;
                        Pose2d newRobotPose = new Pose2d(robotX, robotY, desiredTag.ftcPose.yaw);
                        telemetry.addData("Odom Pose", "%3.1f, %3.1f %3.1f", drive.pose.position.x, drive.pose.position.y, drive.pose.heading.log());
                        telemetry.addData("April Pose", "%3.1f, %3.1f %3.1f", newRobotPose.position.x, newRobotPose.position.y, newRobotPose.heading.log());
                        telemetry.update();
                        drive.setPose(newRobotPose);

                        Action v4StrafeToCenterScorePose = drive.actionBuilder(newRobotPose)
                                .strafeToLinearHeading(RED_ALLIANCE_RIGHT_TAG.position, 0)
                                .build();
                        Actions.runBlocking(v4StrafeToCenterScorePose);

                    } else {
                        aprilTagConverged = drive.alignToAprilTag(desiredTag);
                    }

                } else {
                    telemetry.addLine("Target not found");
                    telemetry.update();
                }
            }
            telemetry.addData("AprilTagConverged? ", aprilTagConverged);
            telemetry.addData("runtime: ", runtime.seconds());
            telemetry.update();
        }

        Actions.runBlocking(
                new SequentialAction(
                        outtake.pixelDropAction,
                        trajectoryActionCloseOut)
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