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
@Autonomous(name = "April Tag Motion Test", group = "RoadRunner 1.0")
public class AprilTagTest extends LinearOpMode {


    public final double HALF_ROBO_LEN = 9;
    public static double TAG_BOT_OFFSET = 18; //20.75;

    public static boolean ENABLE_APRIL_TAG_GLOBAL_POSE = false;

    // Start position red far

    // START POSITIONS
    Pose2d RED_NEAR_START_POSE = new Pose2d(12, -(72-HALF_ROBO_LEN), Math.PI/2.0);

    Pose2d RED_ALLIANCE_CENTER_TAG_REF = new Pose2d(60.25, -35.41, 0);
    Pose2d RED_ALLIANCE_CENTER_TAG = new Pose2d(60.25 - TAG_BOT_OFFSET, -35.41, 0);

    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ScoringElementLocation selectedSide = ScoringElementLocation.CENTER;

    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_ALLIANCE_CENTER_TAG);

        initVisionPortal();
        visionProcessor.SetAlliance(Alliance.RED);

        Action trajectoryActionCloseOut = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(48, 12))
                .build();
        // Set to true when an AprilTag target is detected
        boolean targetFound = false;

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        int desiredTagId = Utils.GetDesiredTagId(Alliance.RED, selectedSide);
        visionPortal.setProcessorEnabled(aprilTag, true);
        sleep(100);
        runtime.reset();
        boolean aprilTagConverged = false;
        desiredTag = null;

        while(opModeIsActive()){
            while(runtime.seconds() < 10.0 && !aprilTagConverged) {
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

                if (targetFound) {
                    telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    telemetry.addData("x", "%5.1f inches", desiredTag.ftcPose.x);
                    telemetry.addData("y", "%5.1f inches", desiredTag.ftcPose.y);
                    telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                    telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
//                    telemetry.addData("Robot Pose", "%3.1f, %3.1 %3.1f", drive.pose.position.x, drive.pose.position.y, drive.pose.heading.log());
                    telemetry.update();

                    if (ENABLE_APRIL_TAG_GLOBAL_POSE){
//                        double cameraX = RED_ALLIANCE_CENTER_TAG_REF.position.x - desiredTag.ftcPose.y;
//                        double cameraY = RED_ALLIANCE_CENTER_TAG_REF.position.y + desiredTag.ftcPose.x;
//                        double robotX = cameraX - MecanumDrive.CALIBRATION.CHASSIS_FROM_CAMERA_X;
//                        double robotY = cameraY - MecanumDrive.CALIBRATION.CHASSIS_FROM_CAMERA_Y;
//                        // YAW sign?
//                        Pose2d newRobotPose = new Pose2d(robotX, robotY, desiredTag.ftcPose.yaw);
//                        telemetry.addData("Odom Pose", "%3.1f, %3.1f %3.1f", drive.pose.position.x, drive.pose.position.y, drive.pose.heading.log());
//                        telemetry.addData("April Pose", "%3.1f, %3.1f %3.1f", newRobotPose.position.x, newRobotPose.position.y, newRobotPose.heading.log());
//                        telemetry.update();
//                        drive.setPose(newRobotPose);
//                        sleep(100);
//
//                        Action v4StrafeToCenterScorePose = drive.actionBuilder(newRobotPose)
//                                .strafeTo(RED_ALLIANCE_CENTER_TAG.position)
//                                .build();
//                        Actions.runBlocking(v4StrafeToCenterScorePose);

                        // Strafe to align with the AprilTag center.
                        Pose2d currentPose = drive.pose;
                        Vector2d desiredPos = new Vector2d(currentPose.position.x, currentPose.position.y - desiredTag.ftcPose.x);
                        telemetry.addData("CurrentPos: ", "%3.1f, %3.1f", currentPose.position.x, currentPose.position.y);
                        telemetry.addData("DesiredPos: ", "%3.1f, %3.1f", desiredPos.x, desiredPos.y);

                        Action v4StrafeToAprilTag = drive.actionBuilder(currentPose)
                                .strafeTo(desiredPos)
                                .build();
                        Actions.runBlocking(v4StrafeToAprilTag);

                        break;

                    } else {

                        aprilTagConverged = drive.alignToAprilTag(desiredTag);
                    }
                    sleep(10);
                }
            }
            // set powers to 0
            drive.moveRobot(0,0,0);

            if (null != desiredTag){

                telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("x", "%5.1f inches", desiredTag.ftcPose.x);
                telemetry.addData("y", "%5.1f inches", desiredTag.ftcPose.y);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            }

            telemetry.addData("AprilTagConverged? ", aprilTagConverged);
            telemetry.addData("runtime: ", runtime.seconds());
            telemetry.update();
        }

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