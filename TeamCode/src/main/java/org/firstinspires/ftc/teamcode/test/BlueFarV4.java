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
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@Autonomous(name = "\uD83D\uDD35 --- Blue Far V4", group = "RoadRunner 1.0")
public class BlueFarV4 extends LinearOpMode {

    double HALF_ROBO_LEN = 9;

    // START POSITIONS
    Pose2d BLUE_FAR_START_POSE = new Pose2d(-36, (72-HALF_ROBO_LEN), -Math.PI/2.0);
    // PARK POSITIONS
    Pose2d BLUE_CENTER_PARK = new Pose2d(56, 12 , 0);
    // SPIKE POSITIONS
    Pose2d BLUE_FAR_CENTER_SPIKE = new Pose2d(-36, (24.5+HALF_ROBO_LEN), Math.PI/2.0);
    Pose2d BLUE_FAR_RIGHT_SPIKE = new Pose2d(-(46 -HALF_ROBO_LEN), 30, -Math.PI);
    Pose2d BLUE_FAR_LEFT_SPIKE = new Pose2d(-(23.5+HALF_ROBO_LEN), (30), 0);
    public static double TAG_BOT_OFFSET = 20.25;
    Pose2d BLUE_ALLIANCE_LEFT_TAG = new Pose2d(60.25 - TAG_BOT_OFFSET, 41.41, 0);
    Pose2d BLUE_ALLIANCE_CENTER_TAG = new Pose2d(60.25 - TAG_BOT_OFFSET, 35.41, 0);
    Pose2d BLUE_ALLIANCE_RIGHT_TAG = new Pose2d(60.25 - TAG_BOT_OFFSET, 29.41, 0);


    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ScoringElementLocation selectedSide = ScoringElementLocation.UNKNOWN;


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_FAR_START_POSE);
        Outtake outtake = new Outtake(hardwareMap);

        initVisionPortal();
        visionProcessor.SetAlliance(Alliance.BLUE);


        Action v4BlueFarGeCenterScore = drive.actionBuilder(BLUE_FAR_START_POSE)
                .strafeTo(BLUE_FAR_CENTER_SPIKE.position)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_FAR_CENTER_SPIKE.position.x, BLUE_FAR_CENTER_SPIKE.position.y + 10)) // come back
                .strafeTo(new Vector2d(-52, BLUE_FAR_CENTER_SPIKE.position.y + 10))
                .strafeTo(new Vector2d(-52, 12)) // move fwd
                .turn(Math.PI/2)
                .strafeToLinearHeading(new Vector2d(28, 12), 0 )
                .strafeToLinearHeading(BLUE_ALLIANCE_CENTER_TAG.position, 0 )
                .build();

        Action v4BlueFarGeLeftScore = drive.actionBuilder(BLUE_FAR_START_POSE)
                .splineToLinearHeading(BLUE_FAR_LEFT_SPIKE, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_FAR_LEFT_SPIKE.position.x - 5, BLUE_FAR_LEFT_SPIKE.position.y))
                .strafeTo(new Vector2d(BLUE_FAR_LEFT_SPIKE.position.x - 5, 10))
                .strafeToLinearHeading(new Vector2d(35, 10), 0 )
                .strafeToLinearHeading(BLUE_ALLIANCE_LEFT_TAG.position, 0 )
                .build();

        Action v4BluefarGeRightScore = drive.actionBuilder(BLUE_FAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(BLUE_FAR_RIGHT_SPIKE.position.x, BLUE_FAR_RIGHT_SPIKE.position.y), -Math.PI)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_FAR_RIGHT_SPIKE.position.x + 2, BLUE_FAR_RIGHT_SPIKE.position.y))
                .strafeTo(new Vector2d(BLUE_FAR_RIGHT_SPIKE.position.x + 2, 10))
                .turn(-(Math.PI+1e-6))
                .strafeToLinearHeading(new Vector2d(20, 10), 0 )
                .strafeToLinearHeading(BLUE_ALLIANCE_RIGHT_TAG.position, 0 )
                .build();


        Action trajectoryActionCloseOutCenter = drive.actionBuilder(BLUE_ALLIANCE_CENTER_TAG)
                .strafeTo(BLUE_CENTER_PARK.position)
                .build();
        Action trajectoryActionCloseOutLeft = drive.actionBuilder(BLUE_ALLIANCE_LEFT_TAG)
                .strafeTo(BLUE_CENTER_PARK.position)
                .build();
        Action trajectoryActionCloseOutRight = drive.actionBuilder(BLUE_ALLIANCE_RIGHT_TAG)
                .strafeTo(BLUE_CENTER_PARK.position)
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


        Action trajectoryToRun = null;
        Action trajectoryActionCloseOut = null;
        switch (selectedSide){
            case LEFT:
                trajectoryToRun = v4BlueFarGeLeftScore;
                trajectoryActionCloseOut = trajectoryActionCloseOutLeft;
                break;
            case CENTER:
                trajectoryToRun = v4BlueFarGeCenterScore;
                trajectoryActionCloseOut = trajectoryActionCloseOutCenter;
                break;
            case RIGHT:
                trajectoryToRun = v4BluefarGeRightScore;
                trajectoryActionCloseOut = trajectoryActionCloseOutRight;
                break;
            default:
                trajectoryToRun = v4BlueFarGeCenterScore;
                trajectoryActionCloseOut = trajectoryActionCloseOutCenter;
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryToRun
                )
        );

        // TODO: INSERT APRIL TAG ALIGNMENT PART

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
