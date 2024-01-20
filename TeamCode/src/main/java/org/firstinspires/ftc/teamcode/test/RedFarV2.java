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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.ScoringElementLocation;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@Autonomous(name = "Red Far V2", group = "RoadRunner 1.0")
@Disabled
public class RedFarV2 extends LinearOpMode {

    double HALF_ROBO_LEN = 9;
    double HALF_ROBO_WIDTH = 9;
    double OUTTAKE_TIME = 0.3;

    // Start position red far; GE: Center
    Pose2d RED_FAR_START_POSE = new Pose2d(-36, -(72-HALF_ROBO_LEN), Math.PI/2.0);
    Pose2d RED_FAR_CENTER_SPIKE = new Pose2d(-36, -(24.5+HALF_ROBO_LEN), Math.PI/2.0);

    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ScoringElementLocation selectedSide = ScoringElementLocation.UNKNOWN;

    private DcMotor intake_front = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        intake_front = hardwareMap.get(DcMotor.class,"intake_front");

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_FAR_START_POSE);

        initVisionPortal();
        visionProcessor.SetAlliance(Alliance.RED);

        Action TrajectoryRedNearGeCenter = drive.actionBuilder(drive.pose)
                .lineToY(RED_FAR_CENTER_SPIKE.position.y-5)
                .waitSeconds(1)
//                .setReversed(true)
//                .splineToLinearHeading(RED_LEFT_PARK_REVERSE, 0)
                .build();

        Action TrajectoryRedNearGeLeft = drive.actionBuilder(drive.pose)
                .lineToY(RED_FAR_CENTER_SPIKE.position.y)
                .turn(Math.PI/2)
//                .strafeTo(new Vector2d(RED_NEAR_START_POSE.position.x, 60))
//                .strafeTo(new Vector2d(48, 60))
//                .waitSeconds(2)
                .build();

        Action TrajectoryRedNearGeRight = drive.actionBuilder(drive.pose)
                .lineToY(RED_FAR_CENTER_SPIKE.position.y)
                .turn(-Math.PI/2)
//                .setReversed(true)
//                .strafeTo(new Vector2d(48, 60))
                .build();

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;


        selectedSide = visionProcessor.getSelection();
        visionPortal.stopStreaming();
        visionPortal.setProcessorEnabled(visionProcessor, false);
        telemetry.addData("selectedSide: ", selectedSide.toString());
        telemetry.update();


        Action trajectoryToRun = TrajectoryRedNearGeCenter;
        switch (selectedSide){
            case LEFT:
                trajectoryToRun = TrajectoryRedNearGeLeft;
                break;
            case CENTER:
                trajectoryToRun = TrajectoryRedNearGeCenter;
                break;
            case RIGHT:
                trajectoryToRun = TrajectoryRedNearGeRight;
                break;
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryToRun, // Example of a drive action
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

        intake_front.setPower(-0.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < OUTTAKE_TIME)) {
        }
        intake_front.setPower(0.0);



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