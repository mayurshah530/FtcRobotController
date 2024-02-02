package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;

@Config
@Autonomous(name = "Pixel Score Only", group = "RoadRunner 1.0")
public class AutoScoreOnlyTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Outtake outtake = new Outtake(hardwareMap);

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        // move linear actuator forward. (how much? how long?)
        // tilt elbow up
        // turn wrist 180
        // move wheels forward, look for signs of deceleration and zero velocity
        // open box
        // close box + move wrist in
        // put boxlever down + move to park position
        Actions.runBlocking(
                new SequentialAction(
                        outtake.actuatorExpand(Outtake.PARAMS.ACTUATOR_ENCODER_COUNT),
                        new ParallelAction(
                                outtake.moveBoxLeverUp(),
                                outtake.moveWristOut()
                        ),
                        outtake.openBox(),
                        new ParallelAction(
                                outtake.closeBox(),
                                outtake.moveWristIn()
                        ),
                        outtake.moveBoxLeverDown()
                )
        );

    } // runOpMode
}