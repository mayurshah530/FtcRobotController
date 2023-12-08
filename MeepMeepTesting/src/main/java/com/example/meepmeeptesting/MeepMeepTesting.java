package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

// Install using https://github.com/acmerobotics/MeepMeep (Road runner 1.0 fork for MeepMeep)
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        double HALF_ROBO_LEN = 9;
        double HALF_ROBO_WIDTH = 9;

        // Blue near start pose
        Pose2d BLUE_NEAR_START_POSE = new Pose2d(12, 72-HALF_ROBO_LEN, -Math.PI/2.0);
        Pose2d BLUE_LEFT_PARK = new Pose2d(58 - HALF_ROBO_LEN, 56, 0);

        Pose2d RED_NEAR_START_POSE = new Pose2d(12, -(72-9), Math.PI/2.0);
        Pose2d RED_RIGHT_PARK = new Pose2d(58 - HALF_ROBO_LEN, -56, 0);

        Action TrajectoryBlueNearToLeftPark = myBot.getDrive().actionBuilder(BLUE_NEAR_START_POSE)
                .splineToLinearHeading(BLUE_LEFT_PARK, 0)
                .build();

        Action TrajectoryRedNearToRightPark = myBot.getDrive().actionBuilder(RED_NEAR_START_POSE)
                .splineToLinearHeading(RED_RIGHT_PARK, 0)
                .build();

        // 1. Start position: blue near; Game Element location: Center.
        Pose2d BLUE_NEAR_CENTER_SPIKE = new Pose2d(12, 24.5+HALF_ROBO_LEN, -Math.PI/2.0);
        Pose2d BLUE_BOARD_CENTER_TAG = new Pose2d(58 - HALF_ROBO_LEN, 36, 0);

        Action TrajectoryBlueNearGeCenter = myBot.getDrive().actionBuilder(BLUE_NEAR_START_POSE)
                .lineToY(BLUE_NEAR_CENTER_SPIKE.position.y)
                .waitSeconds(1)
                .setReversed(true)
                .lineToY(BLUE_NEAR_CENTER_SPIKE.position.y + 18)
                .setReversed(false)
                .splineToLinearHeading(BLUE_BOARD_CENTER_TAG, 0)
                .waitSeconds(2)
//                .strafeToLinearHeading(BLUE_LEFT_PARK.position, Math.toRadians(-135))
                .strafeTo(BLUE_LEFT_PARK.position)
//                .turn(-Math.toRadians(135)) // Optional: Turn so that you are ready to grab pixels in manual mode.
                .waitSeconds(2)
                .build();

        // Start position blue far; GE: Center
        Pose2d BLUE_FAR_START_POSE = new Pose2d(-36, 72-HALF_ROBO_LEN, -Math.PI/2.0);
        Pose2d BLUE_FAR_CENTER_SPIKE = new Pose2d(-36, 24.5+HALF_ROBO_LEN, -Math.PI/2.0);
        Pose2d BLUE_CENTER_PARK = new Pose2d(58 - HALF_ROBO_LEN, 2 + HALF_ROBO_LEN , 0);

        Action TrajectoryBlueFarGeCenter = myBot.getDrive().actionBuilder(BLUE_FAR_START_POSE)
                .lineToY(BLUE_FAR_CENTER_SPIKE.position.y)
                .waitSeconds(1)
                .setReversed(true)
                .lineToY(BLUE_FAR_CENTER_SPIKE.position.y + 6)
                .setReversed(false)
                .strafeTo(new Vector2d(-48, BLUE_FAR_CENTER_SPIKE.position.y + 6))
                .splineToLinearHeading(new Pose2d(-48, 12, 0), 0)
                .lineToX(BLUE_BOARD_CENTER_TAG.position.x)
                .strafeTo(BLUE_BOARD_CENTER_TAG.position)
                .waitSeconds(2)
                .strafeTo(BLUE_CENTER_PARK.position)
                .turn(Math.PI +1e-6)
                .build();

        myBot.runAction(TrajectoryBlueNearGeCenter);


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}