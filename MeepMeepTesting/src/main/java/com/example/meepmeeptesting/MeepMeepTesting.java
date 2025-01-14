package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

// Install using https://github.com/acmerobotics/MeepMeep (Road runner 1.0 fork for MeepMeep)
public class MeepMeepTesting {

    public static Pose2d mirrorPose2d(Pose2d pose2dIn){
        return new Pose2d(new Vector2d(pose2dIn.position.x, -pose2dIn.position.y), pose2dIn.heading.inverse());
    }
    public static void main(String[] args) {

        double HALF_ROBO_LEN = 9;
        double HALF_ROBO_WIDTH = 9;

        // START POSITIONS
        Pose2d BLUE_NEAR_START_POSE = new Pose2d(12, 72-HALF_ROBO_LEN, -Math.PI/2.0);
        Pose2d BLUE_FAR_START_POSE = new Pose2d(-36, (72-HALF_ROBO_LEN), -Math.PI/2.0);


        Pose2d RED_NEAR_START_POSE = new Pose2d(12, -(72-HALF_ROBO_LEN), Math.PI/2.0);
        Pose2d RED_FAR_START_POSE = new Pose2d(-36, -(72-HALF_ROBO_LEN), Math.PI/2.0);


        // PARK POSITIONS
        Pose2d BLUE_LEFT_PARK = new Pose2d(58 - HALF_ROBO_LEN, 56, 0);
        Pose2d BLUE_LEFT_PARK_REVERSE = new Pose2d(58 - HALF_ROBO_LEN, 56, -Math.PI);
        Pose2d BLUE_CENTER_PARK = new Pose2d(58 - HALF_ROBO_LEN, 12 , 0);

        Pose2d RED_RIGHT_PARK = new Pose2d(58 - HALF_ROBO_LEN, -66, 0);
        Pose2d RED_CENTER_PARK = new Pose2d(58 - HALF_ROBO_LEN, -12 , 0);


        // SPIKE POSITIONS
        Pose2d RED_NEAR_CENTER_SPIKE = new Pose2d(12, -(24.5+HALF_ROBO_LEN), Math.PI/2.0);
        Pose2d RED_NEAR_RIGHT_SPIKE = new Pose2d(23.5-HALF_ROBO_LEN, -(30), 0);
        Pose2d RED_NEAR_LEFT_SPIKE = new Pose2d(12, -30, -Math.PI);

        Pose2d RED_FAR_CENTER_SPIKE = new Pose2d(-36, -(24.5+HALF_ROBO_LEN), Math.PI/2.0);
        Pose2d RED_FAR_RIGHT_SPIKE = new Pose2d(-(23.5+HALF_ROBO_LEN), -(30), 0);
        Pose2d RED_FAR_LEFT_SPIKE = new Pose2d(-(46 -HALF_ROBO_LEN), -30, Math.PI);

        Pose2d BLUE_NEAR_CENTER_SPIKE = new Pose2d(12, 24.5+HALF_ROBO_LEN, -Math.PI/2.0);
        Pose2d BLUE_NEAR_LEFT_SPIKE = new Pose2d(23.5-HALF_ROBO_LEN, (30), 0);
        Pose2d BLUE_NEAR_RIGHT_SPIKE = new Pose2d(12, 30,   Math.PI);

        Pose2d BLUE_FAR_CENTER_SPIKE = new Pose2d(-36, (24.5+HALF_ROBO_LEN), Math.PI/2.0);
        Pose2d BLUE_FAR_RIGHT_SPIKE = new Pose2d(-(46 -HALF_ROBO_LEN), 30, -Math.PI);
        Pose2d BLUE_FAR_LEFT_SPIKE = new Pose2d(-(23.5+HALF_ROBO_LEN), (30), 0);

        // TAG locations
        final double TAG_BOT_OFFSET = 20.25;
        Pose2d BLUE_ALLIANCE_LEFT_TAG = new Pose2d(60.25 - TAG_BOT_OFFSET, 41.41, 0);
        Pose2d BLUE_ALLIANCE_CENTER_TAG = new Pose2d(60.25 - TAG_BOT_OFFSET, 35.41, 0);
        Pose2d BLUE_ALLIANCE_RIGHT_TAG = new Pose2d(60.25 - TAG_BOT_OFFSET, 29.41, 0);

        Pose2d RED_ALLIANCE_LEFT_TAG = new Pose2d(60.25 - TAG_BOT_OFFSET, -29.41, 0);
        Pose2d RED_ALLIANCE_CENTER_TAG = new Pose2d(60.25 - TAG_BOT_OFFSET, -35.41, 0);
        Pose2d RED_ALLIANCE_RIGHT_TAG = new Pose2d(60.25 - TAG_BOT_OFFSET, -41.41, 0);

        Pose2d BLUE_BOARD_CENTER_TAG = new Pose2d(58 - HALF_ROBO_LEN, 36, 0);


        Pose2d RED_RIGHT_SPIKE = new Pose2d(37,-30, Math.toRadians(0) );



        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Action TrajectoryBlueNearToLeftPark = myBot.getDrive().actionBuilder(BLUE_NEAR_START_POSE)
                .splineToLinearHeading(BLUE_LEFT_PARK, 0)
                .build();

        Action TrajectoryRedNearToRightPark = myBot.getDrive().actionBuilder(RED_NEAR_START_POSE)
                .splineToLinearHeading(RED_RIGHT_PARK, 0)
                .build();

        // 1. Start position: blue near; Game Element location: Center.

        Action TrajectoryBlueNearGeCenterWithScoring = myBot.getDrive().actionBuilder(BLUE_NEAR_START_POSE)
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

        // Start position blue far; GE: Center
        Action TrajectoryBlueFarGeCenterWithScoring = myBot.getDrive().actionBuilder(BLUE_FAR_START_POSE)
                .lineToY(BLUE_FAR_CENTER_SPIKE.position.y)
                .waitSeconds(1)
                .setReversed(true)
                .lineToY(BLUE_FAR_CENTER_SPIKE.position.y + 6)
                .setReversed(false)
                .strafeTo(new Vector2d(-48, BLUE_FAR_CENTER_SPIKE.position.y + 6))
                .turnTo(0)
                .lineToX(BLUE_BOARD_CENTER_TAG.position.x)
                .strafeTo(BLUE_BOARD_CENTER_TAG.position)
                .waitSeconds(2)
                .strafeTo(BLUE_CENTER_PARK.position)
                .turn(Math.PI +1e-6)
                .build();

        //Start Position blue near; GE: Center
        Action TrajectoryBlueNearGeCenter = myBot.getDrive().actionBuilder(BLUE_NEAR_START_POSE)
                .lineToY(BLUE_NEAR_CENTER_SPIKE.position.y)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(BLUE_LEFT_PARK_REVERSE, 0)
                .waitSeconds(2)
                .build();

        //Start Position blue near; GE: Left
        Action TrajectoryBlueNearGeLeft = myBot.getDrive().actionBuilder(BLUE_NEAR_START_POSE)
                .lineToY(BLUE_NEAR_CENTER_SPIKE.position.y)
                .turn(Math.PI/2)
                .strafeTo(new Vector2d(BLUE_NEAR_START_POSE.position.x, 60))
                .strafeTo(new Vector2d(48, 60))
                .waitSeconds(2)
                .build();

        //Start Position blue near; GE: right
        Action TrajectoryBlueNearGeRight = myBot.getDrive().actionBuilder(BLUE_NEAR_START_POSE)
                .strafeTo(new Vector2d(14, BLUE_NEAR_CENTER_SPIKE.position.y))
//                .lineToY(BLUE_NEAR_CENTER_SPIKE.position.y)
                .turn(-Math.PI/2)
                .lineToX(2+HALF_ROBO_LEN)
                .waitSeconds(1)
                .setReversed(true)
                .strafeTo(new Vector2d(48, 60))
                .waitSeconds(2)
                .build();

        Action TrajectoryBlueFarToPark = myBot.getDrive().actionBuilder(BLUE_FAR_START_POSE)
                .strafeTo(new Vector2d(-36, 12))
                .turn(Math.PI/2)
                .strafeTo(BLUE_CENTER_PARK.position)
                .build();


        Action TrajectoryRedFarToPark = myBot.getDrive().actionBuilder(RED_FAR_START_POSE)
                .strafeTo(new Vector2d(-36, -12))
                .turn(-Math.PI/2)
                .strafeTo(RED_CENTER_PARK.position)
                .build();

        Action v3RedNearGeCenterPark = myBot.getDrive().actionBuilder(RED_NEAR_START_POSE)
                .strafeTo(RED_NEAR_CENTER_SPIKE.position)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(12, -40))
                .turn(Math.toRadians(-120))
                .strafeTo(RED_RIGHT_PARK.position)
                .build();


        Action v3RedNearGeRightPark = myBot.getDrive().actionBuilder(RED_NEAR_START_POSE)
                .strafeToLinearHeading(RED_NEAR_RIGHT_SPIKE.position, 0)
                .strafeTo(new Vector2d(RED_NEAR_RIGHT_SPIKE.position.x - 3, RED_NEAR_RIGHT_SPIKE.position.y))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(12, -48))
                .strafeTo(RED_RIGHT_PARK.position) // Move to parking position
                .turn(Math.toRadians(-30)) // Turn a little to get head start for the manual mode.
                .build();


        Action v3RedNearGeLeftPark = myBot.getDrive().actionBuilder(RED_NEAR_START_POSE)
                .strafeToLinearHeading(RED_NEAR_LEFT_SPIKE.position, Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_NEAR_LEFT_SPIKE.position.x + 3, RED_NEAR_LEFT_SPIKE.position.y))
                .strafeToLinearHeading(RED_RIGHT_PARK.position, Math.toRadians(-30)) // Move to parking position
                .build();


        Action v3RedFarGeCenterPark = myBot.getDrive().actionBuilder(RED_FAR_START_POSE)
                .strafeTo(RED_FAR_CENTER_SPIKE.position)
                .strafeTo(new Vector2d(RED_FAR_CENTER_SPIKE.position.x, RED_FAR_CENTER_SPIKE.position.y - 10)) // come back
                .strafeTo(new Vector2d(-52, RED_FAR_CENTER_SPIKE.position.y - 10)) // slide left
                .strafeTo(new Vector2d(-52, -12)) // move fwd
                .turn(-Math.PI/2)
                .strafeTo(RED_CENTER_PARK.position) // park
                .build();


        Action v3RedFarGeLeftPark = myBot.getDrive().actionBuilder(RED_FAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(RED_FAR_LEFT_SPIKE.position.x, RED_FAR_LEFT_SPIKE.position.y), Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_FAR_LEFT_SPIKE.position.x + 5, RED_FAR_LEFT_SPIKE.position.y))
                .strafeTo(new Vector2d(RED_FAR_LEFT_SPIKE.position.x + 5, -12))
                .turn(Math.PI+1e-6)
                .strafeTo(RED_CENTER_PARK.position) // park
                .build();

        Action v3RedfarGeRightPark = myBot.getDrive().actionBuilder(RED_FAR_START_POSE)
                .splineToLinearHeading(RED_FAR_RIGHT_SPIKE, 0)
//                .strafeToLinearHeading(new Vector2d(RED_FAR_RIGHT_SPIKE.position.x, RED_FAR_RIGHT_SPIKE.position.y), Math.toRadians(0))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_FAR_RIGHT_SPIKE.position.x - 5, RED_FAR_RIGHT_SPIKE.position.y))
                .strafeTo(new Vector2d(RED_FAR_RIGHT_SPIKE.position.x - 5, -12))
                .strafeTo(RED_CENTER_PARK.position) // park
                .build();

        Action v3BlueFarGeCenterPark = myBot.getDrive().actionBuilder(BLUE_FAR_START_POSE)
                .strafeTo(BLUE_FAR_CENTER_SPIKE.position)
                .waitSeconds(1)
                .strafeTo(new Vector2d(BLUE_FAR_CENTER_SPIKE.position.x, BLUE_FAR_CENTER_SPIKE.position.y + 10)) // come back
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-52, BLUE_FAR_CENTER_SPIKE.position.y + 10)) // slide left
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-52, 12)) // move fwd
                .turn(Math.PI/2)
                .strafeTo(BLUE_CENTER_PARK.position) // park
                .build();

        Action v3BlueFarGeRightPark = myBot.getDrive().actionBuilder(BLUE_FAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(BLUE_FAR_RIGHT_SPIKE.position.x, BLUE_FAR_RIGHT_SPIKE.position.y), -Math.PI)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_FAR_RIGHT_SPIKE.position.x + 2, BLUE_FAR_RIGHT_SPIKE.position.y))
                .strafeTo(new Vector2d(BLUE_FAR_RIGHT_SPIKE.position.x + 2, 12))
                .turn(-(Math.PI+1e-6))
                .strafeTo(BLUE_CENTER_PARK.position) // park
                .build();

        Action v3BlueFarGeLeftPark = myBot.getDrive().actionBuilder(BLUE_FAR_START_POSE)
                .splineToLinearHeading(BLUE_FAR_LEFT_SPIKE, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_FAR_LEFT_SPIKE.position.x - 5, BLUE_FAR_LEFT_SPIKE.position.y))
                .strafeTo(new Vector2d(BLUE_FAR_LEFT_SPIKE.position.x - 5, 12))
                .strafeTo(BLUE_CENTER_PARK.position) // park
                .build();

        Action v3BlueNearGeCenterPark = myBot.getDrive().actionBuilder(BLUE_NEAR_START_POSE)
                .strafeTo(BLUE_NEAR_CENTER_SPIKE.position)
                .waitSeconds(1.0)
                .strafeTo(new Vector2d(12, 40))
                .turn(Math.toRadians(120))
                .splineToLinearHeading(BLUE_LEFT_PARK, 0)
                .build();

        Action v3BlueNearGeLeftPark = myBot.getDrive().actionBuilder(BLUE_NEAR_START_POSE)
                .strafeToLinearHeading(BLUE_NEAR_LEFT_SPIKE.position, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_NEAR_LEFT_SPIKE.position.x - 2, BLUE_NEAR_LEFT_SPIKE.position.y))
                .strafeTo(new Vector2d(12, 58))
                .strafeTo(BLUE_LEFT_PARK.position) // Move to parking position
                .build();

        Action v3BlueNearGeRightPark = myBot.getDrive().actionBuilder(BLUE_NEAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(BLUE_NEAR_RIGHT_SPIKE.position.x + 5, BLUE_NEAR_RIGHT_SPIKE.position.y), Math.PI)
                .strafeTo(new Vector2d(BLUE_NEAR_RIGHT_SPIKE.position.x -2, BLUE_NEAR_RIGHT_SPIKE.position.y))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_NEAR_RIGHT_SPIKE.position.x + 0, BLUE_NEAR_RIGHT_SPIKE.position.y))
                .strafeToLinearHeading(BLUE_LEFT_PARK.position, Math.toRadians(0)) // Move to parking position
                .build();

        Action v4RedNearGeCenterScorePark = myBot.getDrive().actionBuilder(RED_NEAR_START_POSE)
                .strafeTo(RED_NEAR_CENTER_SPIKE.position)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(12, -40))
                .turn(Math.toRadians(-90))
                .strafeTo(RED_ALLIANCE_CENTER_TAG.position)
                .build();
        double RED_NEAR_LEFT_SPIKE_Y = -30;

        Action v4RedNearGeLeftScorePark = myBot.getDrive().actionBuilder(RED_NEAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(RED_NEAR_LEFT_SPIKE.position.x + 5, RED_NEAR_LEFT_SPIKE.position.y), Math.toRadians(180))
                .strafeTo(new Vector2d(RED_NEAR_LEFT_SPIKE.position.x - 3, RED_NEAR_LEFT_SPIKE_Y))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_NEAR_LEFT_SPIKE.position.x + 3, RED_NEAR_LEFT_SPIKE.position.y))
                .strafeToLinearHeading(RED_ALLIANCE_LEFT_TAG.position, 0-1e-6)
                .build();

        double RED_NEAR_RIGHT_SPIKE_BACK_X = 4;

        Action v4RedNearGeRightPark = myBot.getDrive().actionBuilder(RED_NEAR_START_POSE)
                .strafeToLinearHeading(RED_NEAR_RIGHT_SPIKE.position, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_NEAR_RIGHT_SPIKE.position.x - RED_NEAR_RIGHT_SPIKE_BACK_X, RED_NEAR_RIGHT_SPIKE.position.y))
                .strafeTo(new Vector2d(12, -58))
                .strafeToLinearHeading(RED_ALLIANCE_RIGHT_TAG.position, 0)
                .build();


        Action v4RedFarGeCenterScore = myBot.getDrive().actionBuilder(RED_FAR_START_POSE)
                .strafeTo(RED_FAR_CENTER_SPIKE.position)
                .waitSeconds(1)
                .strafeTo(new Vector2d(RED_FAR_CENTER_SPIKE.position.x, RED_FAR_CENTER_SPIKE.position.y - 10)) // come back
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-52, RED_FAR_CENTER_SPIKE.position.y - 10)) // slide left
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-52, -12)) // move fwd
                .turn(-Math.PI/2)
                .strafeToLinearHeading(new Vector2d(33, -12), 0 )
                .strafeToLinearHeading(RED_ALLIANCE_CENTER_TAG.position, 0 )
                .build();


        Action v4RedFarGeLeftScore = myBot.getDrive().actionBuilder(RED_FAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(-57, RED_FAR_LEFT_SPIKE.position.y), Math.toRadians(0))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-60, RED_FAR_LEFT_SPIKE.position.y))
                .strafeTo(new Vector2d(-60, -12))
                .strafeToLinearHeading(new Vector2d(10, -12), 0 )
                .strafeToLinearHeading(RED_ALLIANCE_LEFT_TAG.position, 0 )
                .build();

        Action v4RedfarGeRightScore = myBot.getDrive().actionBuilder(RED_FAR_START_POSE)
                .splineToLinearHeading(RED_FAR_RIGHT_SPIKE, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_FAR_RIGHT_SPIKE.position.x - 5, RED_FAR_RIGHT_SPIKE.position.y))
                .strafeTo(new Vector2d(RED_FAR_RIGHT_SPIKE.position.x - 5, -8))
                .strafeToLinearHeading(new Vector2d(RED_ALLIANCE_RIGHT_TAG.position.x, -12), 0 )
                .strafeToLinearHeading(RED_ALLIANCE_RIGHT_TAG.position, 0 )
                .build();

        Action v4RedNearCloseOut = myBot.getDrive().actionBuilder(RED_ALLIANCE_CENTER_TAG)
                .strafeTo(RED_RIGHT_PARK.position)
                .build();

        Action v4RedFarCloseOut = myBot.getDrive().actionBuilder(RED_ALLIANCE_CENTER_TAG)
                .strafeTo(RED_CENTER_PARK.position)
                .build();

        Pose2d START = new Pose2d(11.8, 61.7, Math.toRadians(90));
        Action trajectoryAction1 = myBot.getDrive().actionBuilder(START)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3)
                .build();
        Action trajectoryAction2 = myBot.getDrive().actionBuilder(START)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3)
                .build();
        Action trajectoryAction3 = myBot.getDrive().actionBuilder(START)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3)
                .build();

        Action v4BlueNearGeCenterScore = myBot.getDrive().actionBuilder(BLUE_NEAR_START_POSE)
                .strafeTo(BLUE_NEAR_CENTER_SPIKE.position)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(12, 40))
                .turn(Math.toRadians(90))
                .strafeTo(BLUE_ALLIANCE_CENTER_TAG.position)
                .build();

        Action v4BlueNearGeLeftScore = myBot.getDrive().actionBuilder(BLUE_NEAR_START_POSE)
                .strafeToLinearHeading(BLUE_NEAR_LEFT_SPIKE.position, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_NEAR_LEFT_SPIKE.position.x - 2, BLUE_NEAR_LEFT_SPIKE.position.y))
                .strafeTo(new Vector2d(12, 54))
                .strafeToLinearHeading(BLUE_ALLIANCE_LEFT_TAG.position, 0)
                .build();

        Action v4BlueNearGeRightScore = myBot.getDrive().actionBuilder(BLUE_NEAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(BLUE_NEAR_RIGHT_SPIKE.position.x + 5, BLUE_NEAR_RIGHT_SPIKE.position.y), Math.PI)
                .strafeTo(new Vector2d(BLUE_NEAR_RIGHT_SPIKE.position.x -2, BLUE_NEAR_RIGHT_SPIKE.position.y))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_NEAR_RIGHT_SPIKE.position.x + 3, BLUE_NEAR_RIGHT_SPIKE.position.y))
                .turn(-Math.PI+1e-6)
                .strafeToLinearHeading(BLUE_ALLIANCE_RIGHT_TAG.position, 0)
                .build();

        Action v4BlueFarGeCenterScore = myBot.getDrive().actionBuilder(BLUE_FAR_START_POSE)
                .strafeTo(BLUE_FAR_CENTER_SPIKE.position)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_FAR_CENTER_SPIKE.position.x, BLUE_FAR_CENTER_SPIKE.position.y + 10)) // come back
                .strafeTo(new Vector2d(-52, BLUE_FAR_CENTER_SPIKE.position.y + 10))
                .strafeTo(new Vector2d(-52, 12)) // move fwd
                .turn(Math.PI/2)
                .strafeToLinearHeading(new Vector2d(28, 12), 0 )
                .strafeToLinearHeading(BLUE_ALLIANCE_CENTER_TAG.position, 0 )
                .build();

        Action v4BlueFarGeLeftScore = myBot.getDrive().actionBuilder(BLUE_FAR_START_POSE)
                .splineToLinearHeading(BLUE_FAR_LEFT_SPIKE, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_FAR_LEFT_SPIKE.position.x - 5, BLUE_FAR_LEFT_SPIKE.position.y))
                .strafeTo(new Vector2d(BLUE_FAR_LEFT_SPIKE.position.x - 5, 10))
                .strafeToLinearHeading(new Vector2d(35, 10), 0 )
                .strafeToLinearHeading(BLUE_ALLIANCE_LEFT_TAG.position, 0 )
                .build();

        Action v4BluefarGeRightScore = myBot.getDrive().actionBuilder(BLUE_FAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(BLUE_FAR_RIGHT_SPIKE.position.x, BLUE_FAR_RIGHT_SPIKE.position.y), -Math.PI)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(BLUE_FAR_RIGHT_SPIKE.position.x + 2, BLUE_FAR_RIGHT_SPIKE.position.y))
                .strafeTo(new Vector2d(BLUE_FAR_RIGHT_SPIKE.position.x + 2, 10))
                .turn(-(Math.PI+1e-6))
                .strafeToLinearHeading(new Vector2d(20, 10), 0 )
                .strafeToLinearHeading(BLUE_ALLIANCE_RIGHT_TAG.position, 0 )
                .build();

        Action v4RedFarGeLeftScore0 = myBot.getDrive().actionBuilder(RED_FAR_START_POSE)
                .strafeToLinearHeading(new Vector2d(RED_FAR_LEFT_SPIKE.position.x-2, RED_FAR_LEFT_SPIKE.position.y), Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_FAR_LEFT_SPIKE.position.x + 0, RED_FAR_LEFT_SPIKE.position.y))
                .strafeTo(new Vector2d(RED_FAR_LEFT_SPIKE.position.x + 0, -12))
                .turn(Math.PI+1e-6)
                .strafeToLinearHeading(new Vector2d(20, -12), 0 )
                .strafeToLinearHeading(RED_ALLIANCE_LEFT_TAG.position, 0 )
                .build();


        Action v4RedfarGeRightScore2 = myBot.getDrive().actionBuilder(RED_FAR_START_POSE)
                .strafeTo(new Vector2d(RED_FAR_START_POSE.position.x - 6, RED_FAR_START_POSE.position.y+3))
                .splineToLinearHeading(RED_FAR_RIGHT_SPIKE, 0)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(RED_FAR_RIGHT_SPIKE.position.x - 8, RED_FAR_RIGHT_SPIKE.position.y))
                .strafeTo(new Vector2d(RED_FAR_RIGHT_SPIKE.position.x - 8, -10))
                .strafeToLinearHeading(new Vector2d(RED_ALLIANCE_RIGHT_TAG.position.x, -12), 0 )
                .strafeToLinearHeading(RED_ALLIANCE_RIGHT_TAG.position, 0 )
                .build();

        // ==================================
        // This is what gets shown on the UI
        // ==================================
        myBot.runAction(v4RedfarGeRightScore2);


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}