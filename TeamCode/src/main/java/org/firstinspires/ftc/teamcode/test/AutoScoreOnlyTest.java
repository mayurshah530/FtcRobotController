package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;

@Config
@Autonomous(name = "Pixel Score Only", group = "RoadRunner 1.0")
public class AutoScoreOnlyTest extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    public static boolean USE_ACTION_CONSTRUCT = true;

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Outtake outtake = new Outtake(hardwareMap);

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        if (USE_ACTION_CONSTRUCT){

            // move linear actuator forward. (how much? how long?)
            // tilt elbow up
            // turn wrist 180
            // move wheels forward, look for signs of deceleration and zero velocity
            // open box
            // close box + move wrist in
            // put boxlever down + move to park position
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    outtake.moveBoxLeverUp(),
                                    outtake.actuatorExpand(Outtake.PARAMS.ACTUATOR_ENCODER_COUNT)
                            ),
                            outtake.moveWristOut(),
                            outtake.actuatorExpand(50),
                            outtake.waitSec(0.5),
                            outtake.openBox(),
                            new ParallelAction(
                                    outtake.closeBox(),
                                    outtake.moveWristIn()
                            )
//                       ,outtake.moveBoxLeverDown()
                    )
            );

        } else {

            // alternate: try to do it without the Action construct.
            boolean wristInScoringPosition = false;
            int initLeftActuatorPos = outtake.LinearActLeftEncoder.getPositionAndVelocity().position;
            int initRightActuatorPos = outtake.LinearActRightEncoder.getPositionAndVelocity().position;
            outtake.setBoxLeverPosition(Outtake.PARAMS.BOX_LEVER_SCORING_POSITION);

            boolean leftDone = false;
            boolean rightDone = false;
            runtime.reset();

            while ((!leftDone || !rightDone) && (runtime.seconds() < 10.0)) {
                outtake.linearActLeft.setPower(Outtake.PARAMS.LINEAR_ACTUATOR_POWER);
                outtake.linearActRight.setPower(Outtake.PARAMS.LINEAR_ACTUATOR_POWER);

                int leftActuatorPos = outtake.LinearActLeftEncoder.getPositionAndVelocity().position;
                int rightActuatorPos = outtake.LinearActRightEncoder.getPositionAndVelocity().position;
                int leftDelta = leftActuatorPos - initLeftActuatorPos;
                int rightDelta = rightActuatorPos - initRightActuatorPos;
                telemetry.addData("leftDelta ", leftDelta);
                telemetry.addData("rightDelta ", rightDelta);

                leftDone = Math.abs(leftDelta) >= Math.abs(Outtake.PARAMS.ACTUATOR_ENCODER_COUNT);
                rightDone = Math.abs(rightDelta) >= Math.abs(Outtake.PARAMS.ACTUATOR_ENCODER_COUNT);

                if (leftDone) {
                    outtake.linearActLeft.setPower(0.0);
                }
                if (rightDone) {
                    outtake.linearActRight.setPower(0.0);
                }

                if (!wristInScoringPosition &&
                        Math.abs(leftDelta) >= Outtake.PARAMS.ACTUATOR_ENCODER_COUNT *8/10 &&
                        Math.abs(rightDelta) >= Outtake.PARAMS.ACTUATOR_ENCODER_COUNT  *8/10) {
                    outtake.setWristPosition(Outtake.PARAMS.WRIST_SCORING_POSITION);
                    wristInScoringPosition = true;
                }
                sleep(20);
            }

            outtake.linearActLeft.setPower(0.0);
            outtake.linearActRight.setPower(0.0);

            sleep(500);
            outtake.setBoxPosition(Outtake.PARAMS.BOX_SCORING_POSITION);
            sleep(200);
            outtake.setBoxPosition(Outtake.PARAMS.BOX_CLOSE_POSITION);
            outtake.setWristPosition(Outtake.PARAMS.WRIST_HOME_POSITION);
            sleep(200);
//                outtake.setBoxLeverPosition(Outtake.PARAMS.BOX_LEVER_HOME_POSITION);

        }


    } // runOpMode
}