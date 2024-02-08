package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;


@Config
public final class Outtake {
    public static class Params {

        public double BOX_LEVER_HOME_POSITION = 0.8;
        public double BOX_LEVER_SCORING_POSITION = 0.4;

        public double BOX_LEVER_DURATION = 2.0;

        public double WRIST_HOME_POSITION = 0.0;
        public double WRIST_SCORING_POSITION = 0.56;
        public double WRIST_DURATION = 2.0;

        public double BOX_CLOSE_POSITION = 1.0;
        public double BOX_SCORING_POSITION = 0.5;
        public double BOX_DURATION = 2.0;

        public int ACTUATOR_ENCODER_COUNT = 3500;
        public int ACTUATOR_ENCODER_COUNT_2 = 1000;
        public double LINEAR_ACTUATOR_POWER = 0.8;
        public double LINEAR_ACTUATOR_TIMEOUT_SEC = 12;
    }

    public DcMotorEx linearActLeft = null;
    public DcMotorEx linearActRight = null;

    private Servo wrist = null;
    private Servo boxLever = null;
    private Servo box = null;

    public Encoder LinearActLeftEncoder = null;
    public Encoder LinearActRightEncoder = null;

    public static Params PARAMS = new Params();


    public Outtake(HardwareMap hardwareMap) {

        linearActLeft = hardwareMap.get(DcMotorEx.class, "linear_act_left");
        linearActRight = hardwareMap.get(DcMotorEx.class, "linear_act_right");

        wrist = hardwareMap.get(Servo.class,"wrist");
        boxLever = hardwareMap.get(Servo.class,"box_lever");
        box = hardwareMap.get(Servo.class, "box");

        linearActLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        linearActRight.setDirection(DcMotorSimple.Direction.FORWARD);

        LinearActLeftEncoder = new OverflowEncoder(new RawEncoder(linearActLeft));
        LinearActRightEncoder = new OverflowEncoder(new RawEncoder(linearActRight));

        wrist.setPosition(PARAMS.WRIST_HOME_POSITION);
        boxLever.setPosition(PARAMS.BOX_LEVER_HOME_POSITION);
        box.setPosition(PARAMS.BOX_CLOSE_POSITION);

//        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public class MoveBoxLever implements Action {
        private boolean initialized = false;
        private double beginTs = -1;
        private final double BOX_LEVER_POSITION;

        MoveBoxLever(double boxLeverPosition){
            BOX_LEVER_POSITION = boxLeverPosition;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double duration;
            if (!initialized){
                boxLever.setPosition(BOX_LEVER_POSITION);
                beginTs = Actions.now();
                initialized = true;
            }
            duration = Actions.now() - beginTs;
            packet.put("boxLeverPosition ", boxLever.getPosition());

            if (duration < PARAMS.BOX_LEVER_DURATION){
                return true;
            }
            return false;
        }
    }

    public Action moveBoxLeverUp(){
        return new MoveBoxLever(PARAMS.BOX_LEVER_SCORING_POSITION);
    }
    public Action moveBoxLeverDown(){
        return new MoveBoxLever(PARAMS.BOX_LEVER_HOME_POSITION);
    }

    public class OpenOrCloseBox implements Action {
        private boolean initialized = false;
        private double beginTs = -1;
        private final double BOX_POSITION;

        OpenOrCloseBox(double boxPosition){
            BOX_POSITION = boxPosition;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double duration;
            if (!initialized){
                box.setPosition(BOX_POSITION);
                beginTs = Actions.now();
                initialized = true;
            }
            duration = Actions.now() - beginTs;
            packet.put("boxPosition ", box.getPosition());

            return duration < 2.0;
        }
    }

    public Action closeBox() {
        return new Outtake.OpenOrCloseBox(PARAMS.BOX_CLOSE_POSITION);
    }
    public Action openBox() {
        return new Outtake.OpenOrCloseBox(PARAMS.BOX_SCORING_POSITION);
    }

    public class MoveWrist implements Action {
        private boolean initialized = false;
        private double beginTs = -1;
        private final double WRIST_POSITION;

        MoveWrist(double wristPosition){
            WRIST_POSITION = wristPosition;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double duration;
            if (!initialized){
                beginTs = Actions.now();
                initialized = true;
            }
            duration = Actions.now() - beginTs;
            wrist.setPosition(WRIST_POSITION);
            packet.put("wristPosition ", wrist.getPosition());

            if (duration < PARAMS.WRIST_DURATION){
                return true;
            }
            return false;
        }
    }

    public Action moveWristOut() {
        return new Outtake.MoveWrist(PARAMS.WRIST_SCORING_POSITION);
    }
    public Action moveWristIn() {
        return new Outtake.MoveWrist(PARAMS.WRIST_HOME_POSITION);
    }

    public class LinearActuatorMotion implements Action {
        public final int MOVE_COUNT;
        private boolean initialized = false;

        private int initLeftActuatorPos, initRightActuatorPos;
        private double beginTs = -1;

        public LinearActuatorMotion(int moveCount){
            MOVE_COUNT = moveCount;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double duration;
            if (!initialized){
                initLeftActuatorPos = LinearActLeftEncoder.getPositionAndVelocity().position;
                initRightActuatorPos = LinearActRightEncoder.getPositionAndVelocity().position;
                beginTs = Actions.now();
                duration = 0;

                int sign = MOVE_COUNT > 0 ? 1 : -1;
                linearActLeft.setPower(sign * PARAMS.LINEAR_ACTUATOR_POWER);
                linearActRight.setPower(sign * PARAMS.LINEAR_ACTUATOR_POWER);
                initialized = true;
            }
            duration = Actions.now() - beginTs;
            int leftActuatorPos = LinearActLeftEncoder.getPositionAndVelocity().position;
            int rightActuatorPos = LinearActRightEncoder.getPositionAndVelocity().position;
            int leftDelta = leftActuatorPos - initLeftActuatorPos;
            int rightDelta = rightActuatorPos - initRightActuatorPos;
            telemetryPacket.put("leftActuatorPos ", leftActuatorPos);
            telemetryPacket.put("rightActuatorPos ", rightActuatorPos);
            telemetryPacket.put("leftDelta ", leftDelta);
            telemetryPacket.put("rightDelta ", rightDelta);

            boolean leftDone = Math.abs(leftDelta) >= Math.abs(MOVE_COUNT);
            boolean rightDone = Math.abs(rightDelta) >= Math.abs(MOVE_COUNT);
            if (leftDone){
                linearActLeft.setPower(0.0);
            }
            if (rightDone){
                linearActRight.setPower(0.0);
            }

            if ((leftDone && rightDone) || duration >= PARAMS.LINEAR_ACTUATOR_TIMEOUT_SEC){
                linearActLeft.setPower(0.0);
                linearActRight.setPower(0.0);
                return false;
            }
            return true;
        }
    }

    public Action actuatorExpand(int encoderCount){
        return new LinearActuatorMotion(encoderCount);
    }

    public Action actuatorRetract(int encoderCount){
        return new LinearActuatorMotion(-encoderCount); // -ve sign
    }

    public static class Wait implements Action {
        private boolean initialized = false;
        private double beginTs = -1;
        private final double WAIT_SEC;

        Wait(double waitSec){
            WAIT_SEC = waitSec;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double duration;
            if (!initialized){
                beginTs = Actions.now();
                initialized = true;
            }
            duration = Actions.now() - beginTs;

            return duration < WAIT_SEC;
        }
    }

    public Action waitSec(double waitSec) {
        return new Wait(waitSec);
    }

    public void setBoxLeverPosition(double position){
        boxLever.setPosition(position);
    }
    public void setWristPosition(double position){
        wrist.setPosition(position);
    }
    public void setBoxPosition(double position) {box.setPosition(position);}

    public double getBoxLeverPosition(){return boxLever.getPosition();}
    public double getWristPosition() {
        return wrist.getPosition();
    }
    public double getBoxPosition(){return box.getPosition();}

    public Action pixelDropAction =
            new SequentialAction(
                    new ParallelAction(
                            moveBoxLeverUp(),
                            actuatorExpand(PARAMS.ACTUATOR_ENCODER_COUNT)
                    ),
                    moveWristOut(),
                    actuatorExpand(PARAMS.ACTUATOR_ENCODER_COUNT_2),
                    openBox(),
                    new ParallelAction(
                            closeBox(),
                            moveWristIn()
                    )
            );

}
