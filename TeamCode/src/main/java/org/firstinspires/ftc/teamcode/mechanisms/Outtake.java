package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
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

        public double ELBOW_HOME_POSITION = 0.8;
        public double ELBOW_SCORING_POSITION = 0.4;

        public double WRIST_HOME_POSITION = 0.0;
        public double WRIST_SCORING_POSITION = 0.56;

        public double BOX_CLOSE_POSITION = 1.0;
        public double BOX_SCORING_POSITION = 0.5;

        public int ACTUATOR_ENCODER_COUNT = 3500;
        public double LINEAR_ACTUATOR_POWER = 0.4;
        public double LINEAR_ACTUATOR_TIMEOUT_SEC = 12;
    }

    private DcMotorEx linearActLeft = null;
    private DcMotorEx linearActRight = null;

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
        linearActRight.setDirection(DcMotorSimple.Direction.REVERSE);

        LinearActLeftEncoder = new OverflowEncoder(new RawEncoder(linearActLeft));
        LinearActRightEncoder = new OverflowEncoder(new RawEncoder(linearActRight));

        wrist.setPosition(PARAMS.WRIST_HOME_POSITION);
        boxLever.setPosition(PARAMS.ELBOW_HOME_POSITION);
        box.setPosition(PARAMS.BOX_CLOSE_POSITION);
        
//        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }


    public class CloseBox implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            box.setPosition(PARAMS.BOX_CLOSE_POSITION);
            return false;
        }
    }
    public Action closeBox() {
        return new Outtake.CloseBox();
    }

    public class OpenBox implements Action {
        private boolean initialized = false;
        private double beginTs = -1;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double duration;
            if (!initialized){
                beginTs = Actions.now();
                duration = 0;
                initialized = true;
            }
            duration = Actions.now() - beginTs;
            box.setPosition(PARAMS.BOX_SCORING_POSITION);
            packet.put("boxPosition ", box.getPosition());

            if (duration < 2.0){
                return true;
            }
            return false;
        }
    }
    public Action openBox() {
        return new Outtake.OpenBox();
    }

    public class WristUp implements Action {
        private boolean initialized = false;
        private double beginTs = -1;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            double duration;
            if (!initialized){
                beginTs = Actions.now();
                duration = 0;
                initialized = true;
            }
            duration = Actions.now() - beginTs;
            wrist.setPosition(PARAMS.WRIST_SCORING_POSITION);
            packet.put("wristPosition ", wrist.getPosition());

            if (duration < 2.0){
                return true;
            }

            return false;
        }
    }
    public Action moveWristUp() {
        return new Outtake.WristUp();
    }

    public class WristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(PARAMS.WRIST_HOME_POSITION);
            packet.put("wristPosition ", wrist.getPosition());
            return false;
        }
    }
    public Action moveWristDown() {
        return new Outtake.WristDown();
    }

    public class ElbowHome implements Action {
        public boolean run(@NonNull TelemetryPacket packet) {

            boxLever.setPosition(PARAMS.ELBOW_HOME_POSITION);
            return false;
        }
    }
    public Action moveElbowHomePosition() {
        return new Outtake.ElbowHome();
    }


    public class ElbowScore implements Action {
        private boolean initialized = false;
        private double beginTs = -1;

        public boolean run(@NonNull TelemetryPacket packet) {
            double duration;
            if (!initialized){
                beginTs = Actions.now();
                duration = 0;
                initialized = true;
            }
            duration = Actions.now() - beginTs;
            boxLever.setPosition(PARAMS.ELBOW_SCORING_POSITION);
            packet.put("boxLeverPosition ", boxLever.getPosition());

            if (duration < 2.0){
                return true;
            }

            return false;
        }
    }
    public Action moveElbowToScorePosition() {
        return new Outtake.ElbowScore();
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

    public void setElbowPosition(double position){
        boxLever.setPosition(position);
    }
    public void setWristPosition(double position){
        wrist.setPosition(position);
    }

    public double getWristPosition() {
        return wrist.getPosition();
    }

}
