package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public final class Outtake {
    public static class Params {

        public double ELBOW_HOME_POSITION = 0.0;
        public double ELBOW_SCORING_POSITION = 1.0;
        public double WRIST_HOME_POSITION = 0.0;
        public double WRIST_SCORING_POSITION = 1.0;

        public double BOX_OPEN_POSITION = 1.0;
        public double BOX_CLOSE_POSITION = 0.0;
    }

    private Servo wrist = null;
    private Servo elbow = null;
    private Servo box = null;

    public static Params PARAMS = new Params();

    public Outtake(HardwareMap hardwareMap) {

        wrist = hardwareMap.get(Servo.class,"wrist");
        elbow = hardwareMap.get(Servo.class,"box_lever");
        box = hardwareMap.get(Servo.class, "box");

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
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
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            box.setPosition(PARAMS.BOX_OPEN_POSITION);
            return false;
        }
    }
    public Action openBox() {
        return new Outtake.OpenBox();
    }

    public class WristUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(PARAMS.WRIST_SCORING_POSITION);
            return false;
        }
    }
    public Action wristUp() {
        return new Outtake.WristUp();
    }

    public class WristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(PARAMS.WRIST_HOME_POSITION);
            return false;
        }
    }
    public Action wristDown() {
        return new Outtake.WristDown();
    }

    public class ElbowHome implements Action {
        public boolean run(@NonNull TelemetryPacket packet) {
            elbow.setPosition(PARAMS.ELBOW_HOME_POSITION);
            return false;
        }
    }
    public Action elbowHome() {
        return new Outtake.ElbowHome();
    }


    public class ElbowScore implements Action {
        public boolean run(@NonNull TelemetryPacket packet) {
            elbow.setPosition(PARAMS.ELBOW_SCORING_POSITION);
            return false;
        }
    }
    public Action elbowScore() {
        return new Outtake.ElbowScore();
    }


    public void elbowPosition(double position){
        elbow.setPosition(position);
    }
    public void wristPosition(double position){
        wrist.setPosition(position);
    }

}
