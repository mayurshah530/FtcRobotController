package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumEncodersMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class Outtake {
    public static class Params {

        public double ELBOW_HOME_POSITION = 0.0;
        public double ELBOW_SCORING_POSITION = 1.0;
        public double WRIST_HOME_POSITION = 0.0;
        public double WRIST_SCORING_POSITION = 1.0;

        public double BOX_OPEN_POSITOIN = 1.0;
        public double BOX_CLOSE_POSITOIN = 0.0;
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
            box.setPosition(PARAMS.BOX_CLOSE_POSITOIN);
            return false;
        }
    }
    public Action closeBox() {
        return new Outtake.CloseBox();
    }

    public class OpenBox implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            box.setPosition(PARAMS.BOX_OPEN_POSITOIN);
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
