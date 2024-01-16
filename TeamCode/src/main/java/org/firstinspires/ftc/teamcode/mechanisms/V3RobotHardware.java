/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class V3RobotHardware {

    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor linearActLeft = null;
    private DcMotor linearActRight = null;

    private DcMotor intake = null;
    private DcMotor intake_back = null;
    private CRServo liftLeft = null;
    private CRServo liftRight = null;

    private CRServo wrist = null;

    private Servo plane_launcher= null;
    private Servo boxLever = null;


    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double LIFT_UP_POWER = 0.9;

    public static final double LIFT_DOWN_POWER = -0.5 ;
    public static final double LIFT_ZERO_POWER = 0.5;

    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 312;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
//
//    public static double WHEEL_RADIUS = 1.88976; // in
//    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
//    static final double     COUNTS_PER_INCH         = (TICKS_PER_REV * GEAR_RATIO) /
//            (WHEEL_RADIUS * 2 * Math.PI);
//

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public V3RobotHardware(OpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");

        linearActLeft   = myOpMode.hardwareMap.get(DcMotor.class, "linear_act_left");
        linearActRight   = myOpMode.hardwareMap.get(DcMotor.class, "linear_act_right");

        intake = myOpMode.hardwareMap.get(DcMotor.class,"intake");
        intake_back = myOpMode.hardwareMap.get(DcMotor.class,"intake_back");

        plane_launcher = myOpMode.hardwareMap.get(Servo.class,"plane_launcher");

        liftLeft = myOpMode.hardwareMap.get(CRServo.class,"lift_left");
        liftRight = myOpMode.hardwareMap.get(CRServo.class,"lift_right");
       wrist = myOpMode.hardwareMap.get(CRServo.class,"wrist");
        boxLever = myOpMode.hardwareMap.get(Servo.class,"box_lever");

        // Set directions for drive motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
//
        linearActLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearActRight.setDirection(DcMotorSimple.Direction.FORWARD);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void liftUp(){
        setCRServoPower(LIFT_UP_POWER);
    }

    public void liftDown(){
        setCRServoPower(LIFT_DOWN_POWER);
    }

    public void actuatorExpand(){
        setLinearActuatorPower(0.5);
    }

    public void actuatorRetract(){
        setLinearActuatorPower(-0.5);
    }

    public void actuatorStop(){
        setLinearActuatorPower(0.0);
    }

    public void setIntakePower(double power){
//        intake.setPower(power);
//        intake_back.setPower(power);
    }
    public void setLinearActuatorPower(double power){
//        linearActLeft.setPower(power);
//        linearActRight.setPower(power);
    }

    public void setCRServoPower(double power){
//        liftLeft.setPower(power);
//        liftRight.setPower(power);
    }

    public void setPlaneLauncherPosition(double position){
//        plane_launcher.setPosition(position);
    }

    /**
     * Calculates the power required for each of the 4 wheels' motor to accomplish the required axial, lateral and yaw motions.
     * Then sends these power levels to the motors.
     *
     * @param axial     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lateral   Left/Right driving power (-1.0 to 1.0) +ve is left
     * @param yaw      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double axial, double lateral, double yaw) {

        // Combine requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        // Scale the values so neither exceed +/- 1.0
        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Use existing function to drive all 4 wheels.
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void stop(){
        driveRobot(0.0,0.0,0.0);
    }
    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftFrontPower     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontPower    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackPower    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackPower    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

}
