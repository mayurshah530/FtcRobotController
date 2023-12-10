/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp_Leauge2", group="Linear OpMode")
public class TeleOp_League2 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor viper_slide = null;

    private DcMotor intake_front = null;
    private DcMotor intake_back = null;
    private Servo forearm = null;
    private Servo wrist = null;
    private Servo plane_launcher= null;

    public double INTAKE_MOTOR_POWER = 1.0;
    public double FOREARM_INTAKE_POSITION = 1.0;
    public double FOREARM_LIFT_POSITION = 0.5;
    public double FOREARM_DROP_POSITION = 0.0;

    public double WRIST_INTAKE_POSITION = 1.0;
    public double WRIST_LIFT_POSITION = 0.5;
    public double WRIST_DROP_POSITION = 0.0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        viper_slide = hardwareMap.get(DcMotor.class, "viper_slide");
        intake_front = hardwareMap.get(DcMotor.class,"intake_front");
        intake_back = hardwareMap.get(DcMotor.class,"intake_back");
        plane_launcher = hardwareMap.get(Servo.class,"plane_launcher");
        forearm = hardwareMap.get(Servo.class,"forearm");
        wrist = hardwareMap.get(Servo.class,"wrist");



        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        viper_slide.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_front.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_back.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // =======================
            // Gamepad 1 controls
            // =======================

            // Intake motor
            if (gamepad1.left_bumper) {
                intake_front.setPower(-INTAKE_MOTOR_POWER);
                intake_back.setPower(-INTAKE_MOTOR_POWER);
            } else if (gamepad1.right_bumper) {
                intake_front.setPower(INTAKE_MOTOR_POWER);
                intake_back.setPower(INTAKE_MOTOR_POWER);
            } else {
                intake_back.setPower(0);
                intake_front.setPower(0);
            }

            // Plane launcher
            if (gamepad1.dpad_up){
                plane_launcher.setPosition(1);
            } else{
                plane_launcher.setPosition(0.5);
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y/2;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x/2;
            double yaw     =  gamepad1.right_stick_x/2;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = (axial + lateral + yaw);
            double rightFrontPower = (axial - lateral - yaw);
            double leftBackPower   = (axial - lateral + yaw);
            double rightBackPower  = (axial + lateral - yaw);

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 0.5) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // =======================
            // Gamepad 2 controls
            // =======================

            if (gamepad2.a){
                forearm.setPosition(FOREARM_INTAKE_POSITION);
//                wrist.setPosition(WRIST_INTAKE_POSITION);
            } else if (gamepad2.b){
                forearm.setPosition(FOREARM_LIFT_POSITION);
//                wrist.setPosition(WRIST_LIFT_POSITION);
            } else if (gamepad2.y) {
                forearm.setPosition(FOREARM_DROP_POSITION);
//                wrist.setPosition(WRIST_DROP_POSITION);
            }

            // Right trigger to move viper slide up, left trigger to move it down.
            double viperSlidePower = gamepad2.right_trigger - gamepad2.left_trigger;
            viper_slide.setPower(viperSlidePower);

            // WRIST POSITION
            if (gamepad2.left_bumper){
                wrist.setPosition(WRIST_INTAKE_POSITION);
            } else if (gamepad2.right_bumper){
                wrist.setPosition(WRIST_LIFT_POSITION);
            } else if (gamepad2.x){
                wrist.setPosition(WRIST_DROP_POSITION);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front Wheel left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  Wheel left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Viper slide power", "%4.2f", viperSlidePower );
            telemetry.addData("Intake motor power Front/Back", "%4.2f, %4.2f", intake_front.getPower(), intake_back.getPower());
            telemetry.addData("Forearm servo position", "%4.2f", forearm.getPosition());
            telemetry.addData("Wrist servo position", "%4.2f", wrist.getPosition());
            telemetry.update();
        }
    }}
