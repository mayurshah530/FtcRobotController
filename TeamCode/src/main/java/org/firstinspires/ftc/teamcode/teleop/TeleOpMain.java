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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.V4Hardware;


@TeleOp(name="TeleOpMain", group="Linear OpMode")
public class TeleOpMain extends LinearOpMode {

    V4Hardware robot = new V4Hardware(this);

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    public double SquareInputWithSign(double input){
        double sign = 1.0;
        if (input < 0.0){
            sign = -1.0;
        }
        return sign * input * input;
    }
    @Override
    public void runOpMode() {


        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        robot.setBoxPosition(V4Hardware.BOX_CLOSE_POSITION);
        robot.boxLeverPosition(V4Hardware.BOXLEVER_HOME_POSITION);
        robot.wristPosition(V4Hardware.WRIST_HOME_POSITION);



        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double prevLiftPower = 0.0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            /* =======================
             Gamepad 1 controls
            ======================= */

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
//            double axial   = -gamepad1.left_stick_y/2;  // Note: pushing stick forward gives negative value
//            double lateral =  gamepad1.left_stick_x/2;
//            double yaw     =  gamepad1.right_stick_x/2;

            // Note: pushing stick forward gives negative value
            double axial = SquareInputWithSign(-gamepad1.left_stick_y);
            double lateral =  SquareInputWithSign(gamepad1.left_stick_x);
            double yaw     =  SquareInputWithSign(gamepad1.right_stick_x);
            robot.driveRobot(axial, lateral, yaw);


            /* =======================
             Gamepad 2 controls
            ======================= */

            // Move linear actuator
            if (gamepad2.left_bumper) {
                robot.actuatorExpand();
            } else if (gamepad2.right_bumper){
                robot.actuatorRetract();
            } else {
                robot.actuatorStop();
            }


            // Tilt up/down
            // Right trigger to move viper slide up, left trigger to move it down.
            double servoLiftPower = gamepad2.right_trigger - gamepad2.left_trigger;
            Range.clip(servoLiftPower, -0.9, 0.9);
            robot.setCRServoPower(servoLiftPower);
            prevLiftPower = servoLiftPower;
            if (Math.abs(servoLiftPower) < 0.001) {
                robot.setCRServoPower(prevLiftPower);
            }

            //plane_launcher
            if (gamepad2.dpad_up) {
                robot.setPlaneLauncherPosition(0.0);
            } else {
                robot.setPlaneLauncherPosition(1.0);
            }

            if (gamepad1.a){
                robot.boxLeverPosition(robot.BOXLEVER_HOME_POSITION);
            } else if (gamepad1.y) {
                robot.boxLeverPosition(robot.BOXLEVER_SCORING_POSITION);
            }

            if (gamepad1.x){
                robot.wristPosition(robot.WRIST_HOME_POSITION);
            } else if (gamepad1.b) {
                robot.wristPosition(robot.WRIST_SCORING_POSITION);
            }

            if (gamepad1.dpad_right){
                robot.setIntakePower(1.0);
            } else if (gamepad1.dpad_left) {
                robot.setIntakePower(-1.0);
            } else
                robot.setIntakePower(0.0);

            if (gamepad1.left_bumper){
                robot.setBoxPosition(robot.BOX_CLOSE_POSITION);
            } else if (gamepad1.right_bumper) {
                robot.setBoxPosition(robot.BOX_SCORING_POSITION);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Currently at",  " Left: %7d Right: %7d",
//                    robot.getLinearActuatorLeftPosition(), robot.getLinearActuatorRightPosition());

//            telemetry.addData("Front Wheel left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  Wheel left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.addData("Viper slide power", "%4.2f", viperSlidePower );
//            telemetry.addData("Intake motor power Front/Back", "%4.2f, %4.2f", intake_front.getPower(), intake_back.getPower());
//            telemetry.addData("Forearm servo position", "%4.2f", forearm.getPosition());
//            telemetry.addData("Wrist servo position", "%4.2f", wrist.getPosition());
            telemetry.update();
        }
    }}
