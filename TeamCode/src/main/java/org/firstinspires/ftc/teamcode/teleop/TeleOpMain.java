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


@TeleOp(name="\uD83E\uDD16 \uD83C\uDFC6 TeleOpMain", group="Linear OpMode")
public class TeleOpMain extends LinearOpMode {

    V4Hardware robot = new V4Hardware(this);

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private double liftServoPosition = 0.0;

    public double boxLeverPosition;
    public int leftLaPosition;
    public int rightLaPosition;

    public int initLeftLaPosition;
    public int initRightLaPosition;

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
        robot.setBoxLeverPosition(V4Hardware.BOXLEVER_SCORING_POSITION);
        robot.setWristPosition(V4Hardware.WRIST_HOME_POSITION);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double prevLiftPower = 0.0;
        initLeftLaPosition = robot.getLinearActuatorLeftPosition();
        initRightLaPosition = robot.getLinearActuatorRightPosition();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            leftLaPosition = robot.getLinearActuatorLeftPosition();
            rightLaPosition = robot.getLinearActuatorRightPosition();
            boxLeverPosition = robot.getBoxLeverPosition();

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
            double yaw     =  SquareInputWithSign(gamepad1.right_stick_x * 0.75);
            robot.driveRobot(axial, lateral, yaw);

            //plane_launcher
            if (gamepad1.dpad_up) {
                robot.setPlaneLauncherPosition(0.0);
            } else {
                robot.setPlaneLauncherPosition(1.0);
            }

            // intake
            if (gamepad1.left_bumper){
                robot.setIntakePower(1.0);
                robot.setBoxLeverPosition(V4Hardware.BOXLEVER_INTAKE_POSITION);
            } else if (gamepad1.right_bumper) {
                robot.setIntakePower(-1.0);
                robot.setBoxLeverPosition(V4Hardware.BOXLEVER_INTAKE_POSITION);
            } else {
                robot.setIntakePower(0.0);
            }

            /* =======================
             Gamepad 2 controls
            ======================= */

            // Move linear actuator
            if (gamepad2.left_bumper) {
                robot.actuatorRetract();
            } else if (gamepad2.right_bumper){
                robot.actuatorExpand();
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

//            if (gamepad2.dpad_up){
//                liftServoPosition += 0.1;
//                liftServoPosition = Range.clip(liftServoPosition, 0.0, 0.9);
//            } else if (gamepad2.dpad_down) {
//                liftServoPosition -= 0.1;
//                liftServoPosition = Range.clip(liftServoPosition, 0.0, 0.9);
//            }
//            robot.setLiftPosition(liftServoPosition);


            if (gamepad2.a){
                robot.setBoxLeverPosition(V4Hardware.BOXLEVER_HOME_POSITION);
            } else if (gamepad2.y) {
                robot.setBoxLeverPosition(V4Hardware.BOXLEVER_SCORING_POSITION);
            }

            if (gamepad2.x){
                robot.setWristPosition(V4Hardware.WRIST_HOME_POSITION);
            } else if (gamepad2.b) {
                robot.setWristPosition(V4Hardware.WRIST_SCORING_POSITION);
            }

            if (gamepad2.dpad_left){
                robot.setBoxPosition(V4Hardware.BOX_CLOSE_POSITION);
            } else if (gamepad2.dpad_right) {
                robot.setBoxPosition(V4Hardware.BOX_SCORING_POSITION);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("BoxLeverPosition: ", "%2.1f", boxLeverPosition);
            telemetry.addData("Linear Actuator",  " Left: %5d Right: %5d", leftLaPosition, rightLaPosition);
            telemetry.update();
        }
    }}
