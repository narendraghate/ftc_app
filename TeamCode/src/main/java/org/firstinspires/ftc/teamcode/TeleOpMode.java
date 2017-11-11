/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.R.attr.left;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOpMode Mode", group="Robot Opmode")
public class TeleOpMode extends OpMode {

    private PieceOfCakeRobot robot = new PieceOfCakeRobot();
    static final int NumberToTellWeAreInTilt = 200;

    @Override
    public void init(){
        //Reset motor encoders
        robot.init(hardwareMap);

        robot.GetTilt().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetTilt().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.GetLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetLift().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.GetLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.GetLeft().setDirection(DcMotor.Direction.REVERSE);

        robot.GetRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop(){
        ProcessClaw();
        LiftControl();
        DriveControl();
        TiltControl();
        PowerPercent();
        LiftPercent();
        SlideControl();
        CheckSafetyMode();
        
        telemetry.addData("Tilt", "%d", robot.GetTilt().getCurrentPosition());
        telemetry.addData("Lift", "%d", robot.GetLift().getCurrentPosition());

        telemetry.addData("Left Position", "%d", robot.GetLeft().getCurrentPosition());
        telemetry.addData("Right Position", "%d", robot.GetRight().getCurrentPosition());

        telemetry.addData("PowerPercentage", "%f", robot.GetPowerPercentage());
        telemetry.addData("LiftPercentage", "%f", robot.GetLiftPowerPercentage());
        telemetry.addData("Gamepad2 LeftStick Position", "%f", gamepad2.left_stick_y);
        telemetry.addData("Minimum Lift Height", "%d", robot.GetMinLiftHeight());
        if (robot.IsSafetyOff()) {
            telemetry.addData("Safety", "Off");
        } else {
            telemetry.addData("Safety", "On");
        }

        // this should always be the last line so any telemetry that wes done in other
        // methods is displayed
        telemetry.update();
    }

    /*
    Makes the D-pad left turn the safety on, and the D-pad right changes the safety off
     */
    private void CheckSafetyMode() {
        if (gamepad1.dpad_left) {
            robot.SetSafetyOff(false);
        }

        if (gamepad1.dpad_right) {
            robot.SetSafetyOff(true);
        }
    }

    // Sets power for claws when the bumpers are pressed
    private void ProcessClaw() {
        double leftClawPower = 0.0;
        double rightClawPower = 0.0;
        double power = 0.0;

        if (gamepad2.left_bumper) {
            leftClawPower = -0.5;
            rightClawPower = 0.5;
        }

        if (gamepad2.right_bumper) {
            leftClawPower = 0.5;
            rightClawPower = -0.5;
        }

        robot.GetClawL().setPower(leftClawPower);
        robot.GetClawR().setPower(rightClawPower);
    }

    /*
        LiftControl method checks for safety (Min/Max checks) so that the lift doesn't run off the track
     */
    private void LiftControl() {
        // Sets initial power
        double LiftPower = 0.0;
        int currentLiftPosition = robot.GetLift().getCurrentPosition();
        int currentTiltPosition = robot.GetTilt().getCurrentPosition();
        // Checks to see if SafetyOff = true
        // We can disable the boundaries of maximum lift height and tilt checks
        if (robot.IsSafetyOff()) {
            if (gamepad2.left_stick_y > 0) {
                LiftPower = -(robot.GetLiftPowerPercentage());
            }

            if (gamepad2.left_stick_y < 0) {
                LiftPower = (robot.GetLiftPowerPercentage());
            }
        } else {
            // If the tilt is more than the number to tell we are in tilt,
            if (currentTiltPosition > NumberToTellWeAreInTilt) {
                // The robot can't go down too far because we have to be at a certain lift height in order to tilt
                if ((gamepad2.left_stick_y > 0) && (currentLiftPosition > robot.GetMinLiftTiltHeight())) {
                    LiftPower = -(robot.GetLiftPowerPercentage());
                }

                // Makes the robot only go to a certain height
                if ((gamepad2.left_stick_y < 0) && (currentLiftPosition < robot.GetMaxLiftHeight())) {
                    LiftPower = (robot.GetLiftPowerPercentage());
                }
            } else {
                //Not in tilt
                if ((gamepad2.left_stick_y > 0) && (currentLiftPosition > robot.GetMinLiftHeight())) {
                    LiftPower = -(robot.GetLiftPowerPercentage());
                }
                // Makes the robot only go to a certain height
                if ((gamepad2.left_stick_y < 0) && (currentLiftPosition < robot.GetMaxLiftHeight())) {
                    LiftPower = (robot.GetLiftPowerPercentage());
                }
            }
        }

        robot.GetLift().setPower(LiftPower);
    }

    // Code written by Narendra
    private void LiftPercent() {
        double LiftPercentage = robot.GetLiftPowerPercentage();

        if (gamepad2.y) {
            LiftPercentage = 0.25;
        }
        if (gamepad2.b) {
            LiftPercentage = 0.5;

        }
        if (gamepad2.a) {
            LiftPercentage = 0.75;
        }
        if (gamepad2.x) {
            LiftPercentage = 1;
        }

        robot.SetLiftPercentage(LiftPercentage);
    }

    // Code written by Narendra
    private void SlideControl() {
        double SlidePower = 0.0;

        SlidePower = gamepad1.right_stick_x * robot.GetPowerPercentage();
        SlidePower = Range.clip(SlidePower, -1, 1);

        robot.GetSlide().setPower(SlidePower);
    }

    /*
        Gives you the option of choosing your power setting for the robot drive control/ TeleOp arcade drive
     */
    private void PowerPercent() {
        double PowerPercentage = robot.GetPowerPercentage();

        if (gamepad1.y) {
            PowerPercentage = 0.25;
        }
        if (gamepad1.b) {
            PowerPercentage = 0.5;
        }
        if (gamepad1.a) {
            PowerPercentage = 0.75;
        }
        if (gamepad1.x) {
            PowerPercentage = 1;
        }

        robot.SetPowerPercentage(PowerPercentage);
    }

    /*
        Powers robot for TeleOp arcade drive and clips power values
     */
    private void DriveControl(){
        float xValue = gamepad1.left_stick_x;
        float yValue = -gamepad1.left_stick_y;

        //calculate the power needed for each motor
        double leftPower = (yValue - xValue) * robot.GetPowerPercentage();
        double rightPower =(yValue + xValue) * robot.GetPowerPercentage();

        //clip the power values so that it only goes from -1 to 1
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        //set the power of the motors with the gamepad values
        robot.GetLeft().setPower(leftPower);
        robot.GetRight().setPower(rightPower);
    }

    /*
        TiltControl makes sure the tilt doesn't work if the lift would hit the robot, but also adds
        the option to disregard the restrictions
     */
    private void TiltControl() {
        double TiltPower = 0.0;
        long currentLiftPosition = robot.GetLift().getCurrentPosition();
        long currentTiltPosition = robot.GetTilt().getCurrentPosition();

        // Checks to see if SafetyOff = true
        if (robot.IsSafetyOff()) {
            //Able to break the robot
            //The gamepad can control the tilt
            if (gamepad2.dpad_left) {
                TiltPower = 0.25;
            }

            if (gamepad2.dpad_right) {
                TiltPower = -0.5;
            }
        } else{
            // Tilt can only work between max tilt height and min tilt height
            if ((currentLiftPosition >= robot.GetMinLiftTiltHeight()) && (currentLiftPosition <= robot.GetMaxLiftHeight())) {
                if (gamepad2.dpad_left) {
                    TiltPower = 0.5;
                }
                //Makes the robot unable to tilt back too far
                if (gamepad2.dpad_right)  {
                    TiltPower = -0.5;
                }
            }
        }

        robot.GetTilt().setPower(TiltPower);
    }
}
