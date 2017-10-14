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
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOpMode Mode", group="Robot Opmode")
public class TeleOpMode extends OpMode {

    private PieceOfCakeRobot robot = new PieceOfCakeRobot();
    static final int MaximumLiftHeight = 5780;
    static final int MinimiumLiftHeight = 5275;
    static final int MinimiumLiftHeight2 = 3275;

    @Override
    public void init(){
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

        telemetry.addData("Tilt", "%d", robot.GetTilt().getCurrentPosition());
        telemetry.addData("Lift", "%d", robot.GetLift().getCurrentPosition());

        telemetry.addData("Left Position", "%d", robot.GetLeft().getCurrentPosition());
        telemetry.addData("Right Position", "%d", robot.GetRight().getCurrentPosition());

        telemetry.addData("PowerPercentage", "%f", robot.GetPowerPercentage());
        // this should always be the last line so any telemetry that wes done in other
        // methods is displayed
        telemetry.update();
    }

    private void ProcessClaw()
    {
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

    private void ProcessMovement()
    {
        // eventually we want to move all the code related to the movement of the robot into this
        // method and call it from the main loop
    }

    private void LiftControl()
    {
        double LiftPower = 0.0;
        int currentLiftPosition = robot.GetLift().getCurrentPosition();
        int currentTiltPosition = robot.GetTilt().getCurrentPosition();

        if (currentTiltPosition > 200) {
            if ((gamepad2.left_stick_y > 0) && (currentLiftPosition > MinimiumLiftHeight)) {
                LiftPower = -0.5;
            }

            if ((gamepad2.left_stick_y < 0) && (currentLiftPosition < MaximumLiftHeight)) {
                LiftPower = 0.5;
            }
        }
        else {
            if ((gamepad2.left_stick_y > 0)) {
                LiftPower = -0.5;
            }

            if ((gamepad2.left_stick_y < 0) && (currentLiftPosition < MaximumLiftHeight)) {
                LiftPower = 0.5;
            }
        }

        robot.GetLift().setPower(LiftPower);
    }


    private void SlideControl()
    {
        double SlidePower = 0.0;

        if (gamepad1.left_bumper) {
            SlidePower = 0.5 * robot.GetPowerPercentage();
        }
        if (gamepad1.right_bumper) {
            SlidePower = -0.5 * robot.GetPowerPercentage();
        }

        robot.GetSlide().setPower(SlidePower);

    }

    private void PowerPercent()
    {
        double PowerPercentage = robot.GetPowerPercentage();

        if (gamepad1.y){
            PowerPercentage = 0.25;
        }
        if (gamepad1.b){
            PowerPercentage = 0.5;
        }
        if (gamepad1.a){
            PowerPercentage = 0.75;
        }
        if (gamepad1.x){
            PowerPercentage = 1;
        }

        robot.SetPowerPercentage(PowerPercentage);
    }

    private void DriveControl(){
        float xValue = gamepad1.left_stick_x;
        float yValue = -gamepad1.left_stick_y;

        //calculate the power needed for each motor
        double leftPower = (yValue + xValue) * robot.GetPowerPercentage();
        double rightPower =(yValue - xValue) * robot.GetPowerPercentage();

        //clip the power values so that it only goes from -1 to 1
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        //set the power of the motors with the gamepad values
        robot.GetLeft().setPower(leftPower);
        robot.GetRight().setPower(rightPower);
    }

    private void TiltControl()
    {
        double TiltPower = 0.0;
        long currentLiftPosition = robot.GetLift().getCurrentPosition();
        long currentTiltPosition = robot.GetTilt().getCurrentPosition();

        if ((currentLiftPosition >= MinimiumLiftHeight2) && (currentLiftPosition <= MaximumLiftHeight)) {
            if (gamepad2.dpad_left) {
                TiltPower = 0.5;
            }
            if ((gamepad2.dpad_right) && (currentTiltPosition > -100)){
                TiltPower = -0.5;
            }
        }

        robot.GetTilt().setPower(TiltPower);
    }
}