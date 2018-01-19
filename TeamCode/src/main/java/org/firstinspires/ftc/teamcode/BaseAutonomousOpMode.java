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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

abstract class BaseAutonomousOpMode extends LinearOpMode
{
    protected PieceOfCakeRobot robot   = new PieceOfCakeRobot();
    int LiftHeightPart = 6000;
    protected int TiltHeight = 2700;

    public enum AllianceColor
    {
        Blue,
        Red
    }

    public abstract void runOpMode();

    protected void InitRobot() {
        robot.init(hardwareMap);

        // We are initializing the motors.
        robot.GetLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.GetTilt().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetTilt().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.GetRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetRight().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.GetLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetLeft().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.GetLeft().setDirection(DcMotor.Direction.REVERSE);

        // Setting the Power of the motors to 0.25.
        robot.GetLeft().setPower(0.25);
        robot.GetRight().setPower(0.25);

        idle();

        telemetry.addData("Waiting", "");
        telemetry.update();
    }

    protected void TiltForJewel(){
        // Activate the Lift to go to a certain position while the left and right motors are moving.
        robot.GetLift().setTargetPosition(LiftHeightPart);
        robot.GetLift().setPower(0.5);

        // Wait to reach the lift position and add the telemetry while that is happening.
        telemetry.addData("Tilt", "%d", robot.GetTilt().getCurrentPosition());
        telemetry.addData("Lift", "%d", robot.GetLift().getCurrentPosition());
        telemetry.update();
        while (robot.GetLift().isBusy())
        {
            telemetry.addData("Tilt", "%d", robot.GetTilt().getCurrentPosition());
            telemetry.addData("Lift", "%d", robot.GetLift().getCurrentPosition());
            telemetry.update();
        }

        robot.GetTilt().setTargetPosition(TiltHeight);
        robot.GetTilt().setPower(0.3);
        // Wait to reach the lift position and add the telemetry while that is happening.
        telemetry.addData("Tilt", "%d", robot.GetTilt().getCurrentPosition());
        telemetry.addData("Lift", "%d", robot.GetLift().getCurrentPosition());
        telemetry.update();
        while (robot.GetTilt().isBusy())
        {
            telemetry.addData("Tilt", "%d", robot.GetTilt().getCurrentPosition());
            telemetry.addData("Lift", "%d", robot.GetLift().getCurrentPosition());
            telemetry.update();
        }
    }

    protected void KnockJewel(AllianceColor allianceColor) {
        boolean OpenLeftClaw = false;
        boolean OpenRightClaw = false;
        int LeftRed = 0;
        int RightRed = 0;

        telemetry.addData("Left Color", "%d", robot.GetLeftColorSensor().red());
        telemetry.addData("Right Color", "%d", robot.GetRightColorSensor().red());
        telemetry.update();

        for (int x = 0; x<13; x++) {

            if (robot.GetLeftColorSensor().red() > robot.GetRightColorSensor().red()){
                LeftRed = (LeftRed + 1);
            }
            else {
                RightRed = (RightRed + 1);
            }

        }

        telemetry.addData("LeftRed", "%d", LeftRed);
        telemetry.addData("RightRed", "%d", RightRed);
        
        if (allianceColor == AllianceColor.Blue) {
            if (LeftRed > RightRed) {
                OpenLeftClaw = true;
                telemetry.addData("Opening Claw", "Left");
            } else {
                telemetry.addData("Opening Claw", "Right");
                OpenRightClaw = true;
            }
        } else {
            if (LeftRed > RightRed) {
                OpenRightClaw = true;
                telemetry.addData("Opening Claw", "Right");
            } else {
                OpenLeftClaw = true;
                telemetry.addData("Opening Claw", "Left");
            }
        }

        telemetry.update();
        if (OpenRightClaw) {
            TurnSlightRight();
        } else {
            TurnSlightLeft();
        }
    }

    protected void TiltBackForJewel(int position) {
        robot.GetTilt().setTargetPosition(position);
        robot.GetTilt().setPower(0.5);
        telemetry.addData("Tilt", "%d", robot.GetTilt().getCurrentPosition());
        telemetry.addData("Lift", "%d", robot.GetLift().getCurrentPosition());
        telemetry.update();
        while (robot.GetTilt().isBusy())
        {
            telemetry.addData("Tilt", "%d", robot.GetTilt().getCurrentPosition());
            telemetry.addData("Lift", "%d", robot.GetLift().getCurrentPosition());
            telemetry.update();
        }
    }

    protected void MoveLiftToZero() {
        robot.GetLift().setTargetPosition(10);
        while (robot.GetLift().isBusy())
        {
            telemetry.addData("Tilt", "%d", robot.GetTilt().getCurrentPosition());
            telemetry.addData("Lift", "%d", robot.GetLift().getCurrentPosition());
            telemetry.update();
        }
    }

    protected void MoveRobot(int leftDistance, int rightDistance){

        robot.GetLeft().setTargetPosition(robot.GetLeft().getCurrentPosition() + leftDistance);
        robot.GetRight().setTargetPosition(robot.GetRight().getCurrentPosition() + rightDistance);

        // waiting for the turn to finish
        while (robot.GetLeft().isBusy() || robot.GetRight().isBusy()) {
            telemetry.addData("Left Position", "%d", robot.GetLeft().getCurrentPosition());
            telemetry.addData("Right Position", "%d", robot.GetRight().getCurrentPosition());
            telemetry.update();
        }
    }

    protected void TurnSlightRight() {
        MoveToPosition(-300, 300);
    }

    protected void TurnSlightLeft() {
        MoveToPosition(300, -300);
    }

    protected void TurnRight() {
        MoveRobot(-1040, 1040);
    }

    protected void TurnLeft() {
        MoveRobot(1040, -1040);
    }

    protected void MoveRobot(int distance) {
        MoveRobot(distance, distance);
    }

    protected void MoveToPosition(int leftPostion, int rightPosition) {

        robot.GetLeft().setTargetPosition(leftPostion);
        robot.GetRight().setTargetPosition(rightPosition);

        // waiting for the turn to finish
        while (robot.GetLeft().isBusy() || robot.GetRight().isBusy()) {
        }
    }
}
// Done!