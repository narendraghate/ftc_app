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

@Autonomous(name="Blue Front Mode", group="Robot Opmode")
public class BlueFrontAutonomousOpMode extends LinearOpMode
{
    PieceOfCakeRobot robot   = new PieceOfCakeRobot();
    int LiftHeightPart1 = 3388;
    int LiftHeightPart2 = 5788;
    int TiltHeight = 2850;
    int TiltBack = 1250;
    int DriveDistance = 150;

    @Override
    public void runOpMode() {
        boolean OpenLeftClaw = false;
        boolean OpenRightClaw = false;
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
        idle();

        telemetry.addData("Waiting", "");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Starting", "now");
        telemetry.update();

        // Moving the robot forward.
        robot.GetLeft().setTargetPosition(DriveDistance);
        robot.GetRight().setTargetPosition(DriveDistance);

        // Setting the Power of the motors to 0.25.
        robot.GetLeft().setPower(0.25);
        robot.GetRight().setPower(0.25);

        // Activate the Lift to go to a certain position while the left and right motors are moving.
        robot.GetLift().setTargetPosition(LiftHeightPart1);
        robot.GetLift().setPower(1);

        // Wait to reach the lift position and add the telemetry while that is happening.
        while (robot.GetLift().isBusy())
        {
            telemetry.addData("Lift Position", "%d", robot.GetLift().getCurrentPosition());
            telemetry.update();
        }

        // Setting the second lift height and starting the tilt.
        robot.GetLift().setTargetPosition(LiftHeightPart2);
        robot.GetTilt().setTargetPosition(TiltHeight);
        robot.GetTilt().setPower(.5);

        // Activating the tilt and the lift and the same time
        while (robot.GetTilt().isBusy() || robot.GetLift().isBusy())
        {
            telemetry.addData("Tilt Position", "%d", robot.GetTilt().getCurrentPosition());
            telemetry.addData("Lift Position", "%d", robot.GetLift().getCurrentPosition());
            telemetry.update();
        }

        if (robot.GetLeftColorSensor().red() > robot.GetRightColorSensor().red()) {
            OpenLeftClaw = true;
        }else {
            OpenRightClaw = true;
        }

        // Opening and closing the claws. The sleep allows the claw to actually open and close.
        if  (OpenLeftClaw) {
            robot.GetClawL().setPower(-.9);
            sleep(4000);
            robot.GetClawL().setPower(.9);
            sleep(4200);
            robot.GetClawL().setPower(0);
        }

        if (OpenRightClaw) {
            robot.GetClawR().setPower(.9);
            sleep(4000);
            robot.GetClawR().setPower(-.9);
            sleep(4200);
            robot.GetClawR().setPower(0);
        }

        // Start tilting the robot back.
        robot.GetTilt().setTargetPosition(TiltBack);
        while (robot.GetTilt().isBusy())
        {
            telemetry.addData("Tilt Position", "%d", robot.GetTilt().getCurrentPosition());
            telemetry.update();
        }

        // Tilting the robot back while making the lift go down all the way.
        robot.GetTilt().setTargetPosition(0);
        robot.GetLift().setTargetPosition(0);
        while (robot.GetLift().isBusy() || robot.GetTilt().isBusy())
        {
            telemetry.addData("Lift Position", "%d", robot.GetLift().getCurrentPosition());
            telemetry.update();
        }

        // Try and go to a safe zone
        robot.GetLeft().setTargetPosition(-920);
        robot.GetRight().setTargetPosition(1689);

        // waiting for the turn to finish
        while (robot.GetLeft().isBusy() || robot.GetRight().isBusy()) {
            telemetry.addData("Left Position", "%d", robot.GetLeft().getCurrentPosition());
            telemetry.addData("Right Position", "%d", robot.GetRight().getCurrentPosition());
            telemetry.update();
        }

        // sleep so we can get off balancing stone
        sleep(500);
        robot.GetLeft().setTargetPosition(580);
        robot.GetRight().setTargetPosition(3889);

        // waiting for the robot to go forward
        while (robot.GetLeft().isBusy() || robot.GetRight().isBusy()) {
            telemetry.addData("Left Position", "%d", robot.GetLeft().getCurrentPosition());
            telemetry.addData("Right Position", "%d", robot.GetRight().getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("Status","Finished");
        telemetry.update();
    }
}
// Done!