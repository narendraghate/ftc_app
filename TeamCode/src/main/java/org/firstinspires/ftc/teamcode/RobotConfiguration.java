package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;


public class RobotConfiguration {
    /*
    sets variables for the class
     */
    private Gamepad _gamepad;
    private Telemetry _telemetry;
    private AllianceColor _allianceColor;
    private PieceOfCakeRobot _robot;
    private TurnDirection _turnDirection;

    public enum AllianceColor
    {
        Blue,
        Red
    }

    public enum TurnDirection
    {
        None,
        Left,
        FarLeft,
        Right,
        FarRight,
    }
     /*
     shows what the variables are equal to
      */
    public RobotConfiguration(PieceOfCakeRobot robot, Gamepad gamepad, Telemetry telemetry)
    {
        _gamepad = gamepad;
        _telemetry = telemetry;
        _allianceColor = AllianceColor.Blue;
        _robot = robot;
        _turnDirection = TurnDirection.None;
    }
    /*
    tell the computer/prints out on the screen your choice
     */
    public AllianceColor getAllianceColor() {
        return _allianceColor;
    }

    public TurnDirection getTurnDirection() { return _turnDirection; }

    /*
    if x is pressed it sets the alliancecolor to blue, if b is pressed it sets alliancecolor
    to red, if a is pressed it breaks
     */
    public void ShowMenu() {
        do {
            if (_gamepad.x) {
                _allianceColor = AllianceColor.Blue;
            }

            if (_gamepad.b) {
                _allianceColor = AllianceColor.Red;
            }

            if (_gamepad.dpad_left) {
                _turnDirection = TurnDirection.Left;
            }

            if (_gamepad.dpad_up) {
                _turnDirection = TurnDirection.FarLeft;
            }

            if (_gamepad.dpad_right) {
                _turnDirection = TurnDirection.Right;
            }

            if (_gamepad.dpad_down) {
                _turnDirection = TurnDirection.FarRight;
            }

            if (_gamepad.y) {
                _turnDirection = TurnDirection.None;
            }

            if (_gamepad.a) {
                break;
            }

            _telemetry.addData("Menu", "x = Blue, b = Red");
            _telemetry.addData("Menu", "dpl = turn left, dpu = turn far left");
            _telemetry.addData("Menu", "dpr = turn right, dpd = turn far right");
            _telemetry.addData("Menu", "y = no turn, a = Lock in");
            _telemetry.addData("Alliance color is", "%s", _allianceColor);
            if (_turnDirection == TurnDirection.Left) {
                _telemetry.addData("As a driver in the robot I will turn", "left and go to the middle.");
            } else if (_turnDirection == TurnDirection.Right) {
                _telemetry.addData("As a driver in the robot I will turn", "right and go to the middle.");
            } else if (_turnDirection == TurnDirection.FarLeft) {
                _telemetry.addData("As a driver in the robot I will turn", "left and go to the far back.");
            } else if (_turnDirection == TurnDirection.FarRight) {
                _telemetry.addData("As a driver in the robot I will turn", "right and go to the far back.");
            } else {
                _telemetry.addData("As a driver in the robot I will turn", "nowhere and just sit on my wheels.");
            }
            _telemetry.update();
        } while (true);
    }
}
