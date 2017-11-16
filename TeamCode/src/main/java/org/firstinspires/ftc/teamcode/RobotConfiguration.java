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
        Left,
        Right
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
        _turnDirection = TurnDirection.Left;
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

            if (_gamepad.dpad_right) {
                _turnDirection = TurnDirection.Right;
            }

            if (_gamepad.a) {
                break;
            }

            _telemetry.addData("Menu", "x = Blue, b = Red, dpl = turn left, dpr = turn right");
            _telemetry.addData("Menu", "a = Lock in");
            _telemetry.addData("Left Color", "Red = %d, Green = %d, Blue = %d", _robot.GetLeftColorSensor().red(), _robot.GetLeftColorSensor().green(), _robot.GetLeftColorSensor().blue());
            _telemetry.addData("Right Color", "Red = %d, Green = %d, Blue = %d", _robot.GetRightColorSensor().red(), _robot.GetRightColorSensor().green(), _robot.GetRightColorSensor().blue());
            _telemetry.addData("Alliance color is", "%s", _allianceColor);
            if (_turnDirection == TurnDirection.Left) {
                _telemetry.addData("Turning Left", "when facing the wall");
            } else {
                _telemetry.addData("Turning Right", "when facing the wall");
            }
            _telemetry.update();
        } while (true);
    }
}
