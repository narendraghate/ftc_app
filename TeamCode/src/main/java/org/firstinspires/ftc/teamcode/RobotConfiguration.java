package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;

/**
 * Created by Shishir on 9/21/2017.
 */

public class RobotConfiguration {
    /*
    sets variables for the class
     */
    private Gamepad _gamepad;
    private Telemetry _telemetry;
    private AllianceColor _allianceColor;
    private PieceOfCakeRobot _robot;

    public enum AllianceColor
    {
        Blue,
        Red
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
    }
    /*
    tell the computer/prints out on the screen your choice
     */
    public AllianceColor getAllianceColor() {
        return _allianceColor;
    }

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

            if (_gamepad.a) {
                break;
            }

            _telemetry.addData("Menu", "x = Blue, b = Red, a = Lock in");
            _telemetry.addData("Color", "Red = %d, Green = %d, Blue = %d", _robot.GetColorSensor().red(), _robot.GetColorSensor().green(), _robot.GetColorSensor().blue());
            _telemetry.addData("Alliance color is", "%s", _allianceColor);
            _telemetry.update();
        } while (true);
    }
}
