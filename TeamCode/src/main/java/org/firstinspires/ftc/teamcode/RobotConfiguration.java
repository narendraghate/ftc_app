package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;

/**
 * Created by Shishir on 9/21/2017.
 */

public class RobotConfiguration {

    private Gamepad _gamepad;
    private Telemetry _telemetry;
    private JewelColor _jewelColor;

    public enum JewelColor
    {
        Blue,
        Red
    }

    public RobotConfiguration(Gamepad gamepad, Telemetry telemetry)
    {
        _gamepad = gamepad;
        _telemetry = telemetry;
        _jewelColor = JewelColor.Red;
    }

    public JewelColor getJewelColor() {
        return _jewelColor;
    }

    public void ShowMenu() {
        do {
            if (_gamepad.x) {
                _jewelColor = JewelColor.Red;
            }

            if (_gamepad.b) {
                _jewelColor = JewelColor.Blue;
            }

            _telemetry.addData("Menu", "x = Red, b = Blue,");
            _telemetry.addData("Selected", "JewelColor %s", _jewelColor);
            _telemetry.update();
        } while (true);
    }



}
