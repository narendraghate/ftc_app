package org.firstinspires.ftc.teamcode;
//Imports
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
* Created by MeechMeechman on 9/23/17.
*/
//Defining motors
public class PieceOfCakeRobot {
    private Servo ClawL = null;
    private Servo ClawR = null;
    public DcMotor FrontL = null;
    public DcMotor FrontR = null;
    public DcMotor BackL = null;
    public DcMotor BackR = null;
    public DcMotor Slide = null;


    private HardwareMap hwMap = null;

    // Constructor
    public PieceOfCakeRobot() {

    }

    //Hardware naming
    public void init(HardwareMap ahwMap) {
        Servo servo = null;
        hwMap = ahwMap;

        //Motor naming
        /*FrontL = hwMap.dcMotor.get("front left");
        FrontR = hwMap.dcMotor.get("front right");
        BackL = hwMap.dcMotor.get("back left");
        BackR = hwMap.dcMotor.get("back right");
        Slide = hwMap.dcMotor.get("slide");*/
        servo = hwMap.servo.get("claw left");
        SetClawL(servo);
        servo = hwMap.servo.get("claw right");
        SetClawR(servo);
    }

    public Servo GetClawL() {
        return ClawL;
    }

    public Servo GetClawR() {
        return ClawR;
    }

    private void SetClawR(Servo servo) { ClawR = servo; }
    private void SetClawL(Servo servo) {
        ClawL = servo;
    }
}
