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
    public Servo ClawL = null;
    public Servo ClawR = null;
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
        hwMap = ahwMap;
        //Motor naming
        /*FrontL = hwMap.dcMotor.get("front left");
        FrontR = hwMap.dcMotor.get("front right");
        BackL = hwMap.dcMotor.get("back left");
        BackR = hwMap.dcMotor.get("back right");
        Slide = hwMap.dcMotor.get("slide");*/
        ClawL = hwMap.servo.get("claw left");
        ClawR = hwMap.servo.get("claw right");


    }

}
