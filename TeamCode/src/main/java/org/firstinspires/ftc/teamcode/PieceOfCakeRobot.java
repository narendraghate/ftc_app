package org.firstinspires.ftc.teamcode;
//Imports
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;

/**
* Created by MeechMeechman on 9/23/17.
*/
//Defining motors
public class PieceOfCakeRobot {
    private CRServo ClawL = null;
    private CRServo ClawR = null;
    private DcMotor Left = null;
    private DcMotor Right = null;
    private DcMotor Lift = null;
    private DcMotor Slide = null;
    private DcMotor Tilt = null;
    private ColorSensor ColorSensor = null;

    private HardwareMap hwMap = null;

    private double PowerPercentace;

    // Constructor
    public PieceOfCakeRobot() {

    }

    //Hardware naming
    public void init(HardwareMap ahwMap) {
        CRServo crservo = null;
        DcMotor dcMotor = null;
        ColorSensor colorSensor =null;

        hwMap = ahwMap;

        crservo = hwMap.crservo.get("clawleftservo");
        SetClawL(crservo);

        // uncomment the two lines belowonce you have the
        // other servo hooked into slot 1 and named clawrightservo
        // then go uncomment the code in the teleopmode.java class
        crservo = hwMap.crservo.get("clawrightservo");
        SetClawR(crservo);

        dcMotor = hwMap.dcMotor.get("left");
        SetLeft(dcMotor);

        dcMotor = hwMap.dcMotor.get("right");
        SetRight(dcMotor);

        dcMotor = hwMap.dcMotor.get("lift");
        SetLift(dcMotor);

        dcMotor = hwMap.dcMotor.get("tilt");
        SetTilt(dcMotor);

        dcMotor = hwMap.dcMotor.get("slide");
        SetSlide(dcMotor);

        //colorSensor = hwMap.colorSensor.get("colorsensor");
        //Setcolorsensor(colorSensor);





    }

    public CRServo GetClawL() {
        return ClawL;
    }

    public CRServo GetClawR() {
        return ClawR;
    }

    public DcMotor GetLeft()  {
        return Left;
    }

    public DcMotor GetRight()  {
        return Right;
    }

    public DcMotor GetLift()  {
        return Lift;
    }

    public DcMotor GetSlide()  {
        return Slide;
    }

    public DcMotor GetTilt() {return Tilt;}

    public ColorSensor GetColorSensor() {
        return ColorSensor;
    }

    public Double GetPowerPercentage(){return  PowerPercentace;}


    private void SetClawR(CRServo crservo) { ClawR = crservo; }
    private void SetClawL(CRServo crservo) { ClawL = crservo; }
    private void SetLeft(DcMotor dcMotor) { Left = dcMotor; }
    private void SetRight(DcMotor dcMotor) { Right = dcMotor; }
    private void SetLift(DcMotor dcMotor) { Lift = dcMotor; }
    private void SetSlide(DcMotor dcMotor) { Slide = dcMotor; }
    private void SetTilt(DcMotor dcMotor) {Tilt = dcMotor;}
    private void SetColorSensor(ColorSensor colorSensor) { ColorSensor = colorSensor; }
    public void SetPowerPercentage(Double PowerPercentage){ PowerPercentage = null;}
}
