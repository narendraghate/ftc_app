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

    private double PowerPercentage = 1;

    private double LiftPercentage = 1;

    // Constructor
    public PieceOfCakeRobot() {

    }

    //Hardware naming
    public void init(HardwareMap ahwMap) {
        CRServo crservo = null;
        DcMotor dcMotor = null;
        ColorSensor colorSensor =null;

        hwMap = ahwMap;

        crservo = hwMap.crservo.get("clawleftservo"); //0
        SetClawL(crservo);

        crservo = hwMap.crservo.get("clawrightservo"); //1
        SetClawR(crservo);

        dcMotor = hwMap.dcMotor.get("left"); //0
        SetLeft(dcMotor);

        dcMotor = hwMap.dcMotor.get("right"); //1
        SetRight(dcMotor);

        dcMotor = hwMap.dcMotor.get("lift"); //2
        SetLift(dcMotor);

        dcMotor = hwMap.dcMotor.get("tilt"); //3
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

    public Double GetPowerPercentage(){return  PowerPercentage;}

    public Double GetLiftPowerPercentage() {return LiftPercentage;}

    private void SetClawR(CRServo crservo) { ClawR = crservo; }
    private void SetClawL(CRServo crservo) { ClawL = crservo; }
    private void SetLeft(DcMotor dcMotor) { Left = dcMotor; }
    private void SetRight(DcMotor dcMotor) { Right = dcMotor; }
    private void SetLift(DcMotor dcMotor) { Lift = dcMotor; }
    private void SetSlide(DcMotor dcMotor) { Slide = dcMotor; }
    private void SetTilt(DcMotor dcMotor) {Tilt = dcMotor;}
    private void SetColorSensor(ColorSensor colorSensor) { ColorSensor = colorSensor; }
    public void SetPowerPercentage(Double powerPercentage){ PowerPercentage = powerPercentage;}
    public void SetLiftPercentage(Double liftPercentage){LiftPercentage = liftPercentage;}
}
