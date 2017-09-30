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
    private DcMotor FrontL = null;
    private DcMotor FrontR = null;
    private DcMotor BackL = null;
    private DcMotor BackR = null;
    private DcMotor Slide = null;
    private ColorSensor colorsensor = null;

    private HardwareMap hwMap = null;

    // Constructor
    public PieceOfCakeRobot() {

    }

    //Hardware naming
    public void init(HardwareMap ahwMap) {
        CRServo crservo = null;
        DcMotor dcMotor = null;
        ColorSensor colorSensor =null;

        hwMap = ahwMap;

        //Motor naming
        /*FrontL = hwMap.dcMotor.get("front left");
        FrontR = hwMap.dcMotor.get("front right");
        BackL = hwMap.dcMotor.get("back left");
        BackR = hwMap.dcMotor.get("back right");
        Slide = hwMap.dcMotor.get("slide");*/
        crservo = hwMap.crservo.get("claw left");
        SetClawL(crservo);

        crservo = hwMap.crservo.get("claw right");
        SetClawR(crservo);

        dcMotor = hwMap.dcMotor.get("front left");
        SetFrontL(dcMotor);

        dcMotor = hwMap.dcMotor.get("front right");
        SetFrontR(dcMotor);

       dcMotor = hwMap.dcMotor.get("back left");
        SetBackL(dcMotor);

        dcMotor = hwMap.dcMotor.get("back right");
        SetBackR(dcMotor);

        dcMotor = hwMap.dcMotor.get("slide");
        SetSlide(dcMotor);

        colorSensor = hwMap.colorSensor.get("colorsensor");
        Setcolorsensor(colorSensor);
    }

    public CRServo GetClawL() {
        return ClawL;
    }

    public CRServo GetClawR() {
        return ClawR;
    }

    public DcMotor GetFrontL()  {
        return FrontL;
    }

    public DcMotor GetFrontR()  {
        return FrontR;
    }

    public DcMotor GetBackL()  {
        return BackL;
    }

    public DcMotor GetBackR()  {
        return BackR;
    }

    public DcMotor GetSlide()  {
        return Slide;
    }

    public ColorSensor getcolorsensor() {
        return colorsensor;
    }


    private void SetClawR(CRServo crservo) { ClawR = crservo; }
    private void SetClawL(CRServo crservo) { ClawL = crservo; }
    private void SetFrontL(DcMotor dcMotor) { FrontL = dcMotor; }
    private void SetFrontR(DcMotor dcMotor) { FrontR = dcMotor; }
    private void SetBackL(DcMotor dcMotor) { BackL = dcMotor; }
    private void SetBackR(DcMotor dcMotor) { BackR = dcMotor; }
    private void SetSlide(DcMotor dcMotor) { Slide = dcMotor; }
    private void Setcolorsensor(ColorSensor colorSensor) { colorsensor = colorSensor; }
    


}
