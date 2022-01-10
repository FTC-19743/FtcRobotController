package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
public class OutakeArm {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public DcMotorEx armMotor;
    public CRServo spinnerServo;
    public Servo outakeSlider;
    public static int Ground = 0;
    public static int Level1 = 970;
    public static int Level2 = 2250;
    public static double Level2SliderPosition = .75;
    public static int Level3 = 3200;
    public static int Level3SliderPosition = 0;
    //public static int Top = 10;
    public static int ArmSpeed = 2700; // was 400
    public static int MaxPosition =4000; //max possible position used for TSE
    public static int StallBuffer = 450; //lift off the stall distance
    public static int ManualArmIncrement = 50;
    public static int ArmVelocity = 3000;
    //public static int Level1WithTelescope = 970;


    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    public OutakeArm(){
        teamUtil.log("Constructing Outake Arm");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void init(){
        teamUtil.log("Initializing Arm");
        armMotor = hardwareMap.get(DcMotorEx.class, "outake_arm");
        spinnerServo = hardwareMap.get(CRServo.class,"outake_spinner");
        outakeSlider = hardwareMap.get(Servo.class,"outake_slider");
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    public void writeTelemetry(){
        telemetry.addData("Outake","Arm Position:%d",armMotor.getCurrentPosition());
        telemetry.addData("Outake Speed","Arm Speed:%f",armMotor.getPower());
        telemetry.addData("Outake Slider Position", "Slider Position:%f",outakeSlider.getPosition());

    }

    public void resetArm(){
        outakeSlider.setPosition(1);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int lastEncoderPosition = armMotor.getCurrentPosition();
        armMotor.setPower(-.2);
        teamUtil.pause(250);
        while(armMotor.getCurrentPosition()!=lastEncoderPosition){
            lastEncoderPosition= armMotor.getCurrentPosition();
            teamUtil.pause(50);
        }
        armMotor.setPower(0);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int encoderPosition = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(encoderPosition+StallBuffer);
        armMotor.setPower(0.3);
        while(armMotor.isBusy()){
        }
        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        log("Arm Motor Stalled");
    }
    public void runArmToPosition(int level){//TODO take method out when no longer referenced
    }

    public void runToTop(){
        //runArmToPosition(Top);
    }
    /*
    public void runArmToPosition(int position){
        armMotor.setTargetPosition(position);


        // Turn On RUN_TO_POSITION
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // start motion.
        armMotor.setVelocity(Math.abs(ArmSpeed));

        teamUtil.log("Rotating Arm");
        long currentTime = System.currentTimeMillis() + 5000;

    }

     */

    public void runToFirstLevel(){
        armMotor.setTargetPosition(Level1);
        armMotor.setVelocity(ArmVelocity);
        outakeSlider.setPosition(1);
    }

    public void runToSecondLevel(){
        armMotor.setTargetPosition(Level2);
        armMotor.setVelocity(ArmVelocity);
        outakeSlider.setPosition(Level2SliderPosition);
    }

    public void runToThirdLevel(){
        armMotor.setTargetPosition(Level3);
        armMotor.setVelocity(ArmVelocity);
        outakeSlider.setPosition(Level3SliderPosition);
    }

    public void runToGround(){
        armMotor.setTargetPosition(0);
        armMotor.setVelocity(ArmVelocity);
        outakeSlider.setPosition(1);
    }
    public void runToGroundForDucks(){
        armMotor.setTargetPosition(650);
        armMotor.setVelocity(ArmVelocity);
        outakeSlider.setPosition(0);
    }
    public void runToSharedHub(){
        armMotor.setTargetPosition(1690);
        armMotor.setVelocity(ArmVelocity);
        outakeSlider.setPosition(0.75);
    }
    public void runToCap(){
        armMotor.setTargetPosition(3999);
        armMotor.setVelocity(ArmVelocity);
        outakeSlider.setPosition(0);
    }

    public void runArmUp(){

        log("running arm up");

        int currentPosition= armMotor.getCurrentPosition();
        if(currentPosition+ManualArmIncrement<MaxPosition){
            armMotor.setTargetPosition(currentPosition+ManualArmIncrement);
        }


    }

    public void runArmDown(){

        log("running arm up");

        int currentPosition= armMotor.getCurrentPosition();
        if(currentPosition-ManualArmIncrement>0){
            armMotor.setTargetPosition(currentPosition-ManualArmIncrement);
        }

    }

    public void stopArm(){
        log("Arm Stopped");
        armMotor.setPower(0);

    }

    public void spinnerIntake(){
        spinnerServo.setPower(-1);
    }

    public void spinnerIntakeSlow(){spinnerServo.setPower(-.5);}

    public void spinnerOutput(){
        spinnerServo.setPower(1);
    }

    public void spinnerStop(){
        spinnerServo.setPower(0);
    }

    public void sliderIncrement(){
        log("Slider going out");
        double neededSliderPosition = (outakeSlider.getPosition() + .05);
        outakeSlider.setPosition(neededSliderPosition);
    }

    public void sliderDecrease(){
        log("Slider going in");
        double neededSliderPosition = (outakeSlider.getPosition() - .05);
        outakeSlider.setPosition(neededSliderPosition);
    }



}
