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
    public static int Ground = 0; // was 548
    public static int Level1 = -495; // was 500
    public static int Level2 = -1176; // was 380
    public static int Level3 = -1667; // was 269
    public static int Top = 10;
    public static int ArmSpeed = 2700; // was 400
    public static int MaxPosition =50000; //TODO
    public static int StallBuffer = 500; //TODO

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
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    public void writeTelemetry(){
        telemetry.addData("Outake","Arm Position:%d",armMotor.getCurrentPosition());
        telemetry.addData("Outake Speed","Arm Speed:%f",armMotor.getPower());
        telemetry.addData("Outake Slider Position", "Slider Position:%f",outakeSlider.getPosition());

    }

    public void resetArm(){
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int lastEncoderPosition = armMotor.getCurrentPosition();
        armMotor.setPower(-.1);
        teamUtil.pause(250);
        while(armMotor.getCurrentPosition()!=lastEncoderPosition){
            lastEncoderPosition= armMotor.getCurrentPosition();
            teamUtil.pause(250);
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
        log("Arm Motor Stalled");
    }

    public void runToTop(){
        runArmToPosition(Top);
    }

    public void runArmToPosition(int position){
        armMotor.setTargetPosition(position);


        // Turn On RUN_TO_POSITION
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // start motion.
        armMotor.setVelocity(Math.abs(ArmSpeed));

        teamUtil.log("Rotating Arm");
        long currentTime = System.currentTimeMillis() + 5000;

    }
    public void runArmUp(){
        log("running arm up");
        double currentPosition= armMotor.getCurrentPosition();
        if(currentPosition>MaxPosition){
            armMotor.setPower(0);
        }
        else{
            armMotor.setVelocity(3000);
        }

    }

    public void runArmDown(){
        log("running arm down");
        double currentPosition= armMotor.getCurrentPosition();
        if(currentPosition==0){
            armMotor.setPower(0);
        }
        else{
            armMotor.setVelocity(-3000);
        }

    }

    public void stopArm(){
        armMotor.setPower(0);
    }

    public void spinnerIntake(){
        spinnerServo.setPower(-1);
    }

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
