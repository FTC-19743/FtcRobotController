package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outake {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public Servo grabber;
    public Servo rotator;
    public DcMotorEx pulleyLeft;
    public DcMotorEx pulleyRight;
    public DcMotorEx joint;
    public boolean pulleyCalibrated;
    public final int JOINT_BOTTOM = 0; //tentative value
    public final int JOINT_MAX = 10000; //tentativevalue
    public final int MAX = 3000;
    public final int TOP = 2750; //tentative value
    public final int MEDIUM = 1770;
    public final int SHORT = 650;
    public final int ABOVE_STACK = 1000;
    public final int GROUND = 125;
    public final int BOTTOM = 10;
    public final int CUPSTACK = 580;
    public int[] CUP_HEIGHTS = {580, 415, 260, 140, 10};

    public final double OPEN = 0.51;
    public final double GRAB = 0.37;
    public final double FULLY_OPEN = .65;
    // final double JOINTUP = 0.26; // tentative values
    //public final double JOINTDOWN = 0.46; //tentative values
    public boolean HOLDING = false;
    public static int ManualArmIncrement = 10;
    public static int ManualJointIncrement =10; //tentative value
    public static double ManualRotatorIncrement = 0.05;
    public static double ManualGrabberIncrement = 0.05;
    public static int PulleyVelocity = 2500;

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    public Outake(){
        teamUtil.log("Constructing Output");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }

    public void init(){
        grabber = hardwareMap.servo.get("grabber");
        rotator = hardwareMap.servo.get("rotator");
        pulleyLeft = hardwareMap.get(DcMotorEx.class, "pulleyLeft");
        pulleyRight = hardwareMap.get(DcMotorEx.class, "pulleyRight");
        joint = hardwareMap.get(DcMotorEx.class, "joint");

        //pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pulleyLeft.setDirection(DcMotorSimple.Direction.REVERSE);//tentative direction
        pulleyCalibrated = false;
        pulleyLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pulleyRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        joint.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        teamUtil.log("Output Initialized");
    }

    public void outputTelemetry(){
        telemetry.addData("Output  ", "grabber:%f joint:%d rotator:%f left pulley:%d right pulley:%d",
                grabber.getPosition(), joint.getCurrentPosition(), rotator.getPosition(), pulleyLeft.getCurrentPosition(), pulleyRight.getCurrentPosition());
    }



    public void calibrate(){//pulley
        //grabber.setPosition(OPEN);
        //joint.setPosition(JOINTDOWN);
        pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyLeft.setPower(-0.1);
        pulleyRight.setPower(-0.1);
        teamUtil.pause(500);
        do {
            long lastLeftPos = pulleyLeft.getCurrentPosition();
            long lastRightPos = pulleyRight.getCurrentPosition();
            teamUtil.pause(250);
            // if things aren't moving, we have stalled for the first time (going down)
            if ((pulleyLeft.getCurrentPosition() == lastLeftPos)&& (pulleyRight.getCurrentPosition()==lastRightPos)) {

                pulleyLeft.setPower(0.1);
                pulleyRight.setPower(0.1);
                teamUtil.pause(500);
                do {
                    lastLeftPos = pulleyLeft.getCurrentPosition();
                    lastRightPos = pulleyRight.getCurrentPosition();
                    teamUtil.pause(250);
                    //if things aren't moving now, we have stalled for the second time (going up)
                    if ((pulleyLeft.getCurrentPosition() == lastLeftPos)&& (pulleyRight.getCurrentPosition()==lastRightPos)) {
                        pulleyLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        pulleyRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        pulleyLeft.setPower(0);
                        pulleyRight.setPower(0);
                        pulleyLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        pulleyRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        pulleyCalibrated = true;
                        log("Calibrating Pulley - Finished");

                    }
                    return;
                } while (true);
            }
            log("Pulley Left Calibrated Encoder:" + lastLeftPos );
            log("Pulley Right Calibrated Encoder:" + lastRightPos );
        } while (true);

    }




    //////////////////////////////////////////////////////////////////////////////////////////

    public void stop(){
        if(HOLDING){
            return;
        }
        HOLDING=true;
        pulleyLeft.setTargetPosition(pulleyLeft.getCurrentPosition());
        pulleyLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleyLeft.setVelocity(3000);

    }



    public void runPulleyUp(){

        log("running arm up");

        int currentPosition= pulleyLeft.getCurrentPosition();
        if(currentPosition+ManualArmIncrement<MAX){
            pulleyLeft.setTargetPosition(currentPosition+ManualArmIncrement);
        }
        pulleyLeft.setVelocity(500);


    }
    public void runPulleyDown(){

        log("running arm up");

        int currentPosition= pulleyLeft.getCurrentPosition();
        if(currentPosition-ManualArmIncrement>BOTTOM){
            pulleyLeft.setTargetPosition(currentPosition-ManualArmIncrement);
        }
        pulleyLeft.setVelocity(500);

    }


    public void openGrabber(){
        grabber.setPosition(OPEN);
    }

    public void closeGrabber(){
        grabber.setPosition(GRAB);
    }

    public void jointUp() {
        log("running arm up");

        int currentJointPosition = joint.getCurrentPosition();
        if(currentJointPosition+ManualArmIncrement<JOINT_MAX){
            joint.setTargetPosition(currentJointPosition+ManualJointIncrement);
        }
        joint.setVelocity(100);
    }

    public void jointDown() {
        log("running arm up");

        int currentJointPosition = joint.getCurrentPosition();
        if(currentJointPosition-ManualArmIncrement>JOINT_BOTTOM){
            joint.setTargetPosition(currentJointPosition-ManualJointIncrement);
        }
        joint.setVelocity(100);
    }

    public void turnRotatorRight(){
        log("Turning Rotator Right");
        double currentRotatorPosition = rotator.getPosition();
        rotator.setPosition(currentRotatorPosition+ManualRotatorIncrement);

    }

    public void turnRotatorLeft(){
        log("Turning Rotator Left");
        double currentRotatorPosition = rotator.getPosition();
        rotator.setPosition(currentRotatorPosition-ManualRotatorIncrement);
    }

    public void manualGrabberOpen(){
        log("Opening Grabber");
        double currentGrabberPosition = grabber.getPosition();
        grabber.setPosition(currentGrabberPosition+ManualGrabberIncrement);
    }

    public void manualGrabberClose(){
        log("Opening Grabber");
        double currentGrabberPosition = grabber.getPosition();
        grabber.setPosition(currentGrabberPosition-ManualGrabberIncrement);
    }


    public void runToBottom(){
        pulleyLeft.setTargetPosition(BOTTOM);
        pulleyLeft.setVelocity(PulleyVelocity);
        //joint.setPosition(JOINTDOWN);
    }

    public void runToGroundJunction(){
        pulleyLeft.setTargetPosition(GROUND);
        pulleyLeft.setVelocity(PulleyVelocity);
        //.setPosition(JOINTDOWN);
    }

    public void runToShort(){
        pulleyLeft.setTargetPosition(SHORT);
        pulleyLeft.setVelocity(PulleyVelocity);
        //joint.setPosition(JOINTUP);
    }

    public void runToMedium(){
        pulleyLeft.setTargetPosition(MEDIUM);
        pulleyLeft.setVelocity(PulleyVelocity);
        //joint.setPosition(JOINTUP);
    }

    public void runToTall(){
        pulleyLeft.setTargetPosition(TOP);
        pulleyLeft.setVelocity(PulleyVelocity);
        //joint.setPosition(JOINTUP);
    }

    public void runToCupStack(){
        pulleyLeft.setTargetPosition(CUPSTACK);
        pulleyLeft.setVelocity(PulleyVelocity);
        //joint.setPosition(JOINTDOWN);
    }
/*
    public void runPulleyUpv1(){
        if (pulley.getCurrentPosition() >= TOP) {//////////TOP NEEDS A VALUE!!!!
            stop();
            return;
        }
        HOLDING=false;
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pulley.setPower(1);

    }

    public void runPulleyDownv1(){
        if (pulley.getCurrentPosition() < BOTTOM) {
            pulley.setVelocity(0);
            return;
        }
        HOLDING=false;
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pulley.setPower(-.75);
    }

 */




}
