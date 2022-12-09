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
    public DcMotorEx pulley;
    public Servo joint;
    public boolean pulleyCalibrated;
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
    public final double JOINTUP = 0.26; // tentative values
    public final double JOINTDOWN = 0.46; //tentative values
    public boolean HOLDING = false;
    public static int ManualArmIncrement = 10;
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
        pulley = hardwareMap.get(DcMotorEx.class, "pulley");
        joint = hardwareMap.servo.get("joint");

        //pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pulley.setDirection(DcMotorSimple.Direction.REVERSE);//tentative direction
        pulleyCalibrated = false;

        teamUtil.log("Output Initialized");
    }

    public void outputTelemetry(){
        telemetry.addData("Output  ", "grabber:%f joint:%f  pulley:%d",
                grabber.getPosition(), joint.getPosition(),  pulley.getCurrentPosition());
    }



    public void calibrate(){//pulley
        grabber.setPosition(OPEN);
        joint.setPosition(JOINTDOWN);
        pulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulley.setPower(-0.1);
        teamUtil.pause(500);
        do {
            long lastPos = pulley.getCurrentPosition();
            teamUtil.pause(250);
            // if things aren't moving, we have stalled
            if ((pulley.getCurrentPosition() == lastPos)) {
                pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pulley.setPower(0);
                pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pulleyCalibrated = true;
                log("Calibrating Pulley - Finished");
                log("Pulley Calibrated Encoder:" + lastPos );
                return;
            }


        } while (true);
    }



    //////////////////////////////////////////////////////////////////////////////////////////

    public void stop(){
        if(HOLDING){
            return;
        }
        HOLDING=true;
        pulley.setTargetPosition(pulley.getCurrentPosition());
        pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulley.setVelocity(3000);

    }

    public void runPulleyUp(){

        log("running arm up");

        int currentPosition= pulley.getCurrentPosition();
        if(currentPosition+ManualArmIncrement<MAX){
            pulley.setTargetPosition(currentPosition+ManualArmIncrement);
        }
        pulley.setVelocity(500);


    }
    public void runPulleyDown(){

        log("running arm up");

        int currentPosition= pulley.getCurrentPosition();
        if(currentPosition-ManualArmIncrement>BOTTOM){
            pulley.setTargetPosition(currentPosition-ManualArmIncrement);
        }
        pulley.setVelocity(500);

    }


    public void openGrabber(){
        grabber.setPosition(OPEN);
    }

    public void closeGrabber(){
        grabber.setPosition(GRAB);
    }

    public void jointUp() {
        joint.setPosition(.28);
        teamUtil.pause(250);
        joint.setPosition(JOINTUP);
    }

    public void jointDown() {joint.setPosition(JOINTDOWN);}




    public void runToBottom(){
        pulley.setTargetPosition(BOTTOM);
        pulley.setVelocity(PulleyVelocity);
        joint.setPosition(JOINTDOWN);
    }

    public void runToGroundJunction(){
        pulley.setTargetPosition(GROUND);
        pulley.setVelocity(PulleyVelocity);
        joint.setPosition(JOINTDOWN);
    }

    public void runToShort(){
        pulley.setTargetPosition(SHORT);
        pulley.setVelocity(PulleyVelocity);
        joint.setPosition(JOINTUP);
    }

    public void runToMedium(){
        pulley.setTargetPosition(MEDIUM);
        pulley.setVelocity(PulleyVelocity);
        joint.setPosition(JOINTUP);
    }

    public void runToTall(){
        pulley.setTargetPosition(TOP);
        pulley.setVelocity(PulleyVelocity);
        joint.setPosition(JOINTUP);
    }

    public void runToCupStack(){
        pulley.setTargetPosition(CUPSTACK);
        pulley.setVelocity(PulleyVelocity);
        joint.setPosition(JOINTDOWN);
    }

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


}
