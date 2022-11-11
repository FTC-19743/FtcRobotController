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
    public boolean pulleyCalibrated;
    public final int TOP = 3575; //tentative value
    public final int BOTTOM = 10;
    public final double OPEN = 0.51;
    public final double GRAB = 0.37;
    public boolean HOLDING = false;

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

        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pulley.setDirection(DcMotorSimple.Direction.REVERSE);//tentative direction
        pulleyCalibrated = false;
    }

    public void outputTelemetry(){
        telemetry.addData("Output  ", "grabber:%f pulley:%d",
                grabber.getPosition(), pulley.getCurrentPosition());
    }

    public void calibrate(){//pulley

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
                pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pulleyCalibrated = true;
                log("Calibrating Pulley - Finished");

                return;
            }
            log("Pulley Encoder:" + lastPos );

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
        if (pulley.getCurrentPosition() >= TOP) {//////////TOP NEEDS A VALUE!!!!
            stop();
            return;
        }
        HOLDING=false;
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pulley.setPower(1);

    }

    public void runPulleyDown(){
        if (pulley.getCurrentPosition() < BOTTOM) {
            pulley.setVelocity(0);
            return;
        }
        HOLDING=false;
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pulley.setPower(-.75);
    }

    public void openGrabber(){
        grabber.setPosition(OPEN);
    }

    public void closeGrabber(){
        grabber.setPosition(GRAB);
    }
}
