package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class IntakeOutputWheel {
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    HardwareMap hardwareMap;


    private DcMotor intakeMotor = null;
    private DcMotor outputMotor = null;

    public void initialize() {

        teamUtil.log("Initializing Intake");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        teamUtil.log("Initializing Output");

        outputMotor = hardwareMap.get(DcMotor.class, "output_motor");
        outputMotor.setDirection(DcMotor.Direction.FORWARD);

    }
    public void intakeGo(double speed){
        intakeMotor.setPower(speed);
        }


    public void intakeStop(){
        intakeMotor.setPower(0);
    }

    public void intakeRunUntilResistance(double speed){
        log("Empty For Now");
    }
    public void outputGo(double speed){
        outputMotor.setPower(speed);
    }


    public void outputStop(){
        outputMotor.setPower(0);
    }



}
