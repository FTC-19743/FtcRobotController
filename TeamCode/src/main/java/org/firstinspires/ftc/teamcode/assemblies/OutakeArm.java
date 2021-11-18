package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
public class OutakeArm {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public DcMotorEx armMotor;
    public CRServo spinnerServo;
    public static int Ground = 0;
    public static int Level1 = 0;
    public static int Level2 = 0;
    public static int Level3 = 0;
    public static int ArmSpeed = 400;

    public OutakeArm(){
        teamUtil.log("Constructing Outake Arm");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void init(){
        teamUtil.log("Initializing Arm");
        armMotor = hardwareMap.get(DcMotorEx.class, "outake_arm");
        spinnerServo = hardwareMap.get(CRServo.class,"outake_spinner");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void writeTelemetry(){
        telemetry.addData("Outake","Arm Position:%d",armMotor.getCurrentPosition());
    }

    public void resetArm(){
        armMotor.setPower(-0.1);
        int encoderPosition = armMotor.getTargetPosition();
        teamUtil.pause(500);
        while(armMotor.getTargetPosition()!=encoderPosition){
            encoderPosition = armMotor.getTargetPosition();
            teamUtil.pause(100);
        }
        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    public void spinnerIntake(){
        spinnerServo.setPower(1);
    }

    public void spinnerOutput(){
        spinnerServo.setPower(-1);
    }

    public void spinnerStop(){
        spinnerServo.setPower(0);
    }





}
