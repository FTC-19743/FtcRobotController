package org.firstinspires.ftc.teamcode.assemblies;
/*
This is the class that has the carousel spinner in it, and has the code to run it
 */


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class CarouselSpinner {


    HardwareMap hardwareMap;
    public DcMotor spinner;


    public CarouselSpinner(){
        teamUtil.log("Constructing Spinner");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
    }


    public void init(){
        teamUtil.log("Initializing Spinner");
        // setting the direction of the spinner, different for each alliance
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        if(teamUtil.alliance == teamUtil.Alliance.RED) {
            spinner.setDirection(DcMotorSimple.Direction.FORWARD);
        }else{
            spinner.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void on(double power) {
        spinner.setPower(power);
    }

    public void off(){
        spinner.setPower(0);
    }

    public void spinOnce(){
        int position = spinner.getCurrentPosition();
        int amountPerTurn = 1500;// change this number, this is how many degrees it will take for the spinner to spin the carousel once
        spinner.setTargetPosition(position + amountPerTurn);
    }

    public void spinAll(){
        for (int i = 0; i > 10; i++) {
            spinOnce();
            off();
            teamUtil.pause(1000);
        }
    }
}