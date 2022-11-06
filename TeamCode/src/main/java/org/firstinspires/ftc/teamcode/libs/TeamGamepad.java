package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeamGamepad {
    Gamepad gamepad;
    Telemetry telemetry;

    boolean aWasPressed = false;
    boolean aWasPressedLastTime = false;
    boolean aBumpToDo = true;
    boolean bWasPressed = false;
    boolean bWasPressedLastTime = false;
    boolean bBumpToDo = true;
    boolean xWasPressed = false;
    boolean xWasPressedLastTime = false;
    boolean xBumpToDo = true;
    boolean yWasPressed = false;
    boolean yWasPressedLastTime = false;
    boolean yBumpToDo = true;
    boolean upWasPressed = false;
    boolean upWasPressedLastTime = false;
    boolean upBumpToDo = true;
    boolean downWasPressed = false;
    boolean downWasPressedLastTime = false;
    boolean downBumpToDo = true;
    boolean rightWasPressed = false;
    boolean rightWasPressedLastTime = false;
    boolean rightBumpToDo = true;
    boolean leftWasPressed = false;
    boolean leftWasPressedLastTime = false;
    boolean leftBumpToDo = true;

    public TeamGamepad(){

    }
    public void initilize(boolean gamepad1){
        if(gamepad1) {
            gamepad = teamUtil.theOpMode.gamepad1;
        }else{
            gamepad = teamUtil.theOpMode.gamepad2;
        }
        telemetry = teamUtil.telemetry;
    }
    public void loop(){
        aWasPressedLastTime = aWasPressed;
        aWasPressed = gamepad.a;
        bWasPressedLastTime = bWasPressed;
        bWasPressed = gamepad.b;
        xWasPressedLastTime = xWasPressed;
        xWasPressed = gamepad.x;
        yWasPressedLastTime = yWasPressed;
        yWasPressed = gamepad.y;
        downWasPressedLastTime = downWasPressed;
        downWasPressed = gamepad.dpad_down;
        upWasPressedLastTime = upWasPressed;
        upWasPressed = gamepad.dpad_up;
        leftWasPressedLastTime = leftWasPressed;
        leftWasPressed = gamepad.dpad_left;
        rightWasPressedLastTime = rightWasPressed;
        rightWasPressed = gamepad.dpad_right;
        if (aWasPressed == false && aWasPressedLastTime == true) {
            aBumpToDo = true;
        }
        if (bWasPressed == false && bWasPressedLastTime == true) {
            bBumpToDo = true;
        }
        if (xWasPressed == false && xWasPressedLastTime == true) {
            xBumpToDo = true;
        }
        if (yWasPressed == false && yWasPressedLastTime == true) {
            yBumpToDo = true;
        }
        if (upWasPressed == false && upWasPressedLastTime == true) {
            upBumpToDo = true;
        }
        if (downWasPressed == false && downWasPressedLastTime == true) {
            downBumpToDo = true;
        }
        if (leftWasPressed == false && leftWasPressedLastTime == true) {
            leftBumpToDo = true;
        }
        if (rightWasPressed == false && rightWasPressedLastTime == true) {
            rightBumpToDo = true;
        }
    }
    public boolean wasAPressed(){
        if(aBumpToDo) {
            aBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasBPressed(){
        if(bBumpToDo) {
            bBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasXPressed(){
        if(xBumpToDo) {
            xBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasYPressed(){
        if(yBumpToDo) {
            yBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasUpPressed(){
        if(upBumpToDo) {
            upBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasDownPressed(){
        if(downBumpToDo) {
            downBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasRightPressed(){
        if(rightBumpToDo) {
            rightBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasLeftPressed(){
        if(leftBumpToDo) {
            leftBumpToDo = false;
            return true;
        }
        return false;
    }

}

