package org.firstinspires.ftc.teamcode.assemblies;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class bottomColorSensor {
    private ColorSensor colorSensor;
    private int matValueRed;
    private int matValueBlue;
    double WHITE_THRESHOLD = 500;

    public enum TapeColor {RED, BLUE, WHITE, NONE}

    public bottomColorSensor(ColorSensor theColorSensor) {
        colorSensor = theColorSensor;
    }

    public float getAlpha(){
        return colorSensor.alpha();
    }

    public void calibrate() {//Call this when the light sensor is over empty mat
        matValueBlue=colorSensor.blue();
        matValueRed=colorSensor.red();
    }

    public TapeColor getColor() {
        if (onWhite()) {
            return TapeColor.WHITE;  // Test for white first since white will also read red or blue...
        } else if (onRed()) {
            return TapeColor.RED;
        } else if (onBlue()) {
            return TapeColor.BLUE;
        } else  {
            return TapeColor.NONE;
        }
    }

    public boolean isOnTape(){
        return onBlue() || onRed() || onWhite();

    }
    public boolean onBlue() {
        return colorSensor.blue() > matValueBlue * 1.5;
    }
    public boolean onRed() {
        return colorSensor.red() > matValueBlue * 1.5;
    }
    public boolean onWhite() {
        return colorSensor.alpha() > WHITE_THRESHOLD;
    }

    public int redValue() {
        return colorSensor.red();
    }
    public int blueValue() {
        return colorSensor.blue();
    }
    public int greenValue() {
        return colorSensor.green();
    }




}







