package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name="AbsoluteEncoderTest", group="Linear Opmode")
public class AbsoluteEncoderTest extends LinearOpMode {
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
    @Override
    public void runOpMode(){
        teamUtil.init(this);
        double p1 = .05;
        double p2 = .8;
        TeamGamepad gamepad;
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        AnalogInput absoluteEncoder;
        Servo servo;
        absoluteEncoder = hardwareMap.analogInput.get("absEn");
        servo = hardwareMap.servo.get("servo");
        waitForStart();
        while(opModeIsActive()){
            double voltage = absoluteEncoder.getVoltage();
            telemetry.addData("raw val", "voltage:  " + Double.toString(voltage));
            telemetry.update();
            gamepad.loop();
            if (gamepad.wasAPressed()){
                if (servo.getPosition() > .7){
                    servo.setPosition(p2);
                    long loopStartTime = System.currentTimeMillis();
                    while(absoluteEncoder.getVoltage() < 2){
                        log("Voltage: "+absoluteEncoder.getVoltage());
                    }
                    log("done in "+(System.currentTimeMillis()-loopStartTime)+" milliseconds");
                }else{
                    servo.setPosition(p1);
                    long loopStartTime = System.currentTimeMillis();
                    while(absoluteEncoder.getVoltage() > 4.4){
                        log("Voltage: "+absoluteEncoder.getVoltage());
                    }
                    log("done in "+(System.currentTimeMillis()-loopStartTime)+" milliseconds");
                }
            }
        }
    }
}
