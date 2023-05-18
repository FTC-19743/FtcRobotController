package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name="AbsoluteEncoderTest", group="Linear Opmode")
public class AbsoluteEncoderTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        AnalogInput absoluteEncoder;
        absoluteEncoder = hardwareMap.analogInput.get("absEn");
        waitForStart();
        while(opModeIsActive()){
            double voltage = absoluteEncoder.getVoltage();
            telemetry.addData("raw val", "voltage:  " + Double.toString(voltage));
            telemetry.update();
        }
    }
}
