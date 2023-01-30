package org.firstinspires.ftc.teamcode.testCode;



        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.LED;
        import com.qualcomm.robotcore.hardware.DigitalChannel;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Disabled
@TeleOp(name="Test LEDs")

public class TestLED extends LinearOpMode {
    private DigitalChannel red1LED, red2LED, green1LED, green2LED;
    private DigitalChannel greenLED;


    @Override
    public void runOpMode() {
        teamUtil.init(this);
        // construction code
        red1LED = hardwareMap.get(DigitalChannel.class, "red1");
        green1LED = hardwareMap.get(DigitalChannel.class, "green1");
        red2LED = hardwareMap.get(DigitalChannel.class, "red2");
        green2LED = hardwareMap.get(DigitalChannel.class, "green2");

        // initialization code
        red1LED.setMode(DigitalChannel.Mode.OUTPUT);
        green1LED.setMode(DigitalChannel.Mode.OUTPUT);
        red2LED.setMode(DigitalChannel.Mode.OUTPUT);
        green2LED.setMode(DigitalChannel.Mode.OUTPUT);
        green1LED.setState(true); // both set to true is "off"
        red1LED.setState(true);
        green2LED.setState(true);
        red2LED.setState(true);

        // Wait for the play button to be pressed
        waitForStart();


        // Loop while the Op Mode is running
        while (opModeIsActive()) { //
            red1LED.setState(true);
            green1LED.setState(false);
            red2LED.setState(false);
            green2LED.setState(true);
            teamUtil.pause(500);
            red1LED.setState(false);
            green1LED.setState(true);
            red2LED.setState(true);
            green2LED.setState(false);
            teamUtil.pause(500);
            red1LED.setState(false); // both set to false is "amber"
            green1LED.setState(false);
            red2LED.setState(false);
            green2LED.setState(false);
            teamUtil.pause(500);
            red1LED.setState(true); // both set to true is "off"
            green1LED.setState(true);
            red2LED.setState(true);
            green2LED.setState(true);
            teamUtil.pause(500);
        }
    }
}