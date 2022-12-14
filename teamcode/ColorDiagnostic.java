package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.lang.String.*;

@TeleOp(name="Color Diagnostic")
public class ColorDiagnostic extends LinearOpMode
{
    private ColorSensor cSensor;

    @Override
    public void runOpMode()
    {
        cSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        waitForStart();

        while(opModeIsActive())
        {
            //cSensor.enableLed(false);

            if(cSensor.argb() > 1664568732 && cSensor.argb() < 1764568732)
            {
                //do something
            }
            else if(cSensor.argb() > -1906161634 && cSensor.argb() < -1706161634)
            {
                //do something
            }
            else if(cSensor.argb() > 1462581032 && cSensor.argb() < 1606161634)
            {
                //do something
            }



            telemetry.addData("R ", cSensor.red());
            telemetry.addData("G ", cSensor.green());
            telemetry.addData("B ", cSensor.blue());
            telemetry.addData("RGB ", cSensor.argb());
            telemetry.addData("Light ", cSensor.alpha());
            telemetry.update();
            //https://docs.revrobotics.com/color-sensor/color-sensor-v2/application-examples
            /*
            * Control: 44, 39, 32, 33619968/336620224, 107
            * Green: 952-955, 2008-20010, 1126/1127, 16645687324, 3782
            * Orange: 3520, 1664, -1806161634, 5948-6010
            * Purple: 1416, 981, 1661, 1562581032, 3712-3742
            * */
        }
    }
}
