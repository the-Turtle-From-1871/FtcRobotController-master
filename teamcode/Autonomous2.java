package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Autonomous 2 (motor diagnostic)")
@Disabled
public class Autonomous2 extends LinearOpMode
{

    private DcMotor DriveFL;
    private DcMotor DriveFR;
    private DcMotor DriveBL;
    private DcMotor DriveBR;

    @Override
    public void runOpMode()
    {
        DriveFL = hardwareMap.get(DcMotor.class, "DriveFL");
        DriveFR = hardwareMap.get(DcMotor.class, "DriveFR");
        DriveBL = hardwareMap.get(DcMotor.class, "DriveBL");
        DriveBR = hardwareMap.get(DcMotor.class, "DriveBR");

        /*DriveBL.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveFR.setDirection(DcMotorSimple.Direction.REVERSE);*/

        waitForStart();

        if (opModeIsActive())
        {

            while (opModeIsActive())
            {

                DriveFL.setPower(0.25);
                sleep(1000);
                DriveFL.setPower(0);
                DriveFR.setPower(0.25);
                sleep(1000);
                DriveFR.setPower(0);
                DriveBL.setPower(0.25);
                sleep(1000);
                DriveBL.setPower(0);
                DriveBR.setPower(0.25);
                sleep(1000);
                DriveBR.setPower(0);

            }
        }

        telemetry.update();
    }
}
