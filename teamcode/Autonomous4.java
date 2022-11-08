package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Autonomous 4 (goes into 2nd signal zone only)")
public class Autonomous4 extends LinearOpMode
{

    private DcMotor DriveFL;
    private DcMotor DriveFR;
    private DcMotor DriveBL;
    private DcMotor DriveBR;
    private Servo Claw;
    private DcMotor DriveLS;


    @Override
    public void runOpMode()
    {
        DriveFL = hardwareMap.get(DcMotor.class, "DriveFL");
        DriveFR = hardwareMap.get(DcMotor.class, "DriveFR");
        DriveBL = hardwareMap.get(DcMotor.class, "DriveBL");
        DriveBR = hardwareMap.get(DcMotor.class, "DriveBR");
        Claw = hardwareMap.get(Servo.class, "Claw");
        DriveLS = hardwareMap.get(DcMotor.class, "DriveLS");

        DriveBL.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveFR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive())
        {
            Claw.setPosition(0.5);
            sleep(20);
            DriveLS.setPower(0.25);
            sleep(20);
            DriveLS.setPower(0);

            //while (opModeIsActive())
            //{
                //positive value is forward, negative value is backwards
                DriveBL.setPower(0.5);
                DriveFL.setPower(0.5);
                DriveBR.setPower(0.5);
                DriveFR.setPower(0.5);
                sleep(2000); //AAAAAAAAAAAA
                DriveBL.setPower(0);
                DriveFL.setPower(0);
                DriveBR.setPower(0);
                DriveFR.setPower(0);

                telemetry.addData("FR Position: ", DriveFR.getCurrentPosition());
                telemetry.addData("BR Position: ", DriveBR.getCurrentPosition());
                telemetry.addData("FL Position: ", DriveFL.getCurrentPosition());
                telemetry.addData("BL Position: ", DriveBL.getCurrentPosition());
                telemetry.addData("Target Position (FR, but it should be the same for all motors): ", DriveFR.getTargetPosition());
                telemetry.update();
            //}
        }
    }
}
