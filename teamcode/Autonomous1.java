package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous 1 (very experimental (I mean it))")
@Disabled
public class Autonomous1 extends LinearOpMode
{

    private DcMotor DriveFL;
    private DcMotor DriveFR;
    private DcMotor DriveBL;
    private DcMotor DriveBR;
    private Servo Claw;

    private double ticksPerCm = 1120 / (2 * 4.5 * 3.14159);

    @Override
    public void runOpMode()
    {
        DriveFL = hardwareMap.get(DcMotor.class, "DriveFL");
        DriveFR = hardwareMap.get(DcMotor.class, "DriveFR");
        DriveBL = hardwareMap.get(DcMotor.class, "DriveBL");
        DriveBR = hardwareMap.get(DcMotor.class, "DriveBR");
        Claw = hardwareMap.get(Servo.class, "Claw");
        //DriveLS = hardwareMap.get(DcMotor.class, "DriveLS");

        DriveBL.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveFR.setDirection(DcMotorSimple.Direction.REVERSE);

        DriveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        if (opModeIsActive())
        {
            Claw.setPosition(0.5);

            while (opModeIsActive())
            {
                DriveFL.setTargetPosition(12 * (int) ticksPerCm);
                DriveFR.setTargetPosition(-12 * (int) ticksPerCm);
                DriveBL.setTargetPosition(12 * (int) ticksPerCm);
                DriveBR.setTargetPosition(-12 * (int) ticksPerCm);

                DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                DriveFL.setPower(0.25);
                DriveFR.setPower(0.25);
                DriveBL.setPower(0.25);
                DriveBR.setPower(0.25);

                /*if (!DriveFR.isBusy())
                {
                    sleep(1000);
                    Claw.setPosition(0.75);
                }*/

                telemetry.addData("FR Position: ", DriveFR.getCurrentPosition());
                telemetry.addData("BR Position: ", DriveBR.getCurrentPosition());
                telemetry.addData("FL Position: ", DriveFL.getCurrentPosition());
                telemetry.addData("BL Position: ", DriveBL.getCurrentPosition());
                telemetry.addData("Target Position (FR, but it should be the same for all motors): ", DriveFR.getTargetPosition());
                telemetry.update();
            }
        }
    }
}
