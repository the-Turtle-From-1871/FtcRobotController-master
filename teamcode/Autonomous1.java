package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Autonomous 1 (very experimental (I mean it))")
public class Autonomous1 extends LinearOpMode
{

    private DcMotor DriveFL;
    private DcMotor DriveFR;
    private DcMotor DriveBL;
    private DcMotor DriveBR;
    private Servo Claw;
    private DcMotor DriveLS;

    private double ticksPerCm = 1120 / (2 * 4.5 * 3.14159);

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

        DriveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        Claw.setPosition(0.5);
        sleep(20);
        DriveLS.setPower(-0.25);
        sleep(40);
        DriveLS.setPower(0);

        if (opModeIsActive())
        {
            //Forward +, Backwards -
            DriveFL.setTargetPosition(30);
            DriveFR.setTargetPosition(30);
            DriveBL.setTargetPosition(30);
            DriveBR.setTargetPosition(30);

            DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            DriveFL.setPower(0.25);
            DriveFR.setPower(0.25);
            DriveBL.setPower(0.25);
            DriveBR.setPower(0.25);

            while (opModeIsActive() && DriveFR.isBusy())
            {
                telemetry.addData("FR: ", DriveFR.getCurrentPosition());
                telemetry.addData("FL: ", DriveFL.getCurrentPosition());
                telemetry.addData("BR: ", DriveBR.getCurrentPosition());
                telemetry.addData("BL: ", DriveBL.getCurrentPosition());
                telemetry.update();
            }

            DriveFL.setPower(0);
            DriveFR.setPower(0);
            DriveBL.setPower(0);
            DriveBR.setPower(0);

            DriveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
