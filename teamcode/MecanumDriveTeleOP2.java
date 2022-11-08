package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "MecanumDriveTeleOP2 (w/encoders)")
@Disabled
public class MecanumDriveTeleOP2 extends LinearOpMode {
/**
 * This code is very outdated, so it should be revised before use.
 */
  private DcMotorEx DriveBL;
  private DcMotorEx DriveFL;
  private DcMotorEx DriveFR;
  private DcMotorEx DriveBR;
  private Servo Claw;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    float Forward;
    float Slide;
    float Turn;
    double Limiter;
    float Linear_Slide;
    boolean Claw2;

    DriveBL = (DcMotorEx) hardwareMap.get(DcMotor.class, "DriveBL");
    DriveFL = (DcMotorEx) hardwareMap.get(DcMotor.class, "DriveFL");
    DriveFR = (DcMotorEx) hardwareMap.get(DcMotor.class, "DriveFR");
    DriveBR = (DcMotorEx) hardwareMap.get(DcMotor.class, "DriveBR");
    Claw = hardwareMap.get(Servo.class, "Claw");

    DriveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    DriveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    DriveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    DriveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    DriveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    DriveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    DriveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    DriveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();
    if (opModeIsActive()) {
      DriveBL.setDirection(DcMotorSimple.Direction.REVERSE);
      //DriveFR.setDirection(DcMotorSimple.Direction.REVERSE);
      while (opModeIsActive()) {
        Forward = gamepad1.left_stick_y;
        Slide = gamepad1.left_stick_x;
        Turn = gamepad1.right_stick_x;
        Limiter = JavaUtil.maxOfList(JavaUtil.createListWith(1, Math.abs(Forward) + Math.abs(Slide) + Math.abs(Turn)));
        Linear_Slide = gamepad2.right_stick_y;
        Claw2 = gamepad2.left_bumper;
        //R: 4.5cm
        if (gamepad1.right_bumper) {
          DriveFL.setVelocity(((Forward + Slide + Turn) / Limiter) / 4);
          DriveBL.setVelocity((((-Forward - Slide) + Turn) / Limiter) / 4);
          DriveFR.setVelocity((((Forward - Slide) - Turn) / Limiter) / 4);
          DriveBR.setVelocity((((-Forward + Slide) - Turn) / Limiter) / 4);
        } else if (gamepad1.left_bumper) {
          DriveFL.setVelocity((Forward + Slide + Turn) / Limiter);
          DriveBL.setVelocity(((-Forward - Slide) + Turn) / Limiter);
          DriveFR.setVelocity(((Forward - Slide) - Turn) / Limiter);
          DriveBR.setVelocity(((-Forward + Slide) - Turn) / Limiter);
        } else {
          DriveFL.setVelocity(((Forward + Slide + Turn) / Limiter) / 2);
          DriveBL.setVelocity((((-Forward - Slide) + Turn) / Limiter) / 2);
          DriveFR.setVelocity((((Forward - Slide) - Turn) / Limiter) / 2);
          DriveBR.setVelocity((((-Forward + Slide) - Turn) / Limiter) / 2);
        }
        if (gamepad1.a) {
          Claw.setPosition(0.2);
        } else if (gamepad1.b) {
          Claw.setPosition(0.8);
        } else {
          Claw.setPosition(0);
        }
        DriveFR.setPower(Linear_Slide);
        telemetry.update();
      }
    }
  }
}
