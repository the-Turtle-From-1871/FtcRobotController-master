package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MecanumDriveTeleOP3 (test)")
public class MecanumDriveTeleOP3 extends LinearOpMode {

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
    boolean Claw;

    DriveBL = (DcMotorEx) hardwareMap.get(DcMotor.class, "DriveBL");
    DriveFL = (DcMotorEx) hardwareMap.get(DcMotor.class, "DriveFL");
    DriveFR = (DcMotorEx) hardwareMap.get(DcMotor.class, "DriveFR");
    DriveBR = (DcMotorEx) hardwareMap.get(DcMotor.class, "DriveBR");
    Claw = hardwareMap.get(Servo.class, "Claw");

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
        Claw = gamepad2.a;
        if (gamepad1.right_bumper) {
          DriveFL.setPower(((Forward + Slide + Turn) / Limiter) / 4);
          DriveBL.setPower((((-Forward - Slide) + Turn) / Limiter) / 4);
          DriveFR.setPower((((Forward - Slide) - Turn) / Limiter) / 4);
          DriveBR.setPower((((-Forward + Slide) - Turn) / Limiter) / 4);
        } else if (gamepad1.left_bumper) {
          DriveFL.setPower((Forward + Slide + Turn) / Limiter);
          DriveBL.setPower(((-Forward - Slide) + Turn) / Limiter);
          DriveFR.setPower(((Forward - Slide) - Turn) / Limiter);
          DriveBR.setPower(((-Forward + Slide) - Turn) / Limiter);
        } else {
          DriveFL.setPower(((Forward + Slide + Turn) / Limiter) / 2);
          DriveBL.setPower((((-Forward - Slide) + Turn) / Limiter) / 2);
          DriveFR.setPower((((Forward - Slide) - Turn) / Limiter) / 2);
          DriveBR.setPower((((-Forward + Slide) - Turn) / Limiter) / 2);
        }

        if (gamepad1.a) {
          Claw.setPosition(0.2);
        } else if (gamepad1.b) {
          Claw.setPosition(0.8);
        } else {
          Claw.setPosition(0);
        }

        DriveFR.setPower(Linear_Slide);
        telemetry.addData("FR Angular Velocity: ", DriveFR.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("FL Angular Velocity: ", DriveFL.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("BR Angular Velocity: ", DriveBR.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("BL Angular Velocity: ", DriveBL.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Left Stick X: ", gamepad1.left_stick_x);
        telemetry.addData("Left Stick X: ", gamepad1.left_stick_y);
        telemetry.update();
      }
    }
  }
}
