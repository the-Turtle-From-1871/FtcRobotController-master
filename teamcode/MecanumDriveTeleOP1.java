package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MecanumDriveTeleOP1 (operational)")
public class MecanumDriveTeleOP1 extends LinearOpMode {

  private DcMotorEx DriveBL;
  private DcMotorEx DriveFL;
  private DcMotorEx DriveFR;
  private DcMotorEx DriveBR;
  private DcMotorEx DriveLS;
  private Servo Claw;
  double servoOpen = 1;
  double servoClose = 0.1;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    float Forward;
    float Slide;
    float Turn;
    double Limiter;

    DriveBL = hardwareMap.get(DcMotorEx.class, "DriveBL");
    DriveFL = hardwareMap.get(DcMotorEx.class, "DriveFL");
    DriveFR = hardwareMap.get(DcMotorEx.class, "DriveFR");
    DriveBR = hardwareMap.get(DcMotorEx.class, "DriveBR");
    DriveLS = hardwareMap.get(DcMotorEx.class, "DriveLS");
    Claw = hardwareMap.get(Servo.class, "Claw");

    Claw.setPosition(servoClose);

    DriveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    DriveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    DriveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    DriveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();
    if (opModeIsActive()) {
      DriveBL.setDirection(DcMotorSimple.Direction.REVERSE);
      DriveFL.setDirection(DcMotorSimple.Direction.REVERSE);

      while (opModeIsActive()) {
        Forward = -1 * gamepad1.left_stick_y; //up -1, down 1
        Slide = gamepad1.left_stick_x; //left -1, right 1
        Turn = gamepad1.right_stick_x; //left -1, right 1
        Limiter = JavaUtil.maxOfList(JavaUtil.createListWith(1, Math.abs(Forward) + Math.abs(Slide) + Math.abs(Turn)));
        if (gamepad1.right_bumper) {
          DriveFL.setPower(((Forward + Slide + Turn) / Limiter) / 4);
          DriveBL.setPower((((Forward - Slide) + Turn) / Limiter) / 4);
          DriveFR.setPower((((Forward - Slide) - Turn) / Limiter) / 4);
          DriveBR.setPower((((Forward + Slide) - Turn) / Limiter) / 4);
        } else if (gamepad1.left_bumper) {
          DriveFL.setPower(((Forward + Slide + Turn) / Limiter) / 2);
          DriveBL.setPower((((Forward - Slide) + Turn) / Limiter) / 2);
          DriveFR.setPower((((Forward - Slide) - Turn) / Limiter) / 2);
          DriveBR.setPower((((Forward + Slide) - Turn) / Limiter) / 2);
        } else {
          DriveFL.setPower(((Forward + Slide + Turn) / Limiter));
          DriveBL.setPower((((Forward - Slide) + Turn) / Limiter));
          DriveFR.setPower((((Forward - Slide) - Turn) / Limiter));
          DriveBR.setPower((((Forward + Slide) - Turn) / Limiter));
          /*
           * Fl -1, forwards; updated, forwards
           * BL -1, backwards; updated, forwards
           * FR -1, backwards; updated, forwards
           * BR -1, forwards; updated, forwards
           * on else statement: up/down works, left/right works, turn works
           * */
        }

        if (gamepad2.a) {
          Claw.setPosition(servoOpen);
        } else if (gamepad2.b) {
          Claw.setPosition(servoClose);
        }

        if (gamepad1.a) {
          DriveBL.setPower(1);
          DriveFL.setPower(1);
          DriveBR.setPower(-1);
          DriveFR.setPower(-1);
          sleep(650);
          DriveBL.setPower(0);
          DriveFL.setPower(0);
          DriveBR.setPower(0);
          DriveFR.setPower(0);
        }
        if (gamepad1.b) {
          DriveBL.setPower(1);
          DriveFL.setPower(1);
          DriveBR.setPower(-1);
          DriveFR.setPower(-1);
          sleep(325);
          DriveBL.setPower(0);
          DriveFL.setPower(0);
          DriveBR.setPower(0);
          DriveFR.setPower(0);
        }
        if (gamepad1.x) {
          DriveBL.setPower(-1);
          DriveFL.setPower(-1);
          DriveBR.setPower(1);
          DriveFR.setPower(1);
          sleep(325);
          DriveBL.setPower(0);
          DriveFL.setPower(0);
          DriveBR.setPower(0);
          DriveFR.setPower(0);
        }

        /**
         * Linear Slide Control/Digital Limiter
         */
        if (DriveLS.getCurrentPosition() >= -150)
          DriveLS.setPower(-1 * gamepad2.right_trigger * 0.95);
        else if (DriveLS.getCurrentPosition() <= -6100)
          DriveLS.setPower(gamepad2.left_trigger * 0.95);
        else
          DriveLS.setPower(gamepad2.left_trigger * 0.95 - gamepad2.right_trigger * 0.95); //Left, up; right, down

        if(gamepad2.y)
        {
          DriveLS.setPower(0.5);
          sleep(400);
          DriveLS.setPower(0);

          DriveLS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("FR Angular Velocity: ", DriveFR.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("FL Angular Velocity: ", DriveFL.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("BR Angular Velocity: ", DriveBR.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("BL Angular Velocity: ", DriveBL.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("FR Position: ", DriveFR.getCurrentPosition());
        telemetry.addData("FL Position: ", DriveFL.getCurrentPosition());
        telemetry.addData("BR Position: ", DriveBR.getCurrentPosition());
        telemetry.addData("BL Position: ", DriveBL.getCurrentPosition());
        telemetry.addData("Left Stick X: ", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y: ", gamepad1.left_stick_y);
        telemetry.addData("Claw Position: ", Claw.getPosition());
        telemetry.addData("Claw Target Open/Close: ", gamepad1.a);
        telemetry.addData("LS Position: ", DriveLS.getCurrentPosition());
        telemetry.update();
      }
    }
  }
}
