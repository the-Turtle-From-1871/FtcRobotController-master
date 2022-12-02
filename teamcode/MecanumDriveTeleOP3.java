package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "MecanumDriveTeleOP3 (for testing)")
public class MecanumDriveTeleOP3 extends LinearOpMode {

  private DcMotorEx DriveBL;
  private DcMotorEx DriveFL;
  private DcMotorEx DriveFR;
  private DcMotorEx DriveBR;
  private DcMotorEx DriveLS;
  private Servo Claw;
  private BNO055IMU gyroscopePart;
  private Orientation robotAngle;
  private boolean driven;
  private int wheelRedundancy;
  private double correctionSpeed;

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

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    gyroscopePart = hardwareMap.get(BNO055IMU.class, "imu");
    gyroscopePart.initialize(parameters);

    Claw.setPosition(0.79);

    waitForStart();
    if (opModeIsActive()) {
      DriveBL.setDirection(DcMotorSimple.Direction.REVERSE);
      DriveFL.setDirection(DcMotorSimple.Direction.REVERSE);

      while (opModeIsActive()) {
        Forward = -1 * gamepad1.left_stick_y; //up -1, down 1
        Slide = gamepad1.left_stick_x; //left -1, right 1
        Turn = gamepad1.right_stick_x; //left -1, right 1
        Limiter = JavaUtil.maxOfList(JavaUtil.createListWith(1, Math.abs(Forward) + Math.abs(Slide) + Math.abs(Turn)));
        robotAngle = gyroscopePart.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (gamepad1.right_bumper && wheelRedundancy == 0) {
          DriveFL.setPower(((Forward + Slide + Turn) / Limiter) / 4);
          DriveBL.setPower((((Forward - Slide) + Turn) / Limiter) / 4);
          DriveFR.setPower((((Forward - Slide) - Turn) / Limiter) / 4);
          DriveBR.setPower((((Forward + Slide) - Turn) / Limiter) / 4);
        } else if (gamepad1.left_bumper && wheelRedundancy == 0) {
          DriveFL.setPower(((Forward + Slide + Turn) / Limiter) / 2);
          DriveBL.setPower((((Forward - Slide) + Turn) / Limiter) / 2);
          DriveFR.setPower((((Forward - Slide) - Turn) / Limiter) / 2);
          DriveBR.setPower((((Forward + Slide) - Turn) / Limiter) / 2);
        } else if (wheelRedundancy == 0) {
          DriveFL.setPower(((Forward + Slide + Turn) / Limiter));
          DriveBL.setPower((((Forward - Slide) + Turn) / Limiter));
          DriveFR.setPower((((Forward - Slide) - Turn) / Limiter));
          DriveBR.setPower((((Forward + Slide) - Turn) / Limiter));
        } else if (gamepad1.right_bumper && wheelRedundancy == 1) {
          DriveFL.setPower(((Forward + Slide) * 2 + Turn) / (Limiter + Math.abs(Forward) + Math.abs(Slide)) / 4);
          DriveBL.setPower(((Forward - Slide) * 2 + Turn) / (Limiter + Math.abs(Forward) + Math.abs(Slide)) / 4);
          DriveFR.setPower((((Forward - Slide) - Turn) / Limiter) / 4);
          DriveBR.setPower((((Forward + Slide) - Turn) / Limiter) / 4);
        } else if (gamepad1.left_bumper && wheelRedundancy == 1) {
          DriveFL.setPower(((Forward + Slide) * 2 + Turn) / (Limiter + Math.abs(Forward) + Math.abs(Slide)) / 2);
          DriveBL.setPower(((Forward - Slide) * 2 + Turn) / (Limiter + Math.abs(Forward) + Math.abs(Slide)) / 2);
          DriveFR.setPower((((Forward - Slide) - Turn) / Limiter) / 2);
          DriveBR.setPower((((Forward + Slide) - Turn) / Limiter) / 2);
        } else if (wheelRedundancy == 1) {
          DriveFL.setPower(((Forward + Slide) * 2 + Turn) / (Limiter + Math.abs(Forward) + Math.abs(Slide)));
          DriveBL.setPower(((Forward - Slide) * 2 + Turn) / (Limiter + Math.abs(Forward) + Math.abs(Slide)));
          DriveFR.setPower((((Forward - Slide) - Turn) / Limiter));
          DriveBR.setPower((((Forward + Slide) - Turn) / Limiter));
        } else if (gamepad1.right_bumper && wheelRedundancy == 2) {
          DriveFL.setPower(((Forward + Slide + Turn) / Limiter) / 4);
          DriveBL.setPower((((Forward - Slide) + Turn) / Limiter) / 4);
          DriveFR.setPower(((Forward - Slide) * 2 - Turn) / (Limiter + Math.abs(Forward) + Math.abs(Slide)) / 4);
          DriveBR.setPower(((Forward + Slide) * 2 - Turn) / (Limiter + Math.abs(Forward) + Math.abs(Slide)) / 4);
        } else if (gamepad1.left_bumper && wheelRedundancy == 2) {
          DriveFL.setPower(((Forward + Slide + Turn) / Limiter) / 2);
          DriveBL.setPower((((Forward - Slide) + Turn) / Limiter) / 2);
          DriveFR.setPower(((Forward - Slide) * 2 - Turn) / (Limiter + Math.abs(Forward) + Math.abs(Slide)) / 2);
          DriveBR.setPower(((Forward + Slide) * 2 - Turn) / (Limiter + Math.abs(Forward) + Math.abs(Slide)) / 2);
        } else if (wheelRedundancy == 2) {
          DriveFL.setPower(((Forward + Slide + Turn) / Limiter));
          DriveBL.setPower((((Forward - Slide) + Turn) / Limiter));
          DriveFR.setPower(((Forward - Slide) * 2 - Turn) / (Limiter + Math.abs(Forward) + Math.abs(Slide)));
          DriveBR.setPower(((Forward + Slide) * 2 - Turn) / (Limiter + Math.abs(Forward) + Math.abs(Slide)));
        }

        if (gamepad2.a) {
          Claw.setPosition(0.79);
        } else if (gamepad2.b) {
          Claw.setPosition(0.5);
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
        if (gamepad1.y)
        {
          DriveBL.setPower(-0.9);
          DriveFL.setPower(-0.9);
          DriveBR.setPower(-0.9);
          DriveFR.setPower(-0.9);
          sleep(2000);
          DriveBL.setPower(0);
          DriveFL.setPower(0);
          DriveBR.setPower(0);
          DriveFR.setPower(0);
        }

        //Linear Slide Control/Digital Limiter
        if (DriveLS.getCurrentPosition() >= -150)
          DriveLS.setPower(-1 * gamepad2.right_trigger * 0.95);
        else if (DriveLS.getCurrentPosition() <= -6200)
          DriveLS.setPower(gamepad2.left_trigger * 0.95);
        else
          DriveLS.setPower(gamepad2.left_trigger * 0.95 - gamepad2.right_trigger * 0.95); //Left, up; right, down

        /* if(gamepad2.y)
          DriveLS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); */

        if (gamepad1.dpad_left)
          wheelRedundancy = 1;
        else if (gamepad1.dpad_right)
          wheelRedundancy = 2;

        if (Forward == -1.0 && !gamepad1.right_bumper && !gamepad1.left_bumper)
        {
          driven = true;
          correctionSpeed = 0.5;
        }
        else if (Forward == 1.0 && !gamepad1.right_bumper && !gamepad1.left_bumper)
        {
          driven = true;
          correctionSpeed = -0.5;
        }
        else if (Forward == 0.0 && driven)
        {
          DriveBL.setPower(correctionSpeed);
          DriveFL.setPower(correctionSpeed);
          DriveBR.setPower(correctionSpeed);
          DriveFR.setPower(correctionSpeed);
          sleep(50);
          DriveBL.setPower(0);
          DriveFL.setPower(0);
          DriveBR.setPower(0);
          DriveFR.setPower(0);
          driven = false;
        }

        telemetry.addData("Heading ", robotAngle.firstAngle);
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
        telemetry.addData("LS Position: ", DriveLS.getCurrentPosition());
        telemetry.addData("\n ANGLE CHECK: ", GyroCheck());
        telemetry.update();
      }
    }
  }

  private boolean GyroCheck()
  {
    if(robotAngle.firstAngle > -3 && robotAngle.firstAngle < 3)
    {
      gamepad1.rumble(0.9, 0, 50);
      gamepad2.rumble(0.9, 0, 50);
      return true;
    }
    if(robotAngle.firstAngle > 87 && robotAngle.firstAngle < 93)
    {
      gamepad1.rumble(0.9, 0, 50);
      gamepad2.rumble(0.9, 0, 50);
      return true;
    }
    if(robotAngle.firstAngle > 177 && robotAngle.firstAngle < 183)
    {
      gamepad1.rumble(0.9, 0, 50);
      gamepad2.rumble(0.9, 0, 50);
      return true;
    }
    if(robotAngle.firstAngle < -177 && robotAngle.firstAngle > -183)
    {
      gamepad1.rumble(0.9, 0, 50);
      gamepad2.rumble(0.9, 0, 50);
      return true;
    }
    if(robotAngle.firstAngle < -87 && robotAngle.firstAngle > -93)
    {
      gamepad1.rumble(0.9, 0, 50);
      gamepad2.rumble(0.9, 0, 50);
      return true;
    }
    return false;
  }
}
