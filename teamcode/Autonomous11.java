/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Autonomous 11 (Experimental Slide)")
public class Autonomous11 extends LinearOpMode {

    private DcMotor DriveFL;
    private DcMotor DriveFR;
    private DcMotor DriveBL;
    private DcMotor DriveBR;
    private Servo Claw;
    private DcMotor DriveLS;
    private BNO055IMU gyroscopePart;
    private Orientation robotAngle;
    double servoOpen = 1;
    double servoClose = 0.2;

    private double gearRatio = 2.0;
    private int ticksPerCm = 1120 / (int)(2 * 4.5 * 3.14159);
    private int ticksPerIn = 1120 / (int) gearRatio / (int)(2 * 2 * 3.14159); //code for gear ratio is okay because it's a whole number, but might be sketchy for real doubles


    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "StatesModel";

    private static final String[] LABELS = {
            "1 Fish",
            "2 Octopus",
            "3 Crab"
    };
    //CAMERA: 16cm above ground, ≈27cm from hypothetical wall

    private static final String VUFORIA_KEY =
            "Aayj4Lv/////AAABmYRn8Pzs30jsqzyrQiM8vWIONNlyZZlnpH0SP4vvY8hkMgt3coX5aUc4S2i9ACLh17h2tqHsoLs3p5cGYBoOns0p5KWW58K55/yGt3tDZNeC0xLLCKotCiS4QE/ZbBfbFC4+RlRlwK6I2K7xUn/c+XgpYVxNn/xQqeZS/EYfeWMFulm5+Rs4vSNa2f4+T35Hoq4R7RUd77kQI4Gmhji+UrffCfx+ixEPgluWc2EYgWMBFpsqopXLaX8Gf1GUArdCAh4Att/v18/s5bMGP/mt0zi76SUv5Htcy30V3yjEcdfZNUAHsDUAw/mSInAAt/s/vZiokKvRiXe1uKDz2OrhFeirUYuvlxHwR6LpKSMTxu0r";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        DriveFL = hardwareMap.get(DcMotor.class, "DriveFL");
        DriveFR = hardwareMap.get(DcMotor.class, "DriveFR");
        DriveBL = hardwareMap.get(DcMotor.class, "DriveBL");
        DriveBR = hardwareMap.get(DcMotor.class, "DriveBR");
        Claw = hardwareMap.get(Servo.class, "Claw");
        DriveLS = hardwareMap.get(DcMotor.class, "DriveLS");

        DriveBR.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveFR.setDirection(DcMotorSimple.Direction.REVERSE);

        DriveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters gyroscopeParameters = new BNO055IMU.Parameters();
        gyroscopePart = hardwareMap.get(BNO055IMU.class, "imu");
        gyroscopePart.initialize(gyroscopeParameters);

        robotAngle = gyroscopePart.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addLine(gyroscopePart.isSystemCalibrated() ? "System Calibrated" : "System Not Calibrated");
        telemetry.addLine(gyroscopePart.isGyroCalibrated() ? "Gyroscope Calibrated" : "Gyroscope Not Calibrated");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        Claw.setPosition(servoOpen);

        waitForStart();

        /* Measurements were made with the robot at 13.56 volts, on the back right corner of the starting square,
         the silver edge of the right wheels right on top of the crack where the foam tiles intersect and the back
         wheels just barely touching the back wall (test w/ hand because walls are inconsistent) */

        if (opModeIsActive()) {

            Claw.setPosition(servoClose);
            sleep(10);
            DriveLS.setPower(-0.25);
            sleep(500);
            DriveLS.setPower(0);

            DriveLS.setTargetPosition(-6300);
            DriveLS.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (opModeIsActive()) {

                if (tfod != null && opModeIsActive()) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            if (recognition.getLabel().equals("2 Octopus") && recognition.getConfidence() > 0.60)
                            {
                                //positive value is forward, negative value is backwards
                                setDriveForward(5);

                                DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                                driveToLinearPos();

                                setDriveRight(29);

                                driveToSidePos();

                                setDriveForward(25);

                                driveToLinearPos();

                                setAngleTurn();

                                driveToBlindPos();

                                gyroCheck(true);

                                DriveLS.setPower(-0.95);
                                while(DriveLS.isBusy())
                                {
                                    telemetry.addData("Raising Slide: ", DriveLS.getCurrentPosition());
                                    telemetry.update();
                                }
                                DriveLS.setPower(0);

                                setDriveForward(16);

                                driveToAngledPos();

                                Claw.setPosition(servoOpen);

                                setDriveBackward(16);

                                driveToAngledPos();

                                DriveLS.setTargetPosition(-400);
                                DriveLS.setPower(0.95);
                                while(DriveLS.isBusy())
                                {
                                    telemetry.addData("Lowering Slide: ", DriveLS.getCurrentPosition());
                                    telemetry.update();
                                }
                                DriveLS.setPower(0);

                                setAngleTurnBack();

                                driveToBlindPos();

                                gyroCheck(false);

                                setDriveForward(29);

                                driveToLinearPos();

                                setDriveLeft(38);

                                driveToSidePos();

                                DriveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                sleep(30000);
                            }
                            else if (recognition.getLabel().equals("3 Crab") && recognition.getConfidence() > 0.60)
                            {
                                setDriveForward(5);

                                DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                                driveToLinearPos();

                                setDriveRight(29);

                                driveToSidePos();

                                setDriveForward(25);

                                driveToLinearPos();

                                setAngleTurn();

                                driveToBlindPos();

                                gyroCheck(true);

                                DriveLS.setPower(-0.95);
                                while(DriveLS.isBusy())
                                {
                                    telemetry.addData("Raising Slide: ", DriveLS.getCurrentPosition());
                                    telemetry.update();
                                }
                                DriveLS.setPower(0);

                                setDriveForward(16);

                                driveToAngledPos();

                                Claw.setPosition(servoOpen);

                                setDriveBackward(16);

                                driveToAngledPos();

                                DriveLS.setTargetPosition(-400);
                                DriveLS.setPower(0.95);
                                while(DriveLS.isBusy())
                                {
                                    telemetry.addData("Lowering Slide: ", DriveLS.getCurrentPosition());
                                    telemetry.update();
                                }
                                DriveLS.setPower(0);

                                setAngleTurnBack();

                                driveToBlindPos();

                                gyroCheck(false);

                                DriveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                sleep(30000);
                            }
                            else if (recognition.getLabel().equals("1 Fish") && recognition.getConfidence() > 0.74)
                            {
                                setDriveForward(5);

                                DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                                driveToLinearPos();

                                setDriveRight(29);

                                driveToSidePos();

                                setDriveForward(25);

                                driveToLinearPos();

                                setAngleTurn();

                                driveToBlindPos();

                                gyroCheck(true);

                                DriveLS.setPower(-0.95);
                                while(DriveLS.isBusy())
                                {
                                    telemetry.addData("Raising Slide: ", DriveLS.getCurrentPosition());
                                    telemetry.update();
                                }
                                DriveLS.setPower(0);

                                setDriveForward(16);

                                driveToAngledPos();

                                Claw.setPosition(servoOpen);

                                setDriveBackward(16);

                                driveToAngledPos();

                                DriveLS.setTargetPosition(-400);
                                DriveLS.setPower(0.95);
                                while(DriveLS.isBusy())
                                {
                                    telemetry.addData("Lowering Slide: ", DriveLS.getCurrentPosition());
                                    telemetry.update();
                                }
                                DriveLS.setPower(0);

                                setAngleTurnBack();

                                driveToBlindPos();

                                gyroCheck(false);

                                setDriveForward(30);

                                driveToLinearPos();

                                setDriveLeft(74);

                                driveToSidePos();

                                DriveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                sleep(30000);
                            }
                            else if(getRuntime() > 15000)
                            {
                                setDriveForward(5);

                                DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                                driveToLinearPos();

                                setDriveRight(29);

                                driveToSidePos();

                                setDriveForward(25);

                                driveToLinearPos();

                                setAngleTurn();

                                driveToBlindPos();

                                gyroCheck(true);

                                DriveLS.setPower(-0.95);
                                while(DriveLS.isBusy())
                                {
                                    telemetry.addData("Raising Slide: ", DriveLS.getCurrentPosition());
                                    telemetry.update();
                                }
                                DriveLS.setPower(0);

                                setDriveForward(16);

                                driveToAngledPos();

                                Claw.setPosition(servoOpen);

                                setDriveBackward(16);

                                driveToAngledPos();

                                DriveLS.setTargetPosition(-400);
                                DriveLS.setPower(0.95);
                                while(DriveLS.isBusy())
                                {
                                    telemetry.addData("Lowering Slide: ", DriveLS.getCurrentPosition());
                                    telemetry.update();
                                }
                                DriveLS.setPower(0);

                                setAngleTurnBack();

                                driveToBlindPos();

                                gyroCheck(false);

                                DriveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                DriveLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                sleep(30000);
                            }

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }
                        robotAngle = gyroscopePart.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        telemetry.addData("Heading ", robotAngle.firstAngle);
                        telemetry.update();
                    }
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.60f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }

    private void setDriveForward(int inchesForward)
    {
        if(opModeIsActive())
        {
            DriveFR.setTargetPosition(DriveFR.getCurrentPosition() - inchesForward * ticksPerIn);
            DriveFL.setTargetPosition(DriveFL.getCurrentPosition() - inchesForward * ticksPerIn);
            DriveBR.setTargetPosition(DriveBR.getCurrentPosition() - inchesForward * ticksPerIn);
            DriveBL.setTargetPosition(DriveBL.getCurrentPosition() - inchesForward * ticksPerIn);

            DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void setDriveRight(int inchesRight)
    {
        if(opModeIsActive())
        {
            DriveFR.setTargetPosition(DriveFR.getCurrentPosition() + inchesRight * ticksPerIn);
            DriveFL.setTargetPosition(DriveFL.getCurrentPosition() - inchesRight * ticksPerIn);
            DriveBR.setTargetPosition(DriveBR.getCurrentPosition() - inchesRight * ticksPerIn);
            DriveBL.setTargetPosition(DriveBL.getCurrentPosition() + inchesRight * ticksPerIn);

            DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void setDriveLeft(int inchesLeft)
    {
        if(opModeIsActive())
        {
            DriveFR.setTargetPosition(DriveFR.getCurrentPosition() - inchesLeft * ticksPerIn);
            DriveFL.setTargetPosition(DriveFL.getCurrentPosition() + inchesLeft * ticksPerIn);
            DriveBR.setTargetPosition(DriveBR.getCurrentPosition() + inchesLeft * ticksPerIn);
            DriveBL.setTargetPosition(DriveBL.getCurrentPosition() - inchesLeft * ticksPerIn);

            DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void setDriveBackward(int inchesBack)
    {
        if(opModeIsActive())
        {
            DriveFR.setTargetPosition(DriveFR.getCurrentPosition() + inchesBack * ticksPerIn);
            DriveFL.setTargetPosition(DriveFL.getCurrentPosition() + inchesBack * ticksPerIn);
            DriveBR.setTargetPosition(DriveBR.getCurrentPosition() + inchesBack * ticksPerIn);
            DriveBL.setTargetPosition(DriveBL.getCurrentPosition() + inchesBack * ticksPerIn);

            DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void driveToSidePos()
    {
        if(opModeIsActive())
        {
            DriveFL.setPower(0.5);
            DriveFR.setPower(0.5);
            DriveBL.setPower(0.5);
            DriveBR.setPower(0.5);

            while (opModeIsActive() && (DriveFR.isBusy() || DriveBL.isBusy()))
            {
                robotAngle = gyroscopePart.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                if(robotAngle.firstAngle > 3)
                {
                    DriveFL.setTargetPosition(DriveFL.getTargetPosition() - 25);
                    DriveFR.setTargetPosition(DriveFR.getTargetPosition() + 25);
                    DriveBL.setTargetPosition(DriveBL.getTargetPosition() - 25);
                    DriveBR.setTargetPosition(DriveBR.getTargetPosition() + 25);
                    telemetry.addLine("-Correcting Right");
                }
                else if(robotAngle.firstAngle < -3)
                {
                    DriveFL.setTargetPosition(DriveFL.getTargetPosition() + 25);
                    DriveFR.setTargetPosition(DriveFR.getTargetPosition() - 25);
                    DriveBL.setTargetPosition(DriveBL.getTargetPosition() + 25);
                    DriveBR.setTargetPosition(DriveBR.getTargetPosition() - 25);
                    telemetry.addLine("-Correcting Left");
                }
                else
                {
                    DriveFL.setPower(0.5);
                    DriveFR.setPower(0.5);
                    DriveBL.setPower(0.5);
                    DriveBR.setPower(0.5);
                    telemetry.addLine("-On Course");
                }

                telemetry.addLine("Busy");
                telemetry.addData("FL Target:", DriveFL.getTargetPosition());
                telemetry.addData("BR Target:", DriveBR.getTargetPosition());
                telemetry.addData("FR Target:", DriveFR.getTargetPosition());
                telemetry.addData("BL Target:", DriveBL.getTargetPosition());
                telemetry.addData("FL Pos:", DriveFL.getTargetPosition());
                telemetry.addData("BR Pos: ", DriveBR.getTargetPosition());
                telemetry.addData("FR Pos: ", DriveFR.getTargetPosition());
                telemetry.addData("BL Pos: ", DriveBL.getTargetPosition());
                telemetry.addData("Heading ", robotAngle.firstAngle);
                telemetry.update();
            }

            DriveFL.setPower(0);
            DriveFR.setPower(0);
            DriveBL.setPower(0);
            DriveBR.setPower(0);

            telemetry.addLine("Run Completed");
            telemetry.addData("Heading ", robotAngle.firstAngle);
            telemetry.update();

            sleep(25);
        }
    }

    private void driveToLinearPos()
    {
        if(opModeIsActive())
        {
            DriveFL.setPower(0.5);
            DriveFR.setPower(0.5);
            DriveBL.setPower(0.5);
            DriveBR.setPower(0.5);

            while (opModeIsActive() && (DriveFR.isBusy() || DriveBL.isBusy()))
            {
                robotAngle = gyroscopePart.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                if(robotAngle.firstAngle > 3)
                {
                    DriveFL.setPower(0.55);
                    DriveFR.setPower(0.45);
                    DriveBL.setPower(0.55);
                    DriveBR.setPower(0.45);
                    telemetry.addLine("-Correcting Right");
                }
                else if(robotAngle.firstAngle < -3)
                {
                    DriveFL.setPower(0.45);
                    DriveFR.setPower(0.55);
                    DriveBL.setPower(0.45);
                    DriveBR.setPower(0.55);
                    telemetry.addLine("-Correcting Left");
                }
                else
                {
                    DriveFL.setPower(0.5);
                    DriveFR.setPower(0.5);
                    DriveBL.setPower(0.5);
                    DriveBR.setPower(0.5);
                    telemetry.addLine("-On Course");
                }

                telemetry.addLine("Busy");
                telemetry.addData("FL ", DriveFL.getTargetPosition());
                telemetry.addData("FR ", DriveFR.getTargetPosition());
                telemetry.addData("BL ", DriveBL.getTargetPosition());
                telemetry.addData("BR ", DriveBR.getTargetPosition());
                telemetry.addData("Heading ", robotAngle.firstAngle);
                telemetry.update();
            }

            DriveFL.setPower(0);
            DriveFR.setPower(0);
            DriveBL.setPower(0);
            DriveBR.setPower(0);

            telemetry.addLine("Run Completed");
            telemetry.addData("Heading ", robotAngle.firstAngle);
            telemetry.update();

            sleep(25);
        }
    }

    private void driveToBlindPos()
    {
        if(opModeIsActive())
        {
            DriveFL.setPower(0.5);
            DriveFR.setPower(0.5);
            DriveBL.setPower(0.5);
            DriveBR.setPower(0.5);

            while (opModeIsActive() && (DriveFR.isBusy() || DriveBL.isBusy()))
            {
                robotAngle = gyroscopePart.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                telemetry.addLine("Busy");
                telemetry.addData("FL ", DriveFL.getTargetPosition());
                telemetry.addData("FR ", DriveFR.getTargetPosition());
                telemetry.addData("BL ", DriveBL.getTargetPosition());
                telemetry.addData("BR ", DriveBR.getTargetPosition());
                telemetry.addData("Heading ", robotAngle.firstAngle);
                telemetry.update();
            }

            DriveFL.setPower(0);
            DriveFR.setPower(0);
            DriveBL.setPower(0);
            DriveBR.setPower(0);

            telemetry.addLine("Run Completed");
            telemetry.addData("Heading ", robotAngle.firstAngle);
            telemetry.update();

            sleep(25);
        }
    }

    public void driveToAngledPos()
    {
        if(opModeIsActive())
        {

            DriveFL.setPower(0.5);
            DriveFR.setPower(0.5);
            DriveBL.setPower(0.5);
            DriveBR.setPower(0.5);

            while (opModeIsActive() && (DriveFR.isBusy() || DriveBL.isBusy()))
            {
                robotAngle = gyroscopePart.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                if(robotAngle.firstAngle > 44)
                {
                    DriveFL.setPower(0.45);
                    DriveFR.setPower(0.55);
                    DriveBL.setPower(0.45);
                    DriveBR.setPower(0.55);
                }
                else if(robotAngle.firstAngle < -38)
                {
                    DriveFL.setPower(0.55);
                    DriveFR.setPower(0.45);
                    DriveBL.setPower(0.55);
                    DriveBR.setPower(0.45);
                }
                else
                {
                    DriveFL.setPower(0.5);
                    DriveFR.setPower(0.5);
                    DriveBL.setPower(0.5);
                    DriveBR.setPower(0.5);
                }

                telemetry.addLine("Busy");
                telemetry.addData("FL ", DriveFL.getTargetPosition());
                telemetry.addData("FR ", DriveFR.getTargetPosition());
                telemetry.addData("BL ", DriveBL.getTargetPosition());
                telemetry.addData("BR ", DriveBR.getTargetPosition());
                telemetry.addData("Heading ", robotAngle.firstAngle);
                telemetry.update();
            }

            DriveFL.setPower(0);
            DriveFR.setPower(0);
            DriveBL.setPower(0);
            DriveBR.setPower(0);

            telemetry.addLine("Run Completed");
            telemetry.addData("Heading ", robotAngle.firstAngle);
            telemetry.update();

            sleep(25);
        }
    }

    private void gyroCheck (boolean angled)
    {
        if (opModeIsActive())
        {
            if (angled)
            {
                while (robotAngle.firstAngle < -41)
                {
                    robotAngle = gyroscopePart.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    telemetry.addData("Heading (turning) ", robotAngle.firstAngle);
                    telemetry.update();

                    DriveFL.setTargetPosition(DriveFL.getCurrentPosition() + 50);
                    DriveFR.setTargetPosition(DriveFR.getCurrentPosition() - 50);
                    DriveBL.setTargetPosition(DriveBL.getCurrentPosition() + 50);
                    DriveBR.setTargetPosition(DriveBR.getCurrentPosition() - 50);

                    DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    driveToBlindPos();
                }
                while (robotAngle.firstAngle > -31)
                {
                    robotAngle = gyroscopePart.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    telemetry.addData("Heading (turning) ", robotAngle.firstAngle);
                    telemetry.update();

                    DriveFL.setTargetPosition(DriveFL.getCurrentPosition() - 50);
                    DriveFR.setTargetPosition(DriveFR.getCurrentPosition() + 50);
                    DriveBL.setTargetPosition(DriveBL.getCurrentPosition() - 50);
                    DriveBR.setTargetPosition(DriveBR.getCurrentPosition() + 50);

                    DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    driveToBlindPos();
                }
            }
            else
            {
                while (robotAngle.firstAngle < -3)
                {
                    robotAngle = gyroscopePart.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    DriveFL.setTargetPosition(DriveFL.getCurrentPosition() + 50);
                    DriveFR.setTargetPosition(DriveFR.getCurrentPosition() - 50);
                    DriveBL.setTargetPosition(DriveBL.getCurrentPosition() + 50);
                    DriveBR.setTargetPosition(DriveBR.getCurrentPosition() - 50);

                    driveToBlindPos();
                }
                while (robotAngle.firstAngle > 3)
                {
                    robotAngle = gyroscopePart.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    DriveFL.setTargetPosition(DriveFL.getCurrentPosition() - 50);
                    DriveFR.setTargetPosition(DriveFR.getCurrentPosition() + 50);
                    DriveBL.setTargetPosition(DriveBL.getCurrentPosition() - 50);
                    DriveBR.setTargetPosition(DriveBR.getCurrentPosition() + 50);

                    driveToBlindPos();
                }
            }
            telemetry.addLine("-Gyro Check Cleared.");
            telemetry.update();
        }
    }

    private void setAngleTurn ()
    {
        DriveFR.setTargetPosition(DriveFR.getCurrentPosition() + 480);
        DriveFL.setTargetPosition(DriveFL.getCurrentPosition() - 480);
        DriveBR.setTargetPosition(DriveBR.getCurrentPosition() + 480);
        DriveBL.setTargetPosition(DriveBL.getCurrentPosition() - 480);

        DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //To Do: refine turn (if this doesn't work, work on some dynamic method)
    }

    private void setAngleTurnBack ()
    {
        DriveFR.setTargetPosition(DriveFR.getCurrentPosition() - 480);
        DriveFL.setTargetPosition(DriveFL.getCurrentPosition() + 480);
        DriveBR.setTargetPosition(DriveBR.getCurrentPosition() - 480);
        DriveBL.setTargetPosition(DriveBL.getCurrentPosition() + 480);

        DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
