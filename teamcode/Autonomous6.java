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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Autonomous 6 (w/ 20pt Custom TensorFlow)")
public class Autonomous6 extends LinearOpMode {

    private DcMotor DriveFL;
    private DcMotor DriveFR;
    private DcMotor DriveBL;
    private DcMotor DriveBR;
    private Servo Claw;
    private DcMotor DriveLS;

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "model_20221105_101110.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Fish",
            "2 Octopus",
            "3 Crab"
    };
    //OLD CAMERA: 6.5 in above ground, end of field 35 in from very front of the cone, camera roughly 2 ft from front of cone
    //UPDATED CAMERA: 16cm above ground, ≈27cm from hypothetical wall

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
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

        DriveBL.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveFR.setDirection(DcMotorSimple.Direction.REVERSE);

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
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        Claw.setPosition(0.79);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (DriveLS.getCurrentPosition() >= -2100)
                {
                    Claw.setPosition(0.5);
                    sleep(20);
                    DriveLS.setPower(-0.25);
                    sleep(40);
                    DriveLS.setPower(0);
                }

                if (tfod != null) {
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

                            if (recognition.getLabel().equals("2 Octopus") && recognition.getConfidence() > 0.6)
                            {
                                /*while (opModeIsActive())
                                {*/
                                //positive value is forward, negative value is backwards
                                DriveBL.setPower(0.5);
                                DriveFL.setPower(0.5);
                                DriveBR.setPower(0.5);
                                DriveFR.setPower(0.5);
                                sleep(2000);
                                DriveBL.setPower(0);
                                DriveFL.setPower(0);
                                DriveBR.setPower(0);
                                DriveFR.setPower(0);
                                sleep(30000);
                                //}
                            }
                           else if (recognition.getLabel().equals("3 Crab") && recognition.getConfidence() > 0.6)
                            {
                                DriveBL.setPower(0.5);
                                DriveFL.setPower(0.5);
                                DriveBR.setPower(0.5);
                                DriveFR.setPower(0.5);
                                sleep(2000);
                                DriveBL.setPower(0);
                                DriveFL.setPower(0);
                                DriveBR.setPower(0);
                                DriveFR.setPower(0);
                                sleep(50);
                                DriveBL.setPower(-0.5);
                                DriveFL.setPower(0.5);
                                DriveBR.setPower(0.5);
                                DriveFR.setPower(-0.5);
                                sleep(2100);
                                DriveBL.setPower(0);
                                DriveFL.setPower(0);
                                DriveBR.setPower(0);
                                DriveFR.setPower(0);
                                sleep(30000);
                                //During tournament, robot did weird, unexplained zigzag (see video)
                            }
                            else if (recognition.getLabel().equals("1 Fish") && recognition.getConfidence() > 0.6)
                            {
                                DriveBL.setPower(0.5);
                                DriveFL.setPower(0.5);
                                DriveBR.setPower(0.5);
                                DriveFR.setPower(0.5);
                                sleep(2000);
                                DriveBL.setPower(0);
                                DriveFL.setPower(0);
                                DriveBR.setPower(0);
                                DriveFR.setPower(0);
                                sleep(50);
                                DriveBL.setPower(0.5);
                                DriveFL.setPower(-0.5);
                                DriveBR.setPower(-0.5);
                                DriveFR.setPower(0.5);
                                sleep(2100);
                                DriveBL.setPower(0);
                                DriveFL.setPower(0);
                                DriveBR.setPower(0);
                                DriveFR.setPower(0);
                                sleep(30000);
                            }

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }
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
}
