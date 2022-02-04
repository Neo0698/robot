/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.auto.AutoOpMode;

import java.util.List;

public class Cam
{
    private ElapsedTime runtime;

    private Telemetry telemetry;

    private TouchSensor donutEntryDetector;
    private TouchSensor donutExitDetector;
    private Movement movement;
    private boolean lastDonutEntryDetector;
    private boolean lastDonutExitDetector;
    private int donuts;

    // TENSORFLOW STUFF
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static final float duckPositionLimitRight = 860f;
    private static final float duckPositionLimitLeft = 430f;
    private static final String VUFORIA_KEY =
            "AV6C5jj/////AAABmRYAyzGUZkbejyLfHAMMOYAgm6zoHkv+bfDX9sJL/plz8XEyXfNd07ViBIC8cNHvYHtapiTbjys2TJ9vXwPrM0iPwxKBH+2tObEAcA5B20NfCcj6SEB3ztH9dsYpffJ3Aoj8c8jcAydgAMD3OP70B4utix06Rb7Dcid9txXrsWktkJkVe8Xkmy1k1ySq+CMzgBCf1VrZtgW+pdEQCf3QYjEbeE9REWXKx8EScUlWQqmsquEXl1ZaCM2Aiqtgybg53VpmWqyVvFWNcuhd91IHkQbpVpKOiNQco1gDaLXA8YbuYZq+kVLeNshaQat28qF83Vu8XKoTr2QkrTpUzdbWXxh1nwSEEahwytUlilxUptfR";
    private VuforiaLocalizer vuforia;
    private static TFObjectDetector tfod;

    public Cam(Telemetry globalTelemetry, ElapsedTime globalRuntime, WebcamName camera, int viewId) {
        // INITIALIZE TELEMETRY
        telemetry = globalTelemetry;
        runtime = globalRuntime;

        // Assign servos
        initVuforia(camera);
        initTfod(viewId);
    }



    private void initVuforia(WebcamName camera) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = camera;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod(int viewId) {
        int tfodMonitorViewId = viewId;
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    public void activate() {
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            //tfod.setZoom(1);
            tfod.setZoom(2.5,16.0/9.0);
        }
    }
    public void deactivate() {
        tfod.deactivate();
    }

    public int get() {
        return donuts;
    }

    /*public int update() {
        if (donutEntryDetector.isPressed() && !lastDonutEntryDetector) {
            donuts += 1;
        }
        lastDonutEntryDetector = donutEntryDetector.isPressed();

        if (donutExitDetector.isPressed() && !lastDonutExitDetector) {
            donuts -= 1;
        }
        return donuts;
    }*/

    public enum CameraObject {
        NONE,
        DUCK,
        CUBE,
        BALL,
        UNSURE
    }
    static final public  float get_x(){
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        return updatedRecognitions.get(0).getLeft();
    }
    static final public  float get_y(){
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        return updatedRecognitions.get(0).getLeft();
    }
    public CameraObject checkCamera() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null && !updatedRecognitions.isEmpty()) {
            telemetry.addData("Object Detected", updatedRecognitions.get(0).getLabel());
            // step through the list of recognitions and display boundary info.

            if (updatedRecognitions.get(0).getConfidence() < 0.5) {
                return CameraObject.UNSURE;
            }
            if (updatedRecognitions.get(0).getLabel() == LABELS[0]) {
                return Cam.CameraObject.BALL;
            }
            if (updatedRecognitions.get(0).getLabel() == LABELS[1]) {
                return Cam.CameraObject.CUBE;
            }
            if (updatedRecognitions.get(0).getLabel() == LABELS[2]) {
                return Cam.CameraObject.DUCK;
            }
        }
        return CameraObject.NONE;
    }

    public enum Etage {
        ETAGE_1,
        ETAGE_2,
        ETAGE_3,
        ETAGE_UNSURE
    }



    public Etage getPosition(AutoOpMode.StartPosition startPosition){
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (
                updatedRecognitions != null
                        && !updatedRecognitions.isEmpty()
                        && updatedRecognitions.get(0).getLabel()==LABELS[2]
        ) {
            float averagePos = (updatedRecognitions.get(0).getLeft() + updatedRecognitions.get(0).getRight()) / 2;
            if (averagePos < duckPositionLimitLeft) {
                return startPosition == AutoOpMode.StartPosition.WAREHOUSE ? Etage.ETAGE_3 : Etage.ETAGE_1;
            } else if (averagePos > duckPositionLimitRight) {
                return startPosition == AutoOpMode.StartPosition.WAREHOUSE ? Etage.ETAGE_1 : Etage.ETAGE_3;
            } else {
                return Etage.ETAGE_2;
            }
        }
        return null;
    }

    //public Etage getValue(Etage val) { return val; }
}
