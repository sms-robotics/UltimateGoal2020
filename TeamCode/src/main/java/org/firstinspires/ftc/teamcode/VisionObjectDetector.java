package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.UtilBotSettings;

import java.util.HashMap;
import java.util.List;

public class VisionObjectDetector {
    private static final String TAG = "VisionObjectDetector";
    public static final long NEVER_SAW = -1;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_QUAD = "Quad";
    public static final String LABEL_SINGLE = "Single";

    private final HashMap<String, Long> lastTimeSeenObject = new HashMap<>();

    /**
     * This is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public void initialize(HardwareMap hardwareMap, VuforiaLocalizer vuforia) {
        if (tfod != null) {
            RobotLog.w(TAG, "NOTE: VisionObjectDetector.initialie() was called twice. Ignoring, but it may mean you have a bug somewhere.");
            return; // be idempotent
        }

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        tfodParameters.minResultConfidence = (float) UtilBotSettings.sharedInstance().getVisionObjectDetectionMinResultConfidence();

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD, LABEL_SINGLE);

        tfod.setZoom(
            UtilBotSettings.sharedInstance().getTensorFlowCameraMagnification(),
            UtilBotSettings.sharedInstance().getTensorFlowCameraAspectRatio());
    }

    public void activate() {
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod == null) {
            throw new RuntimeException("VisionObjectDetector was not initialized!");
        }

        tfod.activate();
    }

    public void shutdown() {
        if (tfod != null) {
            tfod.deactivate();
            tfod.shutdown();
        }
    }

    public void loop() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    lastTimeSeenObject.put(recognition.getLabel(), System.currentTimeMillis());
                }
            }
        }
    }

    public long getLastTimeObjectSeen(String objectLabel) {
        return lastTimeSeenObject.getOrDefault(objectLabel, NEVER_SAW);
    }
}
