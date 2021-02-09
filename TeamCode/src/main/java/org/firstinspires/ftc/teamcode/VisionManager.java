package org.firstinspires.ftc.teamcode;


import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class VisionManager {
    private static final String TAG = "VisionManager";

    public static final long NEVER_SAW = VisionObjectDetector.NEVER_SAW;

    /**
     * State regarding our interaction with the camera
     */
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private boolean debugImageCaptureEnabled = true;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    private final DebugImageProcessor debugImageProcessor = new DebugImageProcessor();
    private final VisionObjectDetector objectDetector = new VisionObjectDetector();
    private final VisionNavigator navigator = new VisionNavigator();
    private final RingCountGuesstimator ringCountGuesstimator = new RingCountGuesstimator();

    public boolean initialize(HardwareMap hardwareMap) {
        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, BotSettings.sharedInstance().getWebcamName());

        debugImageProcessor.initialize();
        ringCountGuesstimator.initialize();

        try {
            openCamera();

            if (camera == null) {
                RobotLog.e(TAG, "Error opening camera via the openCamera() function!");
                return false;
            }

            initVuforia(hardwareMap);

            objectDetector.initialize(hardwareMap, vuforia);
            navigator.initialize(hardwareMap, vuforia);

            return true;
        } catch (Exception e) {
            RobotLog.ee(TAG, e, "Opmode Loop Error");
            this.shutdown();
        }

        return false;
    }

    public void activate() {
        objectDetector.activate();
        navigator.activate();
    }

    public void loop() {
        objectDetector.loop();
        navigator.loop();

        try {
            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().poll(10, TimeUnit.MILLISECONDS);
            if (frame != null) {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);

                onNewBitmapAvailable(bitmap);

                bitmap.recycle();
                frame.close();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG, e, "Error processing Vuforia frame");
        }
    }

    public void shutdown() {
        closeCamera();

        if (debugImageProcessor != null) {
            debugImageProcessor.shutdown();
        }

        if (objectDetector != null) {
            objectDetector.shutdown();
        }

        if (navigator != null) {
            navigator.shutdown();
        }

        if (ringCountGuesstimator != null) {
            ringCountGuesstimator.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = BotSettings.sharedInstance().getVuforiaKey();
        parameters.cameraName = hardwareMap.get(WebcamName.class, BotSettings.sharedInstance().getWebcamName());

        parameters.webcamCalibrationResources = new int[]{R.xml.teamwebcamcalibrations};
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // We need this in order to pull frames and use the vuforia.convertFrameToBitmap method
        vuforia.enableConvertFrameToBitmap();

        // This enables the frame queue so we can pull from it in our main loop
        vuforia.setFrameQueueCapacity(5);

        CameraCalibration cameraCalibration = vuforia.getCameraCalibration();

    }

    private void onNewBitmapAvailable(Bitmap bitmap) {
        if (debugImageCaptureEnabled) {
            debugImageProcessor.onNewBitmapAvailable(bitmap);
        }
//
//        ringCountGuesstimator.onNewBitmapAvailable(bitmap);
    }

    private void openCamera() {
        if (camera != null) {
            RobotLog.i("NOTE: openCamera was called when we already had a camera object. Ignoring, but it may mean you have a bug somewhere.");
            return; // be idempotent
        }

        Deadline deadline = new Deadline(Integer.MAX_VALUE, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null) {
            RobotLog.e(TAG, "Camera not found or permission to use not granted: %s", cameraName);
        }
    }

    private void closeCamera() {
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    public double getHowManySecondsAgoSawSingleRing() {
        long timestamp = objectDetector.getLastTimeObjectSeen(VisionObjectDetector.LABEL_SINGLE);
        if (timestamp == VisionObjectDetector.NEVER_SAW) {
            return NEVER_SAW;
        }

        return (System.currentTimeMillis() - timestamp) / 1000.0;
    }

    public double getHowManySecondsAgoSawQuadRings() {
        long timestamp = objectDetector.getLastTimeObjectSeen(VisionObjectDetector.LABEL_QUAD);
        if (timestamp == VisionObjectDetector.NEVER_SAW) {
            return NEVER_SAW;
        }

        return (System.currentTimeMillis() - timestamp) / 1000.0;
    }

    public boolean isBlueAllianceVisible() {
        return navigator.isCurrentlyVisible(VisionNavigator.LABEL_BLUE_ALLIANCE);
    }

    public boolean isBlueTargetVisible() {
        return navigator.isCurrentlyVisible(VisionNavigator.LABEL_BLUE_TARGET);
    }

    public double getHowManySecondsAgoSawBlueAlliance() {
        return getHowManySecondsAgoSawSomethingFromNavigator(VisionNavigator.LABEL_BLUE_ALLIANCE);
    }

    public double getHowManySecondsAgoSawBlueTarget() {
        return getHowManySecondsAgoSawSomethingFromNavigator(VisionNavigator.LABEL_BLUE_TARGET);
    }

    public double getHowManySecondsAgoSawSomethingFromNavigator(String objectLabel) {
        long timestamp = navigator.getLastTimeObjectSeen(objectLabel);
        if (timestamp == VisionNavigator.NEVER_SAW) {
            return NEVER_SAW;
        }

        return (System.currentTimeMillis() - timestamp) / 1000.0;
    }

    public double getPercentageOfPixelsThatAreOrange() {
        return ringCountGuesstimator.getPercentageOrangePixels();
    }

    public double[] getLastComputedLocation() {
        double[] location = navigator.getLastComputedLocationFiltered();

        return location;
    }

    public double[] getLastComputedLocationFiltered() {
        return navigator.getLastComputedLocationFiltered();
    }
}
