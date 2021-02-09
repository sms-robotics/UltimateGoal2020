package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

/**
 * This is our settings class for storing calibrations and other items
 * like the Vuforia license key that we don't put in code.
 * <p>
 * It's a "singleton" meaning that there's only one of them. You can
 * get at it using BotSettings.sharedInstance() and then following that
 * with the function you want.
 * <p>
 * Usage Example::
 * <p>
 * BotSettings.sharedInstance().getVuforiaKey()
 */
public class BotSettings {
    private static final String TAG = "BotSettings";
    private final File settingsDirectory = AppUtil.ROBOT_SETTINGS;
    private final String settingsFileName = "bot.json";

    //---------------------------------------------------------------------------------------------
    // This is where we add new settings.
    //
    // Choose a name that makes sense. If you have a bunch of variables that all are used for
    // the same thing (e.g. tensorFlow), then try starting each variable with the same prefix.
    //
    // Choose a default value for each item here, too. If we don't see it in the settings file,
    // then this is what we'll use.
    //---------------------------------------------------------------------------------------------
    private String vuforiaKey = "";

    // The TensorFlow software will scale the input images from the camera to a lower resolution.
    // This can result in lower detection accuracy at longer distances (> 55cm or 22").
    // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
    // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
    // should be set to the value of the images used to create the TensorFlow Object Detection model
    // (typically 1.78 or 16/9).
    private double tensorFlowCameraMagnification = 3;
    private double tensorFlowCameraAspectRatio = 16/9; // Default to 4:3

    private String webcamName = "Webcam 1";

    private double visionObjectDetectionMinResultConfidence = 0.8;

    //---------------------------------------------------------------------------------------------
    // This is where you read from the settings file and change the values of your variables
    //
    // The template is:
    //   [variable name] = [extractString or extractDouble or extractInt](
    //      jsonObject,
    //      [the name of the field in your file],
    //      this.[your variable name]);
    //
    // Example: this.vuforiaKey = extractString(jsonObject, "vuforiaKey", this.vuforiaKey);
    //
    // This says: "Extract the value 'vuforiaKey' from the file, try to make sense of it as a String,
    // but if you don't see anything or can't make sense of it as a string, use whatever we already
    // have in the vuforiaKey variable."
    //---------------------------------------------------------------------------------------------
    private void loadImpl(JSONObject jsonObject) {
        this.vuforiaKey = extractString(jsonObject, "vuforiaKey", this.vuforiaKey);
        this.tensorFlowCameraMagnification = extractDouble(jsonObject, "tensorFlowCameraMagnification", this.tensorFlowCameraMagnification);
        this.tensorFlowCameraAspectRatio = extractDouble(jsonObject, "tensorFlowCameraAspectRatio", this.tensorFlowCameraAspectRatio);
        this.webcamName = extractString(jsonObject, "webcamName", this.webcamName);
        this.visionObjectDetectionMinResultConfidence = extractDouble(jsonObject, "visionObjectDetectionMinResultConfidence", this.visionObjectDetectionMinResultConfidence);
    }

    //---------------------------------------------------------------------------------------------
    // Getters
    //
    // This is where you add functions to return the values (this is the correct way to do things
    // so that you can only read the values and not write them).
    //---------------------------------------------------------------------------------------------
    public String getVuforiaKey() {
        return vuforiaKey;
    }

    public double getTensorFlowCameraMagnification() {
        return tensorFlowCameraMagnification;
    }

    public double getTensorFlowCameraAspectRatio() {
        return tensorFlowCameraAspectRatio;
    }

    public String getWebcamName() {
        return webcamName;
    }

    public double getVisionObjectDetectionMinResultConfidence() {
        return visionObjectDetectionMinResultConfidence;
    }

    //---------------------------------------------------------------------------------------------
    // Inner workings! You probably don't need to edit any of this.
    //---------------------------------------------------------------------------------------------
    private static class BotSettingsSingletonContainer {
        static final BotSettings SHARED_INSTANCE = BotSettings.factory();
    }

    static public BotSettings sharedInstance() {
        return BotSettingsSingletonContainer.SHARED_INSTANCE;
    }

    private static BotSettings factory() {
        BotSettings botSettings = new BotSettings();

        botSettings.load();

        return botSettings;
    }

    private boolean load() {
        File settingsFile = new File(settingsDirectory, settingsFileName);
        RobotLog.d("Attempting to load settings from %s", settingsFile);

        if (!(settingsFile.exists() && settingsFile.canRead())) {
            RobotLog.e("Either the settings file %s does not exist or could not be read.", settingsFile);
            return false;
        }

        RobotLog.d("Loading settings from %s", settingsFile);
        FileReader fileReader = null;
        try {
            fileReader = new FileReader(settingsFile);
            BufferedReader bufferedReader = new BufferedReader(fileReader);
            StringBuilder stringBuilder = new StringBuilder();
            String line = bufferedReader.readLine();
            while (line != null) {
                stringBuilder.append(line).append("\n");
                line = bufferedReader.readLine();
            }
            bufferedReader.close();

            JSONObject jsonObject = new JSONObject(stringBuilder.toString());

            loadImpl(jsonObject);

            return true;
        } catch (FileNotFoundException e) {
            RobotLog.ee(TAG, e, "Our settings file wasn't found, which is odd because we should have explicitly checked for it.");
            e.printStackTrace();
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "An error occurred when reading our settings file. Perhaps it is corrupt?");
            e.printStackTrace();
        } catch (JSONException e) {
            RobotLog.ee(TAG, e, "An error occurred when reading our settings file because it is not valid .json. Check the file contents!");
            e.printStackTrace();
        } finally {
            if (fileReader != null) {
                try {
                    fileReader.close();
                } catch (IOException e) {
                    RobotLog.ee(TAG, e, "An error occurred when trying to close our settings .json.");
                    e.printStackTrace();
                }
            }
        }

        return false;
    }

    private String extractString(JSONObject jsonObject, String fieldName, String defaultValue) {
        return jsonObject.optString(fieldName, defaultValue);
    }

    private int extractInt(JSONObject jsonObject, String fieldName, int defaultValue) {
        return jsonObject.optInt(fieldName, defaultValue);
    }

    private double extractDouble(JSONObject jsonObject, String fieldName, double defaultValue) {
        return jsonObject.optDouble(fieldName, defaultValue);
    }

    private boolean extractBoolean(JSONObject jsonObject, String fieldName, boolean defaultValue) {
        return jsonObject.optBoolean(fieldName, defaultValue);
    }
}
