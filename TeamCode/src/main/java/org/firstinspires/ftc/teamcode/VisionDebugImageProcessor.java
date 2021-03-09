package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

/**
 * The Debug Image Processor's job is to take Bitmaps as they
 * become available from Vuforia and save them out to the SD card
 * at a predictable interval. If you write them too quickly,
 * you can't effectively read them.
 * <p>
 * This class is useful when you don't have any other means to
 * trigger the saving of an image.
 */
public class VisionDebugImageProcessor {
    private final static String TAG = "DebugImageProcessor";
    private final File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    private long timeSinceLastFrame;
    private long timeBetweenSavedFramesInMilliseconds = 1000;

    /**
     * Call this for when the opmode starts *before* you call
     * any other method.
     */
    public void initialize() {
        timeSinceLastFrame = 0;
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
    }

    /**
     * Call this for cleanup when the opmode is finished.
     */
    public void shutdown() {
    }

    /**
     * This is called periodically when a bitmap is made available.
     * If the amount of time has exceeded the value of
     * timeBetweenSavedFramesInMilliseconds, then the image is saved
     * into the storage/primary/FIRST/data/webcam-frame.jpg
     * (or something like that).
     *
     * @param bitmap
     */
    public void onNewBitmapAvailable(Bitmap bitmap) {
        if (System.currentTimeMillis() - timeSinceLastFrame >= timeBetweenSavedFramesInMilliseconds) {
            timeSinceLastFrame = System.currentTimeMillis();
            final String fileName = "webcam-frame.jpg";

            RobotLog.d(TAG, "Saving Bitmap %s", fileName);
            saveBitmap(bitmap, fileName);
        }
    }

    private void saveBitmap(Bitmap bitmap, String fileName) {
        File file = new File(captureDirectory, fileName);
        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
            }
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "exception in saveBitmap()");
        }
    }

    public long getTimeBetweenSavedFramesInMilliseconds() {
        return timeBetweenSavedFramesInMilliseconds;
    }

    public void setTimeBetweenSavedFramesInMilliseconds(long timeBetweenSavedFramesInMilliseconds) {
        this.timeBetweenSavedFramesInMilliseconds = timeBetweenSavedFramesInMilliseconds;
    }
}
