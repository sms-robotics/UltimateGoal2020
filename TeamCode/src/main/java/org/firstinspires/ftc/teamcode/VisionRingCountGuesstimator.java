package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

public class VisionRingCountGuesstimator {
    private static final int COLOR_THRESHOLD = 100;
    private static final long NEVER_SAW = -1;
    private long lastSawFrameAtTimestamp = NEVER_SAW;
    private long lastImageWidth = 640;
    private long lastImageHeight = 480;
    private long lastNumberOfOrangePixels = 0;

    public void initialize() {
        lastSawFrameAtTimestamp = NEVER_SAW;
        lastImageWidth = 640;
        lastImageHeight = 480;
        lastNumberOfOrangePixels = 0;
    }

    public void shutdown() {
    }

    public void onNewBitmapAvailable(Bitmap bitmap) {
        lastSawFrameAtTimestamp = System.currentTimeMillis();

        lastNumberOfOrangePixels = findRings(bitmap);
    }

    private Bitmap filterOrange(Bitmap picture) {
        Bitmap filtered = picture.copy(picture.getConfig(), true);

        class OrangeCounterPair {
            int left = 0;
            int up = 0;
        }

//        OrangeCounterPair pairs[][] = new OrangeCounterPair[filtered.getWidth()][filtered.getHeight()];

        for (int i = 0; i < picture.getWidth(); i++) {
            for (int j = 0; j < picture.getHeight(); j++) {
                int pixel = picture.getPixel(i, j);
                int blue = Color.blue(pixel);
                int red = Color.red(pixel);
                int green = Color.red(pixel);

                if (red > blue + COLOR_THRESHOLD && green > blue + COLOR_THRESHOLD) {
//                    pairs[i][j].left = pairs[i][j];
//                    filtered.getPixel(i, j);
                    filtered.setPixel(i, j, 0);
                }
            }
        }

        return filtered;
    }

    private int findRings(Bitmap picture) {
        int numFound = 0;

        int counter = 0;
        for (int i = 0; i < picture.getWidth(); i++) {
            for (int j = 0; j < picture.getHeight(); j++) {
                int pixel = picture.getPixel(i, j);
                int blue = Color.blue(pixel);
                int red = Color.red(pixel);
                int green = Color.red(pixel);

                if (red > blue + COLOR_THRESHOLD && green > blue + COLOR_THRESHOLD) {
                    counter++;
                    numFound++;
                }
            }
        }

        return numFound;
    }

    public double getPercentageOrangePixels() {
        long totalNumberPixels = lastImageWidth * lastImageHeight;
        if (totalNumberPixels > 0) {
            return (double) lastNumberOfOrangePixels / (double) totalNumberPixels;
        }

        return 0;
    }
}
