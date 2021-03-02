package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Scans from a known position and waits at each position long
 * enough to detect objects.
 * <p>
 * Usage:
 * <p>
 * webcamScanner = new WebcamScanner();
 * <p>
 * webcamScanner.initialize(hardwareMap);
 * <p>
 * webcamScanner.startScanning();
 * webcamScanner.goToStartingPosition();
 * <p>
 * .. inside opmode ...
 * <p>
 * if (we are scanning) {
 * webcamScanner.loop();
 * <p>
 * if (webcamScanner.isDoneScanning()) {
 * we are not scanning anymore
 * }
 * }
 */
public class WebcamScanner {
    static final double STARTING_POSITION = 0.60;
    static final double ENDING_POSITION = 0.9;
    static final double NEUTRAL_POSITION = 0.75;
    static final double SCAN_INCREMENT = 0.01;

    private Servo servo;
    private long timeSinceLastMove;
    private long timeOfLastLoop;

    public void initialize(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "webcam_servo");
        timeSinceLastMove = 0;
        timeOfLastLoop = 0;
    }

    public void startScanning() {
        timeSinceLastMove = 0;
        timeOfLastLoop = 0;
    }

    public void goToNeutral() {
        timeSinceLastMove = 0;
        timeOfLastLoop = 0;
        servo.setPosition(NEUTRAL_POSITION);
    }

    public void goToStartingPosition() {
        servo.setPosition(STARTING_POSITION);
    }

    public void loop(long timeBetweenScans) {
        if (timeOfLastLoop > 0) {
            timeSinceLastMove += System.currentTimeMillis() - timeOfLastLoop;
        }

        if (timeSinceLastMove >= timeBetweenScans) {
            turnRight();
            timeSinceLastMove = 0;
        }

        timeOfLastLoop = System.currentTimeMillis();
    }

    public void turnRight() {
        double newPosition = servo.getPosition() + SCAN_INCREMENT;
        servo.setPosition(newPosition);
    }

    public void turnLeft() {
        double newPosition = servo.getPosition() - SCAN_INCREMENT;
        servo.setPosition(newPosition);
    }

    public boolean isDoneScanning() {
        if (servo.getPosition() >= ENDING_POSITION) {
            return true;
        }

        return false;
    }
}
