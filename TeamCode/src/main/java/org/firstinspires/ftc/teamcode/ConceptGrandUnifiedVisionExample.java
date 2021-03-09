package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Demonstrates OpMode using the VisionManager
 */
@TeleOp(name = "Grand Unified Vision Example", group = "Concept")
@Disabled
public class ConceptGrandUnifiedVisionExample extends LinearOpMode {
    private static final String TAG = "Grand Unified Vision Example";

    /**
     * State regarding our interaction with the camera
     */
    private final VisionManager visionManager = new VisionManager();

    private final VisionWebcamScanner webcamScanner = new VisionWebcamScanner();
    private String state = "Start";

    @Override
    public void runOpMode() {
        boolean successfullyInitialized = visionManager.initialize(hardwareMap);

        if (!successfullyInitialized) {
            telemetry.addData("FATAL", "Initialization failure!");
            telemetry.update();
            return;
        }

        webcamScanner.initialize(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        state = "Do Nothing";

        waitForStart();

        webcamScanner.goToNeutral();
//        webcamScanner.startScanning();

        visionManager.activate();

        try {
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    webcamScanner.goToStartingPosition();
                    webcamScanner.startScanning();
                    state = "Scanning";
                }

                if (gamepad1.b) {
                    webcamScanner.goToNeutral();
                    state = "Do Nothing";
                }

                //  If we're in the "scanning mode":
                if (state == "Scanning") {
                    webcamScanner.loop(50);

                    if (webcamScanner.isDoneScanning()) {
                        state = "Make Decision";
                    }
                }

                if (state == "Make Decision") {
                    // TBD

                }

                visionManager.loop();

                telemetry.addData("Last saw Quad rings    ", String.format("%3.2f seconds ago", visionManager.getHowManySecondsAgoSawQuadRings()));
                telemetry.addData("Last saw Single ring   ", String.format("%3.2f seconds ago", visionManager.getHowManySecondsAgoSawSingleRing()));

                telemetry.addData("Blue Target Visible    ", visionManager.isBlueTargetVisible());
                telemetry.addData("Last saw Blue Target   ", String.format("%3.2f seconds ago", visionManager.getHowManySecondsAgoSawBlueTarget()));

                telemetry.addData("Blue Alliance Visible  ", visionManager.isBlueAllianceVisible());
                telemetry.addData("Last saw Blue Alliance ", String.format("%3.2f seconds ago", visionManager.getHowManySecondsAgoSawBlueAlliance()));
                telemetry.addData("% Orange Pixels        ", String.format("%2.3f %%", visionManager.getPercentageOfPixelsThatAreOrange() * 100.0));

                double[] lastComputedLocation = visionManager.getLastComputedLocationFiltered();
                if (lastComputedLocation == null) {
                    telemetry.addData("Position                        ", "Unknown");
                } else {
                    telemetry.addData("Position X                      ", String.format("%.1f", lastComputedLocation[0]));
                    telemetry.addData("Position Y                      ", String.format("%.1f", lastComputedLocation[1]));
                    telemetry.addData("Position Z                      ", String.format("%.1f", lastComputedLocation[2]));
                }

                telemetry.update();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG, e, "OpMode Loop Error");
        } finally {
            visionManager.shutdown();
        }
    }
}
