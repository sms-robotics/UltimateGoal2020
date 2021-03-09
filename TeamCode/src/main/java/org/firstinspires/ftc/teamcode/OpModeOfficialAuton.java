package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.MovementInstruction.InstructionType.DRIVE_DISTANCE;

@Autonomous(name="SMS Auton", group="Production")
public class OpModeOfficialAuton extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareUltimate robot = new HardwareUltimate();   // Use a Pushbot's hardware
    ElapsedTime runtime = new ElapsedTime();
    MovementBehaviors movementBehaviors;
    final ArrayList<MovementInstruction> movementMovementInstructions = new ArrayList<>(16);
    /**
     * State regarding our interaction with the camera
     */
    private VisionManager visionManager;

    private VisionWebcamScanner webcamScanner;
    private MovementThread movementThread;

    // Tracks what state we're in
    private enum AutonState {
        START,
        RING_SCAN,
        PICK_UP_WOBBLE,
        DRIVE_TO_FIND_IMAGE,
        DRIVE_TO_A,
        DRIVE_TO_B,
        DRIVE_TO_C,
        STOP,
    };
    private AutonState state = AutonState.START;

    @Override
    public void runOpMode() {
        long timesRun = 0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, true);
        movementBehaviors = robot.createAndInitializeMovementBehaviors(this);
        visionManager = robot.createAndInitializeVisionManager();

        webcamScanner = robot.createAndInitializeWebcamScanner();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        movementThread = new MovementThread(this, movementBehaviors);
        Thread t = new Thread(movementThread);
        t.start();

        // Assuming we're not going to mix and match modes for now.
        movementBehaviors.setRunToPositionMode();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        webcamScanner.goToNeutral();
        visionManager.activate();

        AutonState nextState = state;

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Our State     ", state.name());
            telemetry.addData("Time          ", String.format("%3.2f seconds", runtime.seconds()));
            telemetry.addData("#Instructions ", movementThread.getHowManyInstructions());
            telemetry.addData("Last saw Quad rings    ", String.format("%3.2f seconds ago", visionManager.getHowManySecondsAgoSawQuadRings()));
            telemetry.addData("Last saw Single ring   ", String.format("%3.2f seconds ago", visionManager.getHowManySecondsAgoSawSingleRing()));

            visionManager.loop();

            if (nextState != null && nextState != state) {
                // This means that we should change state
                state = nextState;
                timesRun = 0;
                runtime.reset();
            }

            if (state == AutonState.START) {
                nextState = doStart(timesRun);
            } else if (state == AutonState.RING_SCAN) {
                nextState = doRingScan(timesRun);
            } else if (state == AutonState.DRIVE_TO_A) {
                nextState = doDriveToA(timesRun);
            } else if (state == AutonState.DRIVE_TO_B) {
                nextState = doDriveToB(timesRun);
            } else if (state == AutonState.DRIVE_TO_C) {
                nextState = doDriveToC(timesRun);
            } else {
                nextState = doStop(timesRun);
            }

            timesRun++;

            telemetry.update();
            idle();
        }
    }

    private AutonState doStart(long timesRun) {

        visionManager.activate();

        // All we do here is point the camera
        webcamScanner.goToNeutral();

        return AutonState.RING_SCAN;
    }

    private AutonState doRingScan(long timesRun) {
        if (timesRun == 0) {

            webcamScanner.goToStartingPosition();
            webcamScanner.startScanning();
            runtime.reset();
            return null;
        }

        webcamScanner.loop(50);

        if (webcamScanner.isDoneScanning() && runtime.seconds() > 3) {
            double howManySecondsAgoSawSingleRing = visionManager.getHowManySecondsAgoSawSingleRing();
            double howManySecondsAgoSawQuadRings = visionManager.getHowManySecondsAgoSawQuadRings();

            if (howManySecondsAgoSawSingleRing != VisionManager.NEVER_SAW
                && howManySecondsAgoSawSingleRing < 3) {
                return AutonState.DRIVE_TO_A;
            } else if (howManySecondsAgoSawQuadRings != VisionManager.NEVER_SAW
                && howManySecondsAgoSawQuadRings < 3) {
                return AutonState.DRIVE_TO_C;
            } else {
                return AutonState.DRIVE_TO_C;
            }
        }

        return null;
    }

    private AutonState doDriveToA(long timesRun) {
        if (timesRun == 0) {
            addDriveInstruction(DRIVE_DISTANCE, 500, 0);
            addDriveInstruction(DRIVE_DISTANCE, 500, -90);
            addDriveInstruction(DRIVE_DISTANCE, 500, -180);
            return null;
        }

        // Now we wait
        if (movementThread.hasNothingToDo()) {
            return AutonState.STOP;
        }

        return null;
    }

    private AutonState doDriveToB(long timesRun) {
        return AutonState.STOP;
    }

    private AutonState doDriveToC(long timesRun) {
        return AutonState.STOP;
    }

    private AutonState doStop(long timesRun) {
        if (gamepad1.x) {
            return AutonState.START;
        }

        return null;
    }

    private void addDriveInstruction(MovementInstruction.InstructionType instructionType, double... parameters) {
        MovementInstruction i = new MovementInstruction(instructionType, parameters);
        movementThread.addInstruction(i);
    }
}
