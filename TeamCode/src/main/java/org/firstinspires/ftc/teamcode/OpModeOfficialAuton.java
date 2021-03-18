package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.MovementInstruction.InstructionType.DRIVE_DISTANCE;
import static org.firstinspires.ftc.teamcode.MovementInstruction.InstructionType.FIRE_RING;
import static org.firstinspires.ftc.teamcode.MovementInstruction.InstructionType.MOVE_WOBBLE_ARM_TO_POSITION;
import static org.firstinspires.ftc.teamcode.MovementInstruction.InstructionType.WAIT_FOR_WOBBLE_ARM;
import static org.firstinspires.ftc.teamcode.MovementInstruction.InstructionType.TURN_OFF_CONVEYOR;
import static org.firstinspires.ftc.teamcode.MovementInstruction.InstructionType.TURN_OFF_SHOOTER;
import static org.firstinspires.ftc.teamcode.MovementInstruction.InstructionType.TURN_ON_CONVEYOR;
import static org.firstinspires.ftc.teamcode.MovementInstruction.InstructionType.TURN_ON_SHOOTER;
import static org.firstinspires.ftc.teamcode.MovementInstruction.InstructionType.TURN_TO;
import static org.firstinspires.ftc.teamcode.MovementInstruction.InstructionType.WAIT_FOR_TIME;

@Autonomous(name="SMS Auton", group="Production")
public class OpModeOfficialAuton extends LinearOpMode {
    private static final double MM_PER_FOOT = 304.8;

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
    private ActionWobbleArm wobbleArm;
    private ActionConveyor conveyor;
    private ActionShooter shooter;
    private ActionTrigger trigger;
    private SensorIMU sensorImu;

    // Tracks what state we're in
    private enum AutonState {
        // START: We are waiting to do something
        START,
        // We are having the webcam scan the field
        RING_SCAN,
        // We run the "A" program
        PROGRAM_A,
        // We run the "B" program
        PROGRAM_B,
        // We run the "C" program
        PROGRAM_C,
        // We are done (game controller can restart for debugging)
        STOP,
        ;
    };
    private AutonState state = AutonState.START;

    @Override
    public void runOpMode() {
        long timesRun = 0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, true);
        visionManager = robot.createAndInitializeVisionManager();
        webcamScanner = robot.createAndInitializeWebcamScanner();

        conveyor = robot.createAndInitializeConveyor();
        shooter = robot.createAndInitializeShooter();
        trigger = robot.createAndInitializeTrigger();
        wobbleArm = robot.createAndInitializeWobbleArm(this);
        sensorImu = robot.createAndInitializeIMU(this);

        movementBehaviors = robot.createAndInitializeMovementBehaviors(this, conveyor, shooter, trigger, wobbleArm, sensorImu);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "I am alive");    //
        telemetry.update();

        movementThread = new MovementThread(this, movementBehaviors);
        Thread t = new Thread(movementThread);
        t.start();

        // Assuming we're not going to mix and match modes for now.
        movementBehaviors.setRunToPositionMode();

        webcamScanner.goToNeutral();
        visionManager.activate();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sensorImu.resetAngle();

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
            } else if (state == AutonState.PROGRAM_A) {
                nextState = doDriveToA(timesRun);
            } else if (state == AutonState.PROGRAM_B) {
                nextState = doDriveToB(timesRun);
            } else if (state == AutonState.PROGRAM_C) {
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

    // RING_SCAN:
    //
    // We start the webcamScanner and let it do its
    // full sweep. When it says it's done, we decide
    // how many rings we see and choose our next
    // state accordingly.
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

            // If we see 1 ring, then B
            if (howManySecondsAgoSawSingleRing != VisionManager.NEVER_SAW
                && howManySecondsAgoSawSingleRing < 3) {
                return AutonState.PROGRAM_B;
            } else if (howManySecondsAgoSawQuadRings != VisionManager.NEVER_SAW
                && howManySecondsAgoSawQuadRings < 3) {
                // If we see 4 rings, then C
                return AutonState.PROGRAM_C;
            } else {
                // No rings is A
                return AutonState.PROGRAM_A;
            }
        }

        return null;
    }

    // DRIVE_TO_A:
    private AutonState doDriveToA(long timesRun) {
        if (timesRun == 0) {
            // Drive for 5 ft forward at heading 0 degrees at 50% speed
            addInstruction(DRIVE_DISTANCE, 5 * MM_PER_FOOT, 0, 0.5);
            // Turn 45 degrees to the left (hence the negative)
            addInstruction(TURN_TO, -45);
            // Move the wobble arm to zero position at 25% power
            addInstruction(MOVE_WOBBLE_ARM_TO_POSITION, 1.0, 0.25);
            // Wait at most 4 seconds for it to do its thing
            addInstruction(WAIT_FOR_WOBBLE_ARM, 4000);
            // Turn back to fire the rings into the top slot
            addInstruction(TURN_TO, 0);
            // Turn on shooter at 90% power
            addInstruction(TURN_ON_SHOOTER, 0.9);
            // Turn on conveyor at 75% power
            addInstruction(TURN_ON_CONVEYOR, 0.75);
            // Wait three seconds (3000 milliseconds)
            addInstruction(WAIT_FOR_TIME, 3000);
            // FIRE and wait 2 seconds for the trigger to sweep
            addInstruction(FIRE_RING, 2000);
            // Wait three seconds
            addInstruction(WAIT_FOR_TIME, 3000);
            // FIRE and wait 2 seconds for the trigger to sweep up and back
            addInstruction(FIRE_RING, 2000);
            // Wait three seconds
            addInstruction(WAIT_FOR_TIME, 3000);
            // FIRE and wait 2 seconds for the trigger to sweep up and back
            addInstruction(FIRE_RING, 2000);
            // Turn these off while we park
            addInstruction(TURN_OFF_SHOOTER);
            addInstruction(TURN_OFF_CONVEYOR);
            addInstruction(TURN_TO, 0);
            // Drive straight back to parked position
            // NOTE: we'll have to adjust this because we
            // didn't start on the line, but we'll worry about
            // that when we can fine-tune
            //
            // Drive back 5 feet at 50% speed at 0 degrees (wrong!.. fixme!)
            addInstruction(DRIVE_DISTANCE, -5 * MM_PER_FOOT, 0, 0.5);

            return null;
        }

        // Now we wait
        if (movementThread.hasNothingToDo()) {
            return AutonState.STOP;
        }

        return null;
    }

    private AutonState doDriveToB(long timesRun) {
        if (timesRun == 0) {
            // TODO: I think we can just figure out what the angle to drive at should be
            addInstruction(DRIVE_DISTANCE, 7 * MM_PER_FOOT, -5.0, 0.5);
            // Get the wobble arm in the square
            addInstruction(TURN_TO, -45);
            // Move the wobble arm to zero position at 25% power
            addInstruction(MOVE_WOBBLE_ARM_TO_POSITION, 1.0, 0.25);
            // Wait at most 4 seconds for it to do its thing
            addInstruction(WAIT_FOR_WOBBLE_ARM, 4000);
            // Turn back to fire the rings into the top slot
            addInstruction(TURN_TO, 0);
            // Drive back behind the launch zone -2ft
            // TODO: Would need to use the same angle
            addInstruction(DRIVE_DISTANCE, -2 * MM_PER_FOOT, -5.0, 0.5);
            // Turn on shooter at 90% power
            addInstruction(TURN_ON_SHOOTER, 0.9);
            // Turn on conveyor at 75% power
            addInstruction(TURN_ON_CONVEYOR, 0.75);
            // Wait three seconds
            addInstruction(WAIT_FOR_TIME, 3000);
            // FIRE and wait 2 seconds for the trigger to sweep
            addInstruction(FIRE_RING, 2000);
            // Wait three seconds
            addInstruction(WAIT_FOR_TIME, 3000);
            // FIRE and wait 2 seconds for the trigger to sweep
            addInstruction(FIRE_RING, 2000);
            // Wait three seconds
            addInstruction(WAIT_FOR_TIME, 3000);
            // FIRE and wait 2 seconds for the trigger to sweep
            addInstruction(FIRE_RING, 2000);
            // Turn these off while we park
            addInstruction(TURN_OFF_SHOOTER);
            addInstruction(TURN_OFF_CONVEYOR);
            addInstruction(TURN_TO, 0);
            // Drive straight back to parked position
            // NOTE: we'll have to adjust this because we
            // didn't start on a line, but we'll worry about
            // that when we can fine-tune
            addInstruction(DRIVE_DISTANCE, -5 * MM_PER_FOOT, 0);

            return null;
        }

        // Now we wait
        if (movementThread.hasNothingToDo()) {
            return AutonState.STOP;
        }

        return null;
    }

    private AutonState doDriveToC(long timesRun) {
        if (timesRun == 0) {
            // TODO: I think we can just figure out what the angle to drive at should be
            addInstruction(DRIVE_DISTANCE, 2 * MM_PER_FOOT, -15.0, 0.5);
            addInstruction(DRIVE_DISTANCE, -2 * MM_PER_FOOT, -15.0, 0.5);

            return null;
        }

        // Now we wait
        if (movementThread.hasNothingToDo()) {
            return AutonState.STOP;
        }

        return null;
    }

    private AutonState doStop(long timesRun) {
        if (gamepad1.x) {
            return AutonState.START;
        }

        return null;
    }

    private void addInstruction(MovementInstruction.InstructionType instructionType, double... parameters) {
        MovementInstruction i = new MovementInstruction(instructionType, parameters);
        movementThread.addInstruction(i);
    }
}
