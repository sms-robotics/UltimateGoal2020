package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.LinkedBlockingDeque;

public class MovementThread implements Runnable {
    private final MovementBehaviors movementBehaviors;
    LinkedBlockingDeque<MovementInstruction> movementInstructions = new LinkedBlockingDeque<>();
    LinearOpMode opMode;

    public MovementThread(LinearOpMode opMode, MovementBehaviors movementBehaviors) {
        this.opMode = opMode;
        this.movementBehaviors = movementBehaviors;
    }

    @Override
    public void run() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        opMode.telemetry.addLine("Thread is waiting");
        opMode.telemetry.update();

        opMode.waitForStart();

        while (opMode.opModeIsActive()) {
//            opMode.telemetry.addData("2", "");
            //opMode.telemetry.update();
            try {
                MovementInstruction currentMovementInstruction = movementInstructions.takeFirst(); // Blocks until there is an instruction to take
                double error = movementBehaviors.getError(0);
//                opMode.telemetry.addData("direction from 0: ", error);
//                opMode.telemetry.addData("Current Instruction: ", currentInstruction.instructionType.name());
                currentMovementInstruction.execute(movementBehaviors);

//                opMode.telemetry.addData("6", "");
//                opMode.telemetry.update();
            } catch (InterruptedException e) {
//                opMode.telemetry.addData("MovementThread take()", "interrupted");
//                opMode.telemetry.update();
                e.printStackTrace();
            }
        }
    }

    public void addInstruction(MovementInstruction i) {
        movementInstructions.add(i);
    }

    // Tells you if we have any instructions in our queue
    public boolean hasNothingToDo() {
        return movementInstructions.isEmpty();
    }

    public long getHowManyInstructions() {
        return movementInstructions.size();
    }
}

