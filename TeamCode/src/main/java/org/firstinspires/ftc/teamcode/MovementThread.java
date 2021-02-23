package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.LinkedBlockingDeque;
import java.util.concurrent.atomic.AtomicBoolean;

public class MovementThread implements Runnable {
    volatile LinkedBlockingDeque<Instruction> instructions = new LinkedBlockingDeque<>();
    UltimateAuton opMode;

    public MovementThread(UltimateAuton opMode) {
        this.opMode = opMode;
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
                Instruction currentInstruction = instructions.takeFirst();
                opMode.telemetry.addData("3", "");
                opMode.telemetry.update();
                double error = opMode.getError(0);
                opMode.telemetry.addData("4", "");
                opMode.telemetry.update();
                opMode.telemetry.addData("direction from 0: ", error);
                opMode.telemetry.addData("Current Instruction: ", currentInstruction.instructionType.name());

                opMode.telemetry.addData("5", "");
                opMode.telemetry.update();
                currentInstruction.execute(opMode);

                opMode.telemetry.addData("6", "");
                opMode.telemetry.update();
            } catch (InterruptedException e) {
                opMode.telemetry.addData("MovementThread take()", "interrupted");
                opMode.telemetry.update();
                e.printStackTrace();
            }
        }
    }

    public void addInstruction(Instruction i) {
        instructions.add(i);
    }

}

