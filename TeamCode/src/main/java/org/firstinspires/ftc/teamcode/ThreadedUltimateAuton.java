package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.firstinspires.ftc.teamcode.Instruction.InstructionType.DRIVE_DISTANCE;
import static org.firstinspires.ftc.teamcode.Instruction.InstructionType.START_DRIVING;
import static org.firstinspires.ftc.teamcode.Instruction.InstructionType.STOP_WHEELS;

@Autonomous(name="ThreadedAuton", group="Pushbot")
public class ThreadedUltimateAuton extends LinearOpMode {

    /* Declare OpMode members. */
    UltimateHardware robot = new UltimateHardware();   // Use a Pushbot's hardware
    ElapsedTime runtime = new ElapsedTime();
    MovementBehaviors movementBehaviors;

    List<Instruction> instructions = Arrays.asList(
            new Instruction(START_DRIVING,  -135),
            new Instruction(START_DRIVING, 45),
            new Instruction(START_DRIVING, 135),
            new Instruction(START_DRIVING, -45),
            new Instruction(START_DRIVING, 45),
            new Instruction(START_DRIVING, 0),
            new Instruction(START_DRIVING, 180),
            new Instruction(START_DRIVING, 30),
            new Instruction(START_DRIVING, -30),
            new Instruction(STOP_WHEELS)
    );

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, true);
        movementBehaviors = new MovementBehaviors(this, robot);
        //move to hardware code and only run in AUTON

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        MovementThread movementThread = new MovementThread(this, movementBehaviors);
        Thread t = new Thread(movementThread);
        t.start();

        int instructionIndex = 0;

        // Assuming we're not going to mix and match modes for now.
        movementBehaviors.setRunToPositionMode();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (instructionIndex >= instructions.size()) {
                telemetry.addLine("No more instructions");
            } else {
                Instruction currentInstruction = instructions.get(instructionIndex);
                movementThread.addInstruction(currentInstruction);
                try {
                    Thread.sleep(500, 0);
                } catch (InterruptedException e) {

                    telemetry.addLine("Sleep interrupted!");
                    telemetry.update();
                    e.printStackTrace();
                }
                instructionIndex++;
            }
            telemetry.update();
            idle();
        }
    }

}
