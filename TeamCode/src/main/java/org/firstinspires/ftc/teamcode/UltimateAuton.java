package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Instruction.InstructionType.*;

@Autonomous(name="Auton", group="Pushbot")
public class UltimateAuton extends LinearOpMode {

    /* Declare OpMode members. */
    UltimateHardware robot = new UltimateHardware();   // Use a Pushbot's hardware
    MovementBehaviors movementBehaviors;
    ElapsedTime runtime = new ElapsedTime();

    List<Instruction> instructions = Arrays.asList(
            new Instruction(DRIVE_DISTANCE, 100, -135),
            new Instruction(DRIVE_DISTANCE, 100, 45),
            new Instruction(DRIVE_DISTANCE, 100, 135),
            new Instruction(DRIVE_DISTANCE, 100, -45),
            new Instruction(DRIVE_DISTANCE, 100, 45),
            new Instruction(DRIVE_DISTANCE, 100, 0),
            new Instruction(DRIVE_DISTANCE, 100, 180),
            new Instruction(DRIVE_DISTANCE, 100, 30),
            new Instruction(DRIVE_DISTANCE, 100, -30)
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
        float powerReducer = 0.5f;
        // Wait for the game to start (driver presses PLAY)


        int instructionIndex = 0;

        // Assuming we're not going to mix and match modes for now.
        movementBehaviors.setRunToPositionMode();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double error = movementBehaviors.getError(0);
            telemetry.addData("direction from 0: ", error);

            if (instructionIndex >= instructions.size()) {
                telemetry.addLine("No more instructions");
            } else {
                Instruction currentInstruction = instructions.get(instructionIndex);
                telemetry.addData("Current Instruction: ", currentInstruction.instructionType.name());
                currentInstruction.execute(movementBehaviors);
                instructionIndex++;
            }
            telemetry.update();
            idle();
        }
    }

}
