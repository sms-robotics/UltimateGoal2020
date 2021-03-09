/*
Copyright 2018 FIRST Tech Challenge Team 10644

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp(name = "SMS TeleOp", group = "Production")
public class OpModeOfficialTeleOp extends LinearOpMode {

    HardwareUltimate robot = new HardwareUltimate();

    float driveNominalPower = 0.3f;

    boolean previousDPD = false;
    boolean previousDPU = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, true);

        // Create and initialize all of our different parts
        ActionConveyor conveyor = robot.createAndInitializeConveyor();
        ActionShooter shooter = robot.createAndInitializeShooter();
        ActionTrigger trigger = robot.createAndInitializeTrigger();
        ActionWobbleArm wobbleArm = robot.createAndInitializeWobbleArm();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        float powerReducer = 0.5f;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Wherever the wobble arm is when you press PLAY is where
        // it thinks "zero" is
        wobbleArm.rememberThisAsTheZeroPosition();

        // Reset the trigger position so it's ready to fire
        trigger.resetPosition();

        // These should both be off to start
        shooter.turnOff();
        conveyor.turnOff();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = -gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;

            // holonomic formulas
            float frontRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            float frontLeft = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
            float backRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float backLeft = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;

            float gamepad2LeftY = gamepad2.left_stick_y;
            float gamepad2RightY = -gamepad2.right_stick_y;
            float gamepad2RightTrigger = gamepad2.right_trigger;
            float gamepad2LeftTrigger = gamepad2.left_trigger;
//
//            // Allow driver to select Tank vs POV by pressing START
//            boolean dpad_check = gamepad2.dpad_up;
//            if(dpad_check && (dpad_check != previousDPU)) {
//                telemetry.addLine("Player 2 D-Pad DOWN pressed");
//            }
//            previousDPU = dpad_check;
//
//            dpad_check = gamepad2.dpad_down;
//            if(dpad_check && (dpad_check != previousDPD)) {
//                telemetry.addLine("Player 2 D-Pad UP pressed");
//            }
//            previousDPD = dpad_check;

            powerReducer = driveNominalPower;
            if ( gamepad1.right_trigger > 0) {
                telemetry.addLine("Right trigger pressed");
                powerReducer = 1.0f;
            }
            if ( gamepad1.left_trigger > 0) {
                telemetry.addLine("Left trigger pressed");
                powerReducer = 0.1f;
            }

            // clip the right/left values so that the values never exceed +/- 1
            frontRight = Range.clip(frontRight, -1, 1) * powerReducer;
            frontLeft = Range.clip(frontLeft, -1, 1) * powerReducer;
            backLeft = Range.clip(backLeft, -1, 1) * powerReducer;
            backRight = Range.clip(backRight, -1, 1) * powerReducer;

            // write the values to the motors

            robot.frontRightDrive.setPower(frontRight);
            robot.frontLeftDrive.setPower(frontLeft);
            robot.rearLeftDrive.setPower(backLeft);
            robot.rearRightDrive.setPower(backRight);

            // The D-Pad will drive the wobble arm up and down
            if (gamepad1.dpad_up) {
                wobbleArm.raiseArm();
            } else if (gamepad1.dpad_down) {
                wobbleArm.lowerArm();
            }

            if (gamepad1.y) {
                conveyor.turnOff();
            } else if (gamepad1.x) {
                conveyor.turnOn();
            }

            if (gamepad1.b) {
                shooter.turnOff();
            } else if (gamepad1.a) {
                shooter.turnOn();
            }

            if (gamepad1.right_trigger > 0.1) {
                trigger.fireAndReturn();
            } else {
                trigger.resetPosition();
            }

            trigger.loop();

            //print out motor values
            telemetry.addLine()
                    .addData("front right", frontRight)
                    .addData("front left", frontLeft)
                    .addData("back left", backLeft)
                    .addData("back right", backRight);

            telemetry.update();
        }
    }
}
