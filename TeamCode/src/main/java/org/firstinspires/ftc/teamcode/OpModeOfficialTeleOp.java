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
import com.qualcomm.robotcore.util.ElapsedTime;
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
@TeleOp(name = "Official TeleOp", group = "Production")
public class OpModeOfficialTeleOp extends LinearOpMode {

    HardwareUltimate robot = new HardwareUltimate();

    float driveNominalPower = 0.3f;

    boolean previousDPD = false;
    boolean previousDPU = false;
    boolean previousDPL = false;
    boolean previousDPR = false;
    boolean previousA = false;
    boolean previousY = false;
    boolean imuSteer = true;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, false);

        // Create and initialize all of our different parts
        ActionConveyor conveyor = robot.createAndInitializeConveyor();
        ActionShooter shooter = robot.createAndInitializeShooter();
        ActionTrigger trigger = robot.createAndInitializeTrigger();
        ActionWobbleArm wobbleArm = robot.createAndInitializeWobbleArm(this);
        SensorIMU imu = robot.createAndInitializeIMU(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        float powerReducer = 0.5f;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // IMPORTANT: we don't want to init the angle because
        // we don't know where auton left it
        //imu.resetAngle();

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
            float gamepad1LeftY = gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;

            // Adjust based on angle of field
            float angleOfFieldInDegrees = (float) imu.getAngle();
            float angleOfFieldInRadians = (float) Math.toRadians(angleOfFieldInDegrees);
            float refAngle = (float) (angleOfFieldInRadians + Math.PI/2);
            float gamepad1LeftXPrime = (float) (gamepad1LeftX * Math.cos(refAngle) + gamepad1LeftY * Math.sin(refAngle));
            float gamepad1LeftYPrime = (float) (gamepad1LeftY * Math.cos(refAngle) - gamepad1LeftX * Math.sin(refAngle));

            float leftX = -gamepad1LeftX;
            float leftY = -gamepad1LeftY;

            boolean gpadACheck = gamepad1.a;
            if (gpadACheck && (gpadACheck != previousA)) {
                imuSteer = !imuSteer;
            }
            previousA = gpadACheck;

            boolean gpadYCheck = gamepad1.y;
            if (gpadYCheck && (gpadYCheck != previousY)) {
                imu.resetAngle();
            }
            previousY = gpadYCheck;

            if (imuSteer) {
                leftX = gamepad1LeftXPrime;
                leftY = gamepad1LeftYPrime;
            }

            // holonomic formulas
            float frontRight = leftY + leftX - gamepad1RightX;
            float frontLeft = leftY - leftX + gamepad1RightX;
            float backRight = leftY - leftX - gamepad1RightX;
            float backLeft = leftY + leftX + gamepad1RightX;

            powerReducer = 1.0f;
            if (gamepad1.right_trigger > 0) {
                powerReducer = 1.0f;
            }
            if (gamepad1.left_trigger > 0) {
                powerReducer = 0.4f;
            }
            if (gamepad2.y) {
                powerReducer = 0.4f;
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

            // Job #1: conveyor
            // if (gamepad2.y){
            //     conveyor.turnOff();
            // }
            // else if(gamepad2.x){
            //     conveyor.turnOn();
            // }

            // Job #2: ARM
            if (gamepad2.dpad_up){
                wobbleArm.raiseArm();
            } else if (gamepad2.dpad_down){
                wobbleArm.lowerArm();
            }

            // Job #3: Shooter
            if (gamepad2.b){
                shooter.turnOff();
            } else if (gamepad2.a){
                shooter.turnOnAtPower(1.0);
            } else if (gamepad2.x){
                shooter.turnOnAtPower(0.65);
            }

            if (gamepad2.left_trigger > 0.10) {
                conveyor.turnOn();
            } else {
                conveyor.turnOff();
            }

            // if (gamepad2.right_trigger > 0.10) {
            //     shooter.turnOn();
            // } else {
            //     shooter.turnOff();
            // }

            // Job #2: ARM
            boolean dpad_check;
            dpad_check = gamepad2.dpad_left;
            if (dpad_check && (dpad_check != previousDPL)) {
                shooter.speedDown();
            }
            previousDPL = dpad_check;

            dpad_check = gamepad2.dpad_right;
            if (dpad_check && (dpad_check != previousDPR)) {
                shooter.speedUp();
            }
            previousDPR = dpad_check;

            /// Job #4: Trigger
            if(gamepad2.right_bumper){
                trigger.fire();
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

            telemetry.addData("Shooter Speed", "%.03f", shooter.getSpeed());
            telemetry.addData("Angle", "%.03f", angleOfFieldInDegrees);
            telemetry.addData("Mode", imuSteer ? "IMU" : "Normal");

            telemetry.update();
        }
    }
}
