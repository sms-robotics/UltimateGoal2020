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

import static org.firstinspires.ftc.teamcode.SoundManager.Sound.BAD;
import static org.firstinspires.ftc.teamcode.SoundManager.Sound.COIN;
import static org.firstinspires.ftc.teamcode.SoundManager.Sound.OK;

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
        VisionWebcamScanner webcamScanner = robot.createAndInitializeWebcamScanner();

        SoundManager soundManager = robot.createAndInitializeSoundManager();

        MovementBehaviors movement = robot.createAndInitializeMovementBehaviors(this, conveyor, shooter, trigger, wobbleArm, imu);
        VisionManager visionManager = robot.createAndInitializeVisionManager();

        visionManager.setEnableObjectDetection(false);
        visionManager.setDebugImageCaptureEnabled(false);
        visionManager.activate();

        webcamScanner.goToNeutral();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        float targetPower = 0.5f;

        soundManager.play(OK);

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

//        movement.setStraightDrivingModes();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            visionManager.loop();

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

//            boolean gpadACheck = gamepad1.a;
//            if (gpadACheck && (gpadACheck != previousA)) {
//                imuSteer = !imuSteer;
//            }
//            previousA = gpadACheck;

            boolean gpadYCheck = gamepad1.y;
            if (gpadYCheck && (gpadYCheck != previousY)) {
                imu.resetAngle();
            }
            previousY = gpadYCheck;

            if (imuSteer) {
                leftX = gamepad1LeftXPrime;
                leftY = gamepad1LeftYPrime;
            }

            targetPower = 1.0f;
            if (gamepad1.right_trigger > 0) {
                targetPower = 0.6f;
            }
            if (gamepad1.left_trigger > 0) {
                targetPower = 0.4f;
            }
            if (gamepad2.y) {
                targetPower = 0.4f;
            }

            // Have to keep track of whether the auto-move is doing the
            // work or whether we're using the other joystick inputs
            boolean handledMovement = false;
            if (gamepad1.left_bumper) {
                double[] desiredPosition = new double[] {/*X:*/1000.0, 0, /*Y:*/1000.0};
                double[] lastComputedLocation = visionManager.getLastComputedLocationFiltered();
                double lastSawTarget = visionManager.getHowManySecondsAgoSawBlueTarget();
                if (lastComputedLocation != null && lastSawTarget < 1.0) {
                    soundManager.play(COIN);

                    handledMovement = true;

                    double deltaX = -1 * (lastComputedLocation[0] - desiredPosition[0]);
                    double deltaY = (lastComputedLocation[2] - desiredPosition[2]);
                    double deltaTheta = lastComputedLocation[3] - Math.toRadians(-27.0);

                    double distanceToTravel = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
                    double angleToTravel = 0;
                    String q = "?";

                    if (deltaX > 0) {
                        if (deltaY > 0) {
                            q = "Q1";
                            // Quadrant 1, need to move right and forward
                            angleToTravel = Math.atan(deltaX / deltaY);
                        } else {
                            q = "Q2";
                            // Quadrant 2, need to move right and back
                            angleToTravel = Math.atan(-1 * deltaY / deltaX) + (Math.PI/2);
                        }
                    } else {
                        if (deltaY > 0) {
                            q = "Q4";
                            // Quadrant 4, need to move left and forward
                            angleToTravel = Math.atan(deltaY / (-1 * deltaX)) + (3 * Math.PI / 2);
                        } else {
                            q = "Q3";
                            // Quadrant 3, need to move left and back
                            angleToTravel = Math.atan((-1 * deltaX) / (-1 * deltaY)) + Math.PI;
                        }
                    }

                    double angleToFace = deltaTheta;
                    double seekPower = 0.8;
                    double power = Range.clip(seekPower * Range.clip(distanceToTravel / 300, 0.05, seekPower), 0, seekPower);

                    telemetry.addLine()
                        .addData("DX", deltaX)
                        .addData("DY", deltaY)
                        .addData("Q", q);

                    movement.driveInDirection(angleToTravel, angleToFace, power);
                } else {
                    // No lock on the target
                    soundManager.play(BAD);
                }
            }

            if (!handledMovement) {
                // holonomic formulas
                float frontRight = leftY + leftX - gamepad1RightX;
                float frontLeft = leftY - leftX + gamepad1RightX;
                float backRight = leftY - leftX - gamepad1RightX;
                float backLeft = leftY + leftX + gamepad1RightX;

                // clip the right/left values so that the values never exceed +/- 1
                frontRight = Range.clip(frontRight, -1, 1) * targetPower;
                frontLeft = Range.clip(frontLeft, -1, 1) * targetPower;
                backLeft = Range.clip(backLeft, -1, 1) * targetPower;
                backRight = Range.clip(backRight, -1, 1) * targetPower;

                // write the values to the motors
                robot.frontRightDrive.setPower(frontRight);
                robot.frontLeftDrive.setPower(frontLeft);
                robot.rearLeftDrive.setPower(backLeft);
                robot.rearRightDrive.setPower(backRight);
            }

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

//            telemetry.addLine()
//                .addData("X", leftX)
//                .addData("Y", leftY);

            telemetry.addData("Shooter Speed", "%.03f", shooter.getSpeed());
            telemetry.addData("Angle", "%.03f", angleOfFieldInDegrees);
//            telemetry.addData("Mode", imuSteer ? "IMU" : "Normal");
            double[] lastComputedLocation = visionManager.getLastComputedLocationFiltered();
            if (lastComputedLocation == null) {
                telemetry.addData("Position                        ", "Unknown");
            } else {
                telemetry.addData("Position X                      ", String.format("%.1f", lastComputedLocation[0]));
                telemetry.addData("Position Y                      ", String.format("%.1f", lastComputedLocation[2]));
                telemetry.addData("Position A                      ", String.format("%.1f", Math.toDegrees(lastComputedLocation[3])));
            }

            telemetry.update();
        }

        visionManager.shutdown();
    }
}
