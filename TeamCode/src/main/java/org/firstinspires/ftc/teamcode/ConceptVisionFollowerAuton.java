package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Demonstrates OpMode using the VisionManager
 */
@TeleOp(name = "Vision Follower", group = "Concept")
@Disabled
public class ConceptVisionFollowerAuton extends LinearOpMode {
    private static final String TAG = "Vision Follower";

    private static final double TURN_TICKS_PER_DEGREE = 14.5;
    private static final double TICKS_PER_MILLIMETER = 8.0;
    private static final double TURN_POWER = 0.5;
    private static final double DRIVE_DISTANCE_POWER = 0.5;

    /* Declare OpMode members. */
    HardwareUltimate robot = new HardwareUltimate();   // Use a Pushbot's hardware
    ElapsedTime runtime = new ElapsedTime();

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

        robot.init(hardwareMap, false);

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

                telemetry.addData("Target Visible  ",
                    visionManager.isBlueTargetVisible());

                double[] lastComputedLocation = visionManager.getLastComputedLocationFiltered();
                if (visionManager.isBlueTargetVisible() && lastComputedLocation != null) {
                    // X is less... point to right

                    double actualX, actualZ, desiredX, desiredZ;
                    desiredX = 914.4;
                    desiredZ = 400;

                    actualX = lastComputedLocation[0];
                    actualZ = lastComputedLocation[2];

                    double driveCommandedX = actualX - desiredX;
                    double driveCommandedZ = actualZ - desiredZ;
                    double angle = Math.toDegrees(Math.atan(driveCommandedX / driveCommandedZ));

                    if (Math.abs(driveCommandedZ) > 100) {
                        driveDistance(driveCommandedZ, 0);
                    } else {
                        if (Math.abs(angle) > 30) {
                            turn(angle * 1);
                        }
                    }

                    telemetry.addData("Position X                      ", String.format("%.1f", lastComputedLocation[0]));
                    telemetry.addData("Position Y                      ", String.format("%.1f", lastComputedLocation[1]));
                    telemetry.addData("Position Z                      ", String.format("%.1f", lastComputedLocation[2]));
                    telemetry.addData("Position Angle                  ", String.format("%.1f", angle));
                } else {
                    telemetry.addData("Position                        ", "Unknown");
                }

                telemetry.update();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG, e, "OpMode Loop Error");
        } finally {
            visionManager.shutdown();
        }
    }

    public void driveDistance(double distanceMm, double angle)
    {
        setRunToPositionMode();
//        telemetry.addData("Drive for distance: ", distanceMm);
//        telemetry.update();

        // TODO: Use real angle - Starting out with angle == 0
        int targetPositionForward = (int)(distanceMm * TICKS_PER_MILLIMETER);
        int targetPositionRight = 0;
        int frontRightTarget = robot.frontRightDrive.getCurrentPosition() + targetPositionForward + targetPositionRight;
        int frontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + targetPositionForward - targetPositionRight ;
        int rearRightTarget = robot.rearRightDrive.getCurrentPosition() + targetPositionForward - targetPositionRight;
        int rearLeftTarget = robot.rearLeftDrive.getCurrentPosition() + targetPositionForward - targetPositionRight;

        robot.frontRightDrive.setTargetPosition(frontRightTarget);
        robot.frontLeftDrive.setTargetPosition(frontLeftTarget);
        robot.rearRightDrive.setTargetPosition(rearRightTarget);
        robot.rearLeftDrive.setTargetPosition(rearLeftTarget);

        robot.frontRightDrive.setPower(DRIVE_DISTANCE_POWER);
        robot.frontLeftDrive.setPower(DRIVE_DISTANCE_POWER);
        robot.rearRightDrive.setPower(DRIVE_DISTANCE_POWER);
        robot.rearLeftDrive.setPower(DRIVE_DISTANCE_POWER);

        while (opModeIsActive() && (robot.frontRightDrive.isBusy() || robot.frontLeftDrive.isBusy() || robot.rearRightDrive.isBusy() || robot.rearLeftDrive.isBusy() )) {
            // Update telemetry & Allow time for other processes to run
//            telemetry.update();
            idle();
        }
    }

    public void turn(double degrees) {
        telemetry.addData("Turning ", degrees);
        telemetry.addData("encoder pos ", robot.frontRightDrive.getCurrentPosition());
        telemetry.update();

        setRunToPositionMode();
        int targetPosition = (int)(degrees * TURN_TICKS_PER_DEGREE);
        int frontRightTarget = robot.frontRightDrive.getCurrentPosition() - (int)(targetPosition);
        int frontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(targetPosition);
        int rearRightTarget = robot.rearRightDrive.getCurrentPosition() - (int)(targetPosition);
        int rearLeftTarget = robot.rearLeftDrive.getCurrentPosition() + (int)(targetPosition);

        robot.frontRightDrive.setTargetPosition(frontRightTarget);
        robot.frontLeftDrive.setTargetPosition(frontLeftTarget);
        robot.rearRightDrive.setTargetPosition(rearRightTarget);
        robot.rearLeftDrive.setTargetPosition(rearLeftTarget);

        // TODO: Ease into movement instead of going directly to max power.
        robot.frontRightDrive.setPower(TURN_POWER);
        robot.frontLeftDrive.setPower(TURN_POWER);
        robot.rearRightDrive.setPower(TURN_POWER);
        robot.rearLeftDrive.setPower(TURN_POWER);

        while (opModeIsActive() && (robot.frontRightDrive.isBusy() || robot.frontLeftDrive.isBusy() || robot.rearRightDrive.isBusy() || robot.rearLeftDrive.isBusy() )) {
            // Update telemetry & Allow time for other processes to run
//            telemetry.update();
            idle();
        }
    }

    public void setRunUsingEncoderMode() {
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRunToPositionMode() {
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
