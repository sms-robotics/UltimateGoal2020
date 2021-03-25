package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MovementBehaviors {

    private static final double TURN_TICKS_PER_DEGREE = 14.5;
    private static final double TICKS_PER_MILLIMETER = 3.0;
    private static final double TURN_POWER = 0.125;
    private static final double TURN_ERROR_THRESHOLD_IN_DEGREES = 1;
    private static final double TURN_TIMEOUT_IN_MS = 4000;
    private static final double MIN_MOTOR_POWER = 0.0;
    private static final double MAX_MOTOR_POWER = 1.0;
    private static final double MIN_DRIVE_POWER = 0.01;
    private static final double MAX_DRIVE_POWER = 0.95;
    private static final double MIN_TURN_POWER = MIN_MOTOR_POWER;
    private static final double MAX_TURN_POWER = 0.5;
    private static final double DEFAULT_MOTOR_DRIVE_POWER = 0.5;
    private static final double DEFAULT_DRIVE_ANGLE = 0;

    private final ActionConveyor conveyor;
    private final ActionShooter shooter;
    private final ActionTrigger trigger;
    private final ActionWobbleArm wobbleArm;
    private final SensorIMU sensorImu;

    private enum MOTOR {
        FR,
        FL,
        RR,
        RL,
    };

    LinearOpMode opMode;
    HardwareUltimate robot;
    Telemetry telemetry;
    ElapsedTime runtime = new ElapsedTime();

    public MovementBehaviors(LinearOpMode opMode,
                             HardwareUltimate robot,
                             ActionConveyor conveyor,
                             ActionShooter shooter,
                             ActionTrigger trigger,
                             ActionWobbleArm wobbleArm,
                             SensorIMU sensorImu) {
        this.opMode = opMode;
        this.robot = robot;
        this.telemetry = opMode.telemetry;
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.trigger = trigger;
        this.wobbleArm = wobbleArm;
        this.sensorImu = sensorImu;
    }

    /*
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (in degrees relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */

    public double getError(double targetAngle) {
        double robotError;
        double currentAngle = sensorImu.getAngle();

        robotError = targetAngle - currentAngle;

        return robotError;
    }

    private double motorPowerCompensator(MOTOR motor, double power) {
        if (motor == MOTOR.FR) {
            return 1.03 * power;
        }
        if (motor == MOTOR.FL) {
            return 1.0 * power;
        }
        if (motor == MOTOR.RR) {
            return 1.0 * power;
        }
        if (motor == MOTOR.RL) {
            return 1.0 * power;
        }

        return power;
    }

    public void driveDistance(double distanceMm)  {
        driveDistance(distanceMm, DEFAULT_DRIVE_ANGLE);
    }

    public void driveDistance(double distanceMm, double angle)  {
        driveDistance(distanceMm, angle, DEFAULT_MOTOR_DRIVE_POWER);
    }

    public void driveDistance(double distanceMm, double angle, double power)
    {
        double clippedPower = Range.clip(power, MIN_DRIVE_POWER, MAX_DRIVE_POWER);
        setStraightDrivingModes();

        double targetRadAngle = Math.toRadians(angle);
        double radAngle = targetRadAngle;

        double angleX = Math.sin(radAngle);
        double angleY = Math.cos(radAngle);

        int targetPositionForward = (int)(distanceMm * TICKS_PER_MILLIMETER * angleY);
        int targetPositionRight = -(int)(distanceMm * TICKS_PER_MILLIMETER * angleX);
        int frontRightTarget = robot.frontRightDrive.getCurrentPosition() + targetPositionForward + targetPositionRight;
        int frontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + targetPositionForward - targetPositionRight;
        int rearRightTarget = robot.rearRightDrive.getCurrentPosition() + targetPositionForward - targetPositionRight;
        int rearLeftTarget = robot.rearLeftDrive.getCurrentPosition() + targetPositionForward + targetPositionRight;

        robot.frontRightDrive.setTargetPosition(frontRightTarget);
        robot.frontLeftDrive.setTargetPosition(frontLeftTarget);
        robot.rearRightDrive.setTargetPosition(rearRightTarget);
        robot.rearLeftDrive.setTargetPosition(rearLeftTarget);

        double frontRightDriveAmount = (angleY - angleX);
        double frontLeftDriveAmount = (angleY + angleX);
        double rearRightDriveAmount = (angleY + angleX);
        double rearLeftDriveAmount = (angleY - angleX);

        double frontRightPower = motorPowerCompensator(MOTOR.FR,
            clippedPower * frontRightDriveAmount);
        double frontLeftPower = motorPowerCompensator(MOTOR.FL,
            clippedPower * frontLeftDriveAmount);
        double rearRightPower = motorPowerCompensator(MOTOR.RR,
            clippedPower * rearRightDriveAmount);
        double rearLeftPower = motorPowerCompensator(MOTOR.RL,
            clippedPower * rearLeftDriveAmount);

        // If an extremely small power is set, the motor might indicate "busy" for a very long time, so don't do that.
        if (Math.abs(frontRightPower) > MIN_DRIVE_POWER) robot.frontRightDrive.setPower(frontRightPower);
        if (Math.abs(frontLeftPower) > MIN_DRIVE_POWER) robot.frontLeftDrive.setPower(frontLeftPower);
        if (Math.abs(rearRightPower) > MIN_DRIVE_POWER) robot.rearRightDrive.setPower(rearRightPower);
        if (Math.abs(rearLeftPower) > MIN_DRIVE_POWER) robot.rearLeftDrive.setPower(rearLeftPower);

        while (opMode.opModeIsActive() && (robot.frontRightDrive.isBusy() || robot.frontLeftDrive.isBusy() || robot.rearRightDrive.isBusy() || robot.rearLeftDrive.isBusy() )) {
            // Update telemetry & Allow time for other processes to run
            // telemetry.addData("Drive for distance: ", distanceMm);
            // telemetry.addData("target: ", Math.toDegrees(targetRadAngle));
            // telemetry.addData("angleX: ", Math.toDegrees(angleX));
            // telemetry.addData("angleY: ", Math.toDegrees(angleY));
            // telemetry.update();
            opMode.idle();
        }
    }

    public void controlledDriveAtHeading(double distanceMm, double heading, double power)
    {
        double clippedPower = Range.clip(power, MIN_DRIVE_POWER, MAX_DRIVE_POWER);
        setStraightDrivingModes();

        double targetRadHeading = Math.toRadians(heading);
        double angleX = Math.sin(targetRadHeading);
        double angleY = Math.cos(targetRadHeading);

        double sumOfAllPositionDeltas = 99999;
        int timeoutInMs = 1000;
        int clippedTimeoutInMs = Math.max(timeoutInMs, 0);

        ElapsedTime elapsedTime = new ElapsedTime();
        sensorImu.startAccelerationIntegration();

        while (true
            && !opMode.isStopRequested()
            && elapsedTime.milliseconds() < clippedTimeoutInMs) {

            double headingError = getError(heading);

            // Adjust based on angle of field
            double angleOfFieldInDegrees = sensorImu.getAngle();
            double angleOfFieldInRadians = Math.toRadians(angleOfFieldInDegrees);

            double leftX = 0; //Math.sin(targetRadHeading);
            double leftY = 1;//-Math.cos(targetRadHeading);

            // float refAngle = (float)targetRadHeading;

            // float leftX = (float) (gamepad1LeftX * Math.cos(refAngle) + gamepad1LeftY * Math.sin(refAngle));
            // float leftY = (float) (gamepad1LeftY * Math.cos(refAngle) - gamepad1LeftX * Math.sin(refAngle));

            double frontRight = leftY + leftX;
            double frontLeft = leftY - leftX;
            double backRight = leftY - leftX;
            double backLeft = leftY + leftX;

            RobotLog.i("FOO", String.format("frontRight: %f", frontRight));
            RobotLog.i("FOO", String.format("frontLeft: %f", frontLeft));

            // clip the right/left values so that the values never exceed +/- 1
            frontRight = Range.clip(frontRight, -1, 1) * power/2;
            frontLeft = Range.clip(frontLeft, -1, 1) * power/2;
            backLeft = Range.clip(backLeft, -1, 1) * power/2;
            backRight = Range.clip(backRight, -1, 1) * power/2;

            // write the values to the motors
            robot.frontRightDrive.setPower(frontRight);
            robot.frontLeftDrive.setPower(frontLeft);
            robot.rearLeftDrive.setPower(backLeft);
            robot.rearRightDrive.setPower(backRight);

            // int targetPositionForward = (int)(distanceMm * TICKS_PER_MILLIMETER * angleY);
            // int targetPositionRight = -(int)(distanceMm * TICKS_PER_MILLIMETER * angleX);
            // int frontRightTarget = robot.frontRightDrive.getCurrentPosition() + targetPositionForward + targetPositionRight;
            // int frontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + targetPositionForward - targetPositionRight;
            // int rearRightTarget = robot.rearRightDrive.getCurrentPosition() + targetPositionForward - targetPositionRight;
            // int rearLeftTarget = robot.rearLeftDrive.getCurrentPosition() + targetPositionForward + targetPositionRight;

            // // robot.frontRightDrive.setTargetPosition(frontRightTarget);
            // // robot.frontLeftDrive.setTargetPosition(frontLeftTarget);
            // // robot.rearRightDrive.setTargetPosition(rearRightTarget);
            // // robot.rearLeftDrive.setTargetPosition(rearLeftTarget);

            // int frontRightActual = robot.frontRightDrive.getCurrentPosition();
            // int frontLeftActual = robot.frontLeftDrive.getCurrentPosition();
            // int rearRightActual = robot.rearRightDrive.getCurrentPosition();
            // int rearLeftActual = robot.rearLeftDrive.getCurrentPosition();

            // sumOfAllPositionDeltas = Math.abs(frontRightActual - frontRightTarget)
            //     + Math.abs(frontLeftActual - frontLeftTarget)
            //     + Math.abs(rearRightActual - rearRightTarget)
            //     + Math.abs(rearLeftActual - rearLeftTarget);

            RobotLog.i(sensorImu.getPosition().toString());
        }

        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);

        sensorImu.stopAccelerationIntegration();
    }

    public void driveForTime(double forward, double right, double msToRun)
    {
        robot.frontRightDrive.setPower(forward + right);
        robot.frontLeftDrive.setPower(forward - right);
        robot.rearRightDrive.setPower(forward - right);
        robot.rearLeftDrive.setPower(forward + right);
        double intRunTime = runtime.milliseconds() + msToRun;

        while (runtime.milliseconds() < intRunTime) {
            telemetry.addData("drive-l", forward);
            telemetry.addData("drive-r", right);
            telemetry.addData("time", runtime.milliseconds());
            telemetry.update();
            opMode.idle();
        }

        robot.frontRightDrive.setPower(0.0);
        robot.frontLeftDrive.setPower(0.0);
        robot.rearRightDrive.setPower(0.0);
        robot.rearLeftDrive.setPower(0.0);
    }

    public void turn(double degrees) {
        telemetry.addData("Turning ", degrees);
        telemetry.addData("encoder pos ", robot.frontRightDrive.getCurrentPosition());
        telemetry.update();

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

        while (opMode.opModeIsActive() && (robot.frontRightDrive.isBusy() || robot.frontLeftDrive.isBusy() || robot.rearRightDrive.isBusy() || robot.rearLeftDrive.isBusy() )) {
            // Update telemetry & Allow time for other processes to run
            telemetry.update();
            // opMode.idle();
            Thread.yield();
        }
    }

    public void startTurningLeft(double power) {
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rearLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.frontRightDrive.setPower(power);
        robot.frontLeftDrive.setPower(power);
        robot.rearRightDrive.setPower(power);
        robot.rearLeftDrive.setPower(power);
    }

    public void startTurningRight(double power) {
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rearRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.frontRightDrive.setPower(power);
        robot.frontLeftDrive.setPower(power);
        robot.rearRightDrive.setPower(power);
        robot.rearLeftDrive.setPower(power);
    }

    public void stopWheels() {
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
    }

    public void turnTo(double degrees) {
        ElapsedTime elapsedTime = new ElapsedTime();

        while (opMode.opModeIsActive() && elapsedTime.milliseconds() < TURN_TIMEOUT_IN_MS) {
            double error = getError(degrees);

            double targetTurnPower = Math.abs(error) / 90.0;
            double normalizedTurnPower = Range.clip(targetTurnPower, MIN_TURN_POWER, MAX_TURN_POWER);

            // telemetry.addData("Error is: ", error);
            // telemetry.update();
            if (Math.abs(error) < TURN_ERROR_THRESHOLD_IN_DEGREES) {
                stopWheels();
                Thread.yield();
                // opMode.idle();
                break;
            } else if (error < 0) {
                startTurningLeft(normalizedTurnPower);
            } else {
                startTurningRight(normalizedTurnPower);
            }
            Thread.yield();
            // opMode.idle();
        }

        stopWheels();
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

    public void setStraightDrivingModes() {
        setRunToPositionMode();
        robot.frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void waitForTimeInMilliseconds(double millisecondsToWait) {
        double msRunTime = runtime.milliseconds() + millisecondsToWait;

        while (runtime.milliseconds() < msRunTime && opMode.opModeIsActive()) {
            // opMode.idle();
            Thread.yield();
        }
    }

    public void turnOnCoveyor(double parameter) {
        conveyor.turnOnAtPower(parameter);
    }

    public void turnOffCoveyor() {
        conveyor.turnOff();
    }

    public void turnOnShooter(double parameter) {
        shooter.turnOnAtPower(parameter);
    }

    public void turnOffShooter() {
        shooter.turnOff();
    }

    public void lowerWobbleArmCompletely() {
        wobbleArm.lowerArmFully();
    }

    public void raiseWobbleArmCompletely() {
        wobbleArm.raiseArmFully();
    }

    public void moveWobbleArmToPosition(double positionAsPercentage, double atPower) {
        wobbleArm.moveArmToPosition(positionAsPercentage, atPower);
    }

    public void fireTrigger(double millisecondsToWaitForTriggerSweep) {
        trigger.fire();

        waitForTimeInMilliseconds(millisecondsToWaitForTriggerSweep);

        trigger.resetPosition();

        waitForTimeInMilliseconds(millisecondsToWaitForTriggerSweep);
    }

    public void waitForWobbleArm(double timeoutInMs) {
        wobbleArm.waitForWobbleArm((int)timeoutInMs);
    }
}
