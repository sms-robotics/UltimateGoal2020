package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class MovementBehaviors {

    private static final double TURN_TICKS_PER_DEGREE = 14.5;
    private static final double TICKS_PER_MILLIMETER = 8.0;
    private static final double TURN_POWER = 0.5;
    private static final double DRIVE_DISTANCE_POWER = 0.5;
    private static final double TURN_ERROR_THRESHOLD = 0.2;

    LinearOpMode opMode;
    UltimateHardware robot;
    Telemetry telemetry;
    ElapsedTime runtime = new ElapsedTime();

    public MovementBehaviors(LinearOpMode opMode, UltimateHardware robot) {
        this.opMode = opMode;
        this.robot = robot;
        this.telemetry = opMode.telemetry;
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
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public void driveDistance(double distanceMm, double angle)
    {
        setStraightDrivingModes();

        double radAngle = Math.toRadians(angle);
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

        // holonomic formulas
//        float FrontRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
//        float FrontLeft = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
//        float BackRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
//        float BackLeft = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;

        double frontRightDriveAmount = (angleY - angleX);
        double frontLeftDriveAmount = (angleY + angleX);
        double rearRightDriveAmount = (angleY + angleX);
        double rearLeftDriveAmount = (angleY - angleX);

        double frontRightPower = DRIVE_DISTANCE_POWER * frontRightDriveAmount;
        double frontLeftPower = DRIVE_DISTANCE_POWER * frontLeftDriveAmount;
        double rearRightPower = DRIVE_DISTANCE_POWER * rearRightDriveAmount;
        double rearLeftPower = DRIVE_DISTANCE_POWER * rearLeftDriveAmount;

        // If an extremely small power is set, the motor might indicate "busy" for a very long time, so don't do that.
        if (Math.abs(frontRightPower) > .01) robot.frontRightDrive.setPower(frontRightPower);
        if (Math.abs(frontLeftPower) > .01) robot.frontLeftDrive.setPower(frontLeftPower);
        if (Math.abs(rearRightPower) > .01) robot.rearRightDrive.setPower(rearRightPower);
        if (Math.abs(rearLeftPower) > .01) robot.rearLeftDrive.setPower(rearLeftPower);

        while (opMode.opModeIsActive() && (robot.frontRightDrive.isBusy() || robot.frontLeftDrive.isBusy() || robot.rearRightDrive.isBusy() || robot.rearLeftDrive.isBusy() )) {
            // Update telemetry & Allow time for other processes to run
            telemetry.addData("Drive for distance: ", distanceMm);
            telemetry.addData("angleX: ", angleX);
            telemetry.addData("angleY: ", angleY);
            telemetry.addData("frontRightDriveAmount: ", frontRightDriveAmount);
            telemetry.addData("frontLeftDriveAmount: ", frontLeftDriveAmount);
            telemetry.addData("rearRightDriveAmount: ", rearRightDriveAmount);
            telemetry.addData("rearLeftDriveAmount: ", rearLeftDriveAmount);
            telemetry.update();
            opMode.idle();
        }
    }

    public void driveForTime(double forward, double right, double msToRun)
    {
        telemetry.update();

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
            opMode.idle();
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

    public void startDriving(double angle) {

        double distanceMm = 10000;
        setStraightDrivingModes();

        double radAngle = Math.toRadians(angle);
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

        // holonomic formulas
//        float FrontRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
//        float FrontLeft = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
//        float BackRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
//        float BackLeft = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;

        double frontRightDriveAmount = (angleY - angleX);
        double frontLeftDriveAmount = (angleY + angleX);
        double rearRightDriveAmount = (angleY + angleX);
        double rearLeftDriveAmount = (angleY - angleX);

        double frontRightPower = DRIVE_DISTANCE_POWER * frontRightDriveAmount;
        double frontLeftPower = DRIVE_DISTANCE_POWER * frontLeftDriveAmount;
        double rearRightPower = DRIVE_DISTANCE_POWER * rearRightDriveAmount;
        double rearLeftPower = DRIVE_DISTANCE_POWER * rearLeftDriveAmount;

        // If an extremely small power is set, the motor might indicate "busy" for a very long time, so don't do that.
        if (Math.abs(frontRightPower) > .01) robot.frontRightDrive.setPower(frontRightPower);
        if (Math.abs(frontLeftPower) > .01) robot.frontLeftDrive.setPower(frontLeftPower);
        if (Math.abs(rearRightPower) > .01) robot.rearRightDrive.setPower(rearRightPower);
        if (Math.abs(rearLeftPower) > .01) robot.rearLeftDrive.setPower(rearLeftPower);

        // Update telemetry & Allow time for other processes to run
        telemetry.addData("Start driving at angle: ", angle);
        telemetry.addData("angleX: ", angleX);
        telemetry.addData("angleY: ", angleY);
        telemetry.addData("frontRightDriveAmount: ", frontRightDriveAmount);
        telemetry.addData("frontLeftDriveAmount: ", frontLeftDriveAmount);
        telemetry.addData("rearRightDriveAmount: ", rearRightDriveAmount);
        telemetry.addData("rearLeftDriveAmount: ", rearLeftDriveAmount);
        opMode.idle();
    }

    public void stopWheels() {
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
    }


    public void turnTo(double degrees) {
        telemetry.addData("Turning To: ", degrees);
        telemetry.addData("encoder pos ", robot.frontRightDrive.getCurrentPosition());
        telemetry.update();

        // TODO: Ease into movement instead of going directly to max power.

        double error = getError(degrees);

        double turnPower = TURN_POWER;

        if (error > 0) {
            startTurningLeft(turnPower);
        } else {
            startTurningRight(turnPower);
        }
        while (opMode.opModeIsActive()) {
            error = getError(degrees);

            if (Math.abs(error) < 15 && turnPower > TURN_POWER * .2) {
                turnPower *= .95;
                robot.frontRightDrive.setPower(turnPower);
                robot.frontLeftDrive.setPower(turnPower);
                robot.rearRightDrive.setPower(turnPower);
                robot.rearLeftDrive.setPower(turnPower);
            }

            telemetry.addData("Error is: ", error);
            telemetry.update();
            if (Math.abs(error) < TURN_ERROR_THRESHOLD) {
                stopWheels();
                opMode.idle();
                break;
            } else if (error > 0) {
                startTurningLeft(turnPower);
            } else {
                startTurningRight(turnPower);
            }
            opMode.idle();
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

    public void setStraightDrivingModes() {
        setRunToPositionMode();
        robot.frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
