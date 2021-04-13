package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Wobble Arm

public class ActionWobbleArm {
    static final int LOWER_ARM_ENCODER_COUNTS_INCREMENT = 30;
    static final int RAISE_ARM_ENCODER_COUNTS_INCREMENT = 30;
    static final double ARM_MOTOR_SPEED = 1.0;
    static final int MAX_ENCODER_COUNTS = 1500;
    static final int LOWER_WOBBLE_TIMEOUT = 5000;
    static final int RAISE_WOBBLE_TIMEOUT = 5000;

    private final DcMotor armMotor;
    private final LinearOpMode opMode;
    private final TouchSensor WobbleUp;
    private final TouchSensor WobbleDown;

    private int zeroPosition;
    private int maxPosition;
    private int currentCommandedPosition = Integer.MAX_VALUE;

    public ActionWobbleArm(DcMotor armMotor, LinearOpMode opMode, TouchSensor touchUp, TouchSensor touchDown) {
        this.armMotor = armMotor;
        this.opMode = opMode;
        this.WobbleUp = touchUp;
        this.WobbleDown = touchDown;
    }

    public void initialize() {
        currentCommandedPosition = Integer.MAX_VALUE;
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rememberThisAsTheZeroPosition();
    }

    public void rememberThisAsTheZeroPosition() {
        zeroPosition = armMotor.getCurrentPosition();
        maxPosition = zeroPosition + MAX_ENCODER_COUNTS;
    }

    public void lowerArm() {
        lowerArmByAmountLimited(LOWER_ARM_ENCODER_COUNTS_INCREMENT);
    }

    public void lowerArmByAmount(int byAmountInEncoderCounts) {
        if (currentCommandedPosition == Integer.MAX_VALUE) {
            currentCommandedPosition = zeroPosition;
        }

        // Don't let it get too far ahead
        if (Math.abs(armMotor.getCurrentPosition() - currentCommandedPosition) > 2 * RAISE_ARM_ENCODER_COUNTS_INCREMENT) {
            return;
        }

        currentCommandedPosition = Range.clip(currentCommandedPosition + byAmountInEncoderCounts, zeroPosition, maxPosition);

        armMotor.setTargetPosition(currentCommandedPosition);
        armMotor.setPower(ARM_MOTOR_SPEED);
    }

    public void lowerArmByAmountLimited(int byAmountInEncoderCounts) {
        if (currentCommandedPosition == Integer.MAX_VALUE) {
            currentCommandedPosition = zeroPosition;
        }

        // Turn off motor and exit if at limit
        if (WobbleDown.isPressed() == true) {
            armMotor.setPower(0.0);
            currentCommandedPosition = armMotor.getCurrentPosition();
            return;
        }

        // Don't let it get too far ahead
        if (Math.abs(armMotor.getCurrentPosition() - currentCommandedPosition) > 2 * LOWER_ARM_ENCODER_COUNTS_INCREMENT) {
            return;
        }

        currentCommandedPosition = armMotor.getCurrentPosition() + byAmountInEncoderCounts;

        armMotor.setTargetPosition(currentCommandedPosition);
        armMotor.setPower(ARM_MOTOR_SPEED);
    }

    public void raiseArm() {
        raiseArmByAmountLimited(RAISE_ARM_ENCODER_COUNTS_INCREMENT);
    }

    public void lowerArmFully() {
        lowerArmFully(LOWER_WOBBLE_TIMEOUT);
    }

    public void lowerArmFully(int timeoutInMs) {
        int clippedTimeoutInMs = Math.max(timeoutInMs, 0);

        ElapsedTime elapsedTime = new ElapsedTime();
        while (!opMode.isStopRequested() && elapsedTime.milliseconds() < clippedTimeoutInMs) {
            lowerArmByAmountLimited(LOWER_ARM_ENCODER_COUNTS_INCREMENT);

            // Turn off motor and exit if at limit
            if (WobbleDown.isPressed() == true) {
                break;
            }
        }

        armMotor.setPower(0.0);
        maxPosition = armMotor.getCurrentPosition();
        currentCommandedPosition = armMotor.getCurrentPosition();
    }

    public void raiseArmFully() {
        raiseArmFully(RAISE_WOBBLE_TIMEOUT);
    }

    public void raiseArmFully(int timeoutInMs) {
        int clippedTimeoutInMs = Math.max(timeoutInMs, 0);

        ElapsedTime elapsedTime = new ElapsedTime();
        while (!opMode.isStopRequested() && elapsedTime.milliseconds() < clippedTimeoutInMs) {
            raiseArmByAmountLimited(RAISE_ARM_ENCODER_COUNTS_INCREMENT);

            // Turn off motor and exit if at limit
            if (WobbleUp.isPressed() == true) {
                break;
            }
        }

        armMotor.setPower(0.0);
        zeroPosition = armMotor.getCurrentPosition();
        currentCommandedPosition = armMotor.getCurrentPosition();
    }

    public void raiseArmByAmount(int byAmountInEncoderCounts) {
        if (currentCommandedPosition == Integer.MAX_VALUE) {
            currentCommandedPosition = zeroPosition;
        }

        // Don't let it get too far ahead
        if (Math.abs(armMotor.getCurrentPosition() - currentCommandedPosition) > 2 * LOWER_ARM_ENCODER_COUNTS_INCREMENT) {
            return;
        }

        currentCommandedPosition = Range.clip(currentCommandedPosition - byAmountInEncoderCounts, zeroPosition, maxPosition);

        armMotor.setTargetPosition(currentCommandedPosition);
        armMotor.setPower(ARM_MOTOR_SPEED);
    }

    public void raiseArmByAmountLimited(int byAmountInEncoderCounts) {
        if (currentCommandedPosition == Integer.MAX_VALUE) {
            currentCommandedPosition = zeroPosition;
        }

        // Turn off motor and exit if at limit
        if (WobbleUp.isPressed() == true) {
            armMotor.setPower(0.0);
            currentCommandedPosition = armMotor.getCurrentPosition();
            return;
        }

        // Don't let it get too far ahead
        // if (Math.abs(armMotor.getCurrentPosition() - currentCommandedPosition) > 2 * RAISE_ARM_ENCODER_COUNTS_INCREMENT) {
        //     return;
        // }

        currentCommandedPosition = armMotor.getCurrentPosition() - byAmountInEncoderCounts;

        armMotor.setTargetPosition(currentCommandedPosition);
        armMotor.setPower(ARM_MOTOR_SPEED);
    }

    public void moveArmToPosition(double percent, double atPower) {
        double clippedPower = Range.clip(atPower, 0.0, 1.0);
        double clippedPercentage = Range.clip(percent, 0.0, 1.0);
        // Typical formula for figuring out where in an
        // arbitrary range by percentage [0.0, 1.0]
        int targetPosition = (int)((double)(maxPosition - zeroPosition) * clippedPercentage) + zeroPosition;

        currentCommandedPosition = targetPosition;

        armMotor.setTargetPosition(currentCommandedPosition);
        armMotor.setPower(clippedPower);
    }

    public void waitForWobbleArm(int timeoutInMs) {
        int clippedTimeoutInMs = Math.max(timeoutInMs, 0);

        ElapsedTime elapsedTime = new ElapsedTime();
        while (!opMode.isStopRequested() && armMotor.getPower() > 0 && elapsedTime.milliseconds() < clippedTimeoutInMs) {
            opMode.idle();
        }
    }
}
