package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class ActionWobbleArm {
    static final int LOWER_ARM_ENCODER_COUNTS_INCREMENT = 25;
    static final int RAISE_ARM_ENCODER_COUNTS_INCREMENT = 25;
    static final int MAX_ENCODER_COUNTS = 1500;

    private final DcMotor armMotor;
    private int zeroPosition;
    private int maxPosition;
    private int currentCommandedPosition = Integer.MAX_VALUE;

    public ActionWobbleArm(DcMotor armMotor) {
        this.armMotor = armMotor;
    }

    public void initialize() {
        currentCommandedPosition = Integer.MAX_VALUE;
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rememberThisAsTheZeroPosition();
    }

    public void rememberThisAsTheZeroPosition() {
        zeroPosition = armMotor.getCurrentPosition();
        maxPosition = zeroPosition + MAX_ENCODER_COUNTS;
    }

    public void lowerArm() {
        lowerArmByAmount(LOWER_ARM_ENCODER_COUNTS_INCREMENT);
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
        armMotor.setPower(0.25);
    }

    public void raiseArm() {
        raiseArmByAmount(RAISE_ARM_ENCODER_COUNTS_INCREMENT);
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
        armMotor.setPower(0.25);
    }
}
