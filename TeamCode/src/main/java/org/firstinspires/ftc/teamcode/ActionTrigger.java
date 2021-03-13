package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public class ActionTrigger {
    static final double STARTING_POSITION = 0;
    static final double TRIGGER_RESET_POSITION = STARTING_POSITION;
    static final double TRIGGER_MAX_POSITION = 0.5;
    static final double POINT_OF_RETURN_POSITION = TRIGGER_MAX_POSITION * 0.95;

    private final Servo triggerMotor;
    private enum CMD {
        RESET,
        FIRE_AND_RETURN,
    }

    private CMD currentCommand = CMD.RESET;

    public ActionTrigger(Servo triggerMotor) {
        this.triggerMotor = triggerMotor;
    }

    public void initialize() {
        triggerMotor.setDirection(Servo.Direction.REVERSE);
    }

    public void resetPosition() {
        triggerMotor.setPosition(STARTING_POSITION);
    }

    public void fireAndReturn() {
        currentCommand = CMD.FIRE_AND_RETURN;
        triggerMotor.setPosition(TRIGGER_MAX_POSITION);
    }

    public void loop() {
        // If we're told to fire and return, then we just need to
        // monitor the position until it exceeds mostly all the
        // way to 100%. At that time, we go back to a trigger
        // reset position.
//        if (currentCommand == CMD.FIRE_AND_RETURN) {
//            double currentPosition = triggerMotor.getController().getServoPosition(5);
//            RobotLog.ii("FOO", "getPosition(): %f", currentPosition);
//            if (currentPosition > POINT_OF_RETURN_POSITION) {
//                triggerMotor.setPosition(TRIGGER_RESET_POSITION);
//
//                // We set ourselves back to reset so we don't
//                // keep going.
//                currentCommand = CMD.RESET;
//            } else {
//                triggerMotor.setPosition(TRIGGER_MAX_POSITION);
//            }
//        }
//
//        if (currentCommand == CMD.RESET) {
//            triggerMotor.setPosition(TRIGGER_RESET_POSITION);
//        }
    }
}
