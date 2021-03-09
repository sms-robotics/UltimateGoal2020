package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ActionShooter {
    private final DcMotor shooterMotor;

    public ActionShooter(DcMotor conveyorMotor) {
        this.shooterMotor = conveyorMotor;
    }

    public void initialize() {
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnOn() {
        shooterMotor.setPower(1);
    }

    public void turnOff() {
        shooterMotor.setPower(0);
    }
}
