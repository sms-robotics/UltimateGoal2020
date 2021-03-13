package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class ActionShooter {
    private final DcMotor shooterMotor;
    private double speed;

    public ActionShooter(DcMotor conveyorMotor) {
        this.shooterMotor = conveyorMotor;
        speed = 0.5;
    }

    public void initialize() {
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void speedUp() {
        speed = Range.clip(speed + 0.05, 0.0, 1.0);

        if (shooterMotor.getPower() > 0) {
            shooterMotor.setPower(speed);
        }
    }

    public void speedDown() {
        speed = Range.clip(speed - 0.05, 0.0, 1.0);

        if (shooterMotor.getPower() > 0) {
            shooterMotor.setPower(speed);
        }
    }

    public void turnOn() {
        shooterMotor.setPower(speed);
    }

    public void turnOff() {
        shooterMotor.setPower(0);
    }

    public double getSpeed() {
        return this.speed;
    }
}
