package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class ActionShooter {
    public static final double INITIAL_SPEED = 1.0;

    private final DcMotor shooterMotor;
    private double speed;

    public ActionShooter(DcMotor conveyorMotor) {
        this.shooterMotor = conveyorMotor;
        speed = INITIAL_SPEED;
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

    public void turnOnAtPower(double power) {
        double clippedPower = Range.clip(power, 0.0, 1.0);
        speed = clippedPower;
        shooterMotor.setPower(clippedPower);
    }
}
