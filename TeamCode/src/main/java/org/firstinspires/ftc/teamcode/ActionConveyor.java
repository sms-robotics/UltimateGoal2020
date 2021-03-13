package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ActionConveyor {
    private final DcMotor conveyorMotor;

    public ActionConveyor(DcMotor conveyorMotor) {
        this.conveyorMotor = conveyorMotor;
    }

    public void initialize() {
        conveyorMotor.setDirection(DcMotor.Direction.REVERSE);
        conveyorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnOn() {
        conveyorMotor.setPower(0.5);
    }

    public void turnOff() {
        conveyorMotor.setPower(0);
    }
}
