package org.firstinspires.ftc.teamcode;

public class MovementInstruction {
    public InstructionType instructionType;
    public double parameters[];

    public MovementInstruction(InstructionType it, double... parameters) {
        this.instructionType = it;
        this.parameters = parameters;
    }

    public void execute(MovementBehaviors behaviors) {
        this.instructionType.action.execute(behaviors, this.parameters);
    }

    interface Action {
        void execute(MovementBehaviors behaviors, double... parameters);
    }

    public enum InstructionType {
        FIRE_RING(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.fireTrigger(1000);
            }
        }),

        WAIT_FOR_TIME(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.waitForTimeInMilliseconds(parameters[0]);
            }
        }),

        TURN_ON_CONVEYOR(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.turnOnConveyor(parameters[0]);
            }
        }),

        TURN_OFF_CONVEYOR(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.turnOffCoveyor();
            }
        }),

        TURN_ON_SHOOTER(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.turnOnShooter(parameters[0]);
            }
        }),

        TURN_OFF_SHOOTER(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.turnOffShooter();
            }
        }),

        DRIVE_FOR_TIME(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.driveForTime(parameters[0], parameters[1], parameters[2]);
            }
        }),

        DRIVE_DISTANCE(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                switch (parameters.length) {
                    // Only distance
                    case 1:
                        behaviors.driveDistance(parameters[0]);
                        break;

                    // distance and angle
                    case 2:
                        behaviors.driveDistance(parameters[0], parameters[1]);
                        break;

                    // distance and angle and power
                    case 3:
                        behaviors.driveDistance(parameters[0], parameters[1], parameters[2]);
                        break;

                    default:
                        break;
                }
            }
        }),

        DRIVE_PID_DISTANCE(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                switch (parameters.length) {
                    // distance and angle and power
                    case 4:
                        behaviors.nerdPidDrive(parameters[0], parameters[1], parameters[2], parameters[3], 10);
                        break;

                    default:
                        break;
                }
            }
        }),

        TURN_TO(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
//                behaviors.nerdPidTurn(parameters[0]);
                behaviors.turnTo(parameters[0]);
            }
        }),

        LOWER_WOBBLE_ARM(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.lowerWobbleArmCompletely();
            }
        }),

        RAISE_WOBBLE_ARM(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.raiseWobbleArmCompletely();
            }
        }),

        STOP_WHEELS(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.stopWheels();
            }
        });

        public Action action;
        InstructionType(Action action) {
            this.action = action;
        }
    }
}

