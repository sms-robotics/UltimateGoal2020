package org.firstinspires.ftc.teamcode;

class Instruction {
    public InstructionType instructionType;
    public double parameters[];

    Instruction(InstructionType it, double... parameters) {
        this.instructionType = it;
        this.parameters = parameters;
    }

    public void execute(UltimateAuton ua) {
        this.instructionType.action.execute(ua, this.parameters);
    }

    interface Action {
        void execute(UltimateAuton ua, double... parameters);
    }

    public enum InstructionType {
        DRIVE_FOR_TIME(new Action() {
            @Override
            public void execute(UltimateAuton ua, double... parameters) {
                ua.driveForTime(parameters[0], parameters[1], parameters[2]);
            }
        }),

        DRIVE_DISTANCE(new Action() {
            @Override
            public void execute(UltimateAuton ua, double... parameters) {
                ua.driveDistance(parameters[0], parameters[1]);
            }
        }),

        TURN(new Action() {
            @Override
            public void execute(UltimateAuton ua, double... parameters) {
                ua.turn(parameters[0]);
            }
        }),

        TURN_TO(new Action() {
            @Override
            public void execute(UltimateAuton ua, double... parameters) {
                ua.turnTo(parameters[0]);
            }
        }),
        START_DRIVING(new Action() {
            @Override
            public void execute(UltimateAuton ua, double... parameters) {
                ua.startDriving(parameters[0]);
            }
        }),
        STOP_WHEELS(new Action() {
            @Override
            public void execute(UltimateAuton ua, double... parameters) {
                ua.stopWheels();
            }
        });

        public Action action;
        InstructionType(Action action) {
            this.action = action;
        }
    }
}

