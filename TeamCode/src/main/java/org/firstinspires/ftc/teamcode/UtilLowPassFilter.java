package org.firstinspires.ftc.teamcode;

public class UtilLowPassFilter {
    private double lastOutputValue = Double.NaN;
    private long lastInputTimestamp;
    private final double rcInMs;

    public UtilLowPassFilter(double rcInMs) {
        this.rcInMs = rcInMs;
    }

    public double addSample(double inputValue, long inputTimestamp) {
        if (Double.isNaN(lastOutputValue)) {
            lastOutputValue = inputValue;
            lastInputTimestamp = inputTimestamp;

            return inputValue;
        }

        double dt = inputTimestamp - lastInputTimestamp;
        double alpha = Math.max(0, Math.min(1, ((double)dt) / (dt + this.rcInMs)));

        double outputValue = (inputValue * alpha) + ((1-alpha) * lastOutputValue);

        lastOutputValue = outputValue;
        lastInputTimestamp = inputTimestamp;

        return outputValue;
    }

    public double getValue() {
        return lastOutputValue;
    }
}
