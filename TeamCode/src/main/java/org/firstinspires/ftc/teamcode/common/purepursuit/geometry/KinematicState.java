package org.firstinspires.ftc.teamcode.common.purepursuit.geometry;

public class KinematicState {
    public final int intakeStartingPos;
    public final int intakeEndPos;

    public final double intakeVelo;
    public final double intakeAccel;
    
    public final double fourbarStartPos;
    public final double fourbarEndPos;

    public double time = 0;

    private final double DEGREES_PER_TICK = 360 / 2.04;
    private final double ROTATIONS_PER_SECOND = 1.76;
    private final double TICKS_TO_INCHES = 23.5;
    private final double C2C_DISTANCE = 225 / 25.4;

    public KinematicState(int intakeStartingPos, double fourbarStartPos, double fourbarEndPos) {
        this.intakeStartingPos = intakeStartingPos;

        this.fourbarStartPos = fourbarStartPos;
        this.fourbarEndPos = fourbarEndPos;

        double deltaTheta = DEGREES_PER_TICK * (this.fourbarEndPos - this.fourbarStartPos);
        double time = deltaTheta / ROTATIONS_PER_SECOND;

        int deltaPosition = (int)((Math.cos(deltaTheta) * C2C_DISTANCE) * TICKS_TO_INCHES);

        this.intakeEndPos = this.intakeStartingPos + deltaPosition;

        this.intakeVelo = (this.intakeEndPos - this.intakeStartingPos) / time;
        this.intakeAccel = intakeVelo / time;
    }
}