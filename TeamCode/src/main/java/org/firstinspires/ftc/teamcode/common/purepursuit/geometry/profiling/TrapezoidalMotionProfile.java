package org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling;

public class TrapezoidalMotionProfile implements MotionProfile {
    public double maxV;
    public double maxA;
    public double distance;

    private double inverseTime;
    private final double epsilon = 1.401298E-45;
    private double dRad;
    private double tRad;
    private double tCir;
    private double dCir;
    private double aCur = 0.0;
    private double vCur = 0.0;
    private double pCur = 0.0;

    // thx daryoush <3
    public TrapezoidalMotionProfile(double maxV, double maxA, double distance) {
        this.maxV = maxV;
        this.maxA = maxA;
        this.distance = distance;

        this.inverseTime = Math.pow(maxV, 2) / Math.pow(maxA, 3);
    }

    public void recondition(double maxV, double maxA, double distance){
        this.maxV = maxV;
        this.maxA = maxA;
        this.distance = distance;

        this.inverseTime = Math.pow(maxV, 2) / Math.pow(maxA, 3);
    }

    @Override
    public double[] update(double time) {

        if (time < 0) {
            return new double[]{0, 0, 0};
        }

        if (inverseTime <= distance) {
            tRad = maxV / maxA;
            dRad = (maxA * Math.pow(tRad, 2)) / 2;
        } else {
            tRad = Math.sqrt(distance / maxA);
            dRad = distance / 2;
        }

        dCir = distance - (2 * dRad);
        tCir = dCir / maxV;

        //aCur = getAccel(time);
//        temporarily removing code to fix bug
//        if (Math.abs(aCur) > epsilon) {
//            vCur = getVelo(time);
//        }
//
//        if (Math.abs(vCur) > epsilon) {
//            pCur = getPos(time);
//        }

//        if (distance <= 0) {
//            pCur = Math.abs(distance) + pCur;
//        }
        aCur = getAccel(time);
        vCur = getVelo(time);
        pCur = getPos(time);
//        if (distance <= 0) {
//            pCur += Math.abs(distance);
//        }

        return new double[]{pCur, vCur, aCur};
    }

    public double[] update(double time, double start) {
        double[] updateFuncs = update(time);
        updateFuncs[0] += start;
        return updateFuncs;
    }

    private double getPos(double time) {
        if (time <= tRad) {
            return (maxA * Math.pow(time, 2)) / 2;
        } else if (time <= tRad + tCir) {
            return (dRad + ((time - tRad) * getVelo(tRad)));
        } else if (time - tRad - tCir <= tRad) {
            return dRad + dCir + (getVelo(tRad + tCir) * (time - tRad - tCir)) - ((maxA * Math.pow(time - tRad - tCir, 2)) / 2);
        } else {
            return distance;
        }
    }

    private double getVelo(double time) {
        if (time <= tRad) {
            return getAccel(time) * time;
        } else if ((time - tRad) <= tCir) {
            return maxV;
        } else if (time - tRad - tCir <= tRad) {
            return getAccel(tRad) * tRad - maxA * (time - tCir - tRad);
        } else {
            return 0.0;
        }
    }

    private double getAccel(double time) {
        if (time <= tRad) {
            return maxA;
        } else if (time <= (tRad + tCir)) {
            return 0.0;
        } else if (time <= 2 * tRad + tCir) {
            return -maxA;
        } else {
            return 0.0;
        }
    }
}
