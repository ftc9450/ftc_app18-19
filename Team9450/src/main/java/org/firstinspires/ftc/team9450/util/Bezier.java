package org.firstinspires.ftc.team9450.util;

public class Bezier {
    private double[] p1;
    private double[] p2;
    private double[] a1;
    private double[] a2;

    final double RESOLUTION = 1000.0;

    public Bezier(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        p1 = new double[]{x1, y1};
        p2 = new double[]{x2, y2};
        a1 = new double[]{x3, y3};
        a2 = new double[]{x4, y4};
    }

    public double valueX(double t) {
        return Math.pow(1.0 - t, 3)*p1[0] + 3*Math.pow(1.0 - t, 2)*t*(a1[0] + a2[0]) + Math.pow(t, 3)*p2[0];
    }

    public double valueY(double t) {
        return Math.pow(1.0 - t, 3)*p1[1] + 3*Math.pow(1.0 - t, 2)*t*(a1[1] + a2[1]) + Math.pow(t, 3)*p2[1];
    }

    public double derivativeX(double t){
        return 3*Math.pow(1.0 - t, 2)*p1[0] + 6*Math.pow(1.0 - t, 2)*t*(a1[0] + a2[0]) + 3*Math.pow(1.0 - t, 3)*(a1[0] + a2[0]) + 3*Math.pow(t, 2)*p2[0];
    }

    public double derivativeY(double t){
        return 3*Math.pow(1.0 - t, 2)*p1[1] + 6*Math.pow(1.0 - t, 2)*t*(a1[1] + a2[1]) + 3*Math.pow(1.0 - t, 3)*(a1[1] + a2[1]) + 3*Math.pow(t, 2)*p2[1];
    }

    public double length() {
        double result = 0.0;
        for(double i = 0; i < RESOLUTION; i++){
            double dx = valueX((i + 1)/RESOLUTION) - valueX(i/RESOLUTION);
            double dy = valueY((i + 1)/RESOLUTION) - valueY(i/RESOLUTION);
            result += Math.sqrt(dx * dx + dy * dy);
        }
        return result;
    }
}
