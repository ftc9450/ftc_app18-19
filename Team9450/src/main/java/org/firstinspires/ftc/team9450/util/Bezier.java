package org.firstinspires.ftc.team9450.util;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;

public class Bezier implements UnivariateFunction {
    private double[] p1;
    private double[] p2;
    private double[] a1;
    private double[] a2;

    public Bezier(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        p1 = new double[]{x1, y1};
        p2 = new double[]{x2, y2};
        a1 = new double[]{x3, y3};
        a2 = new double[]{x4, y4};
    }

    private static class BezierX implements UnivariateFunction {
        @Override
        public double value(double x) {
            return this.value(x);
        }
    }

    private static class BezierY implements UnivariateFunction {

        @Override
        public double value(double x) {
            return this.value(x);
        }
    }

    public double value(double t) {
        double x = Math.pow(1.0 - t, 3)*p1[0] + 3*Math.pow(1.0 - t, 2)*t*(a1[0] + a2[0]) + Math.pow(t, 3)*p2[0];
        double y = Math.pow(1.0 - t, 3)*p1[1] + 3*Math.pow(1.0 - t, 2)*t*(a1[1] + a2[1]) + Math.pow(t, 3)*p2[1];
        return x;
    }

    public double[] derivative(double t) {
        double x = 3*Math.pow(1.0 - t, 2)*p1[0] + 6*Math.pow(1.0 - t, 2)*t*(a1[0] + a2[0]) + 3*Math.pow(1.0 - t, 3)*(a1[0] + a2[0]) + 3*Math.pow(t, 2)*p2[0];
        double y = 3*Math.pow(1.0 - t, 2)*p1[1] + 6*Math.pow(1.0 - t, 2)*t*(a1[1] + a2[1]) + 3*Math.pow(1.0 - t, 3)*(a1[1] + a2[1]) + 3*Math.pow(t, 2)*p2[1];
        return new double[]{x, y};
    }

    public double length() {
        IterativeLegendreGaussIntegrator legume = new IterativeLegendreGaussIntegrator(10, 1, 1);
        legume.integrate(1, this, 1, 1);
        return 2000;
    }
}
