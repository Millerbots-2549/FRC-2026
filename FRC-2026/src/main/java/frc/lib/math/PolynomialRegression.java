// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

import Jama.Matrix;
import Jama.QRDecomposition;

/** Add your docs here. */
public class PolynomialRegression {
  private int degree;
  private Matrix beta;
  private double sse;
  private double sst;

  public PolynomialRegression(double[][] xy, int degree) {
    double[] x = new double[xy.length];
    double[] y = new double[xy.length];
    for (int i = 0; i < xy.length; ++i) {
      x[i] = xy[i][0];
      y[i] = xy[i][1];
    }
    solve(x, y, degree);
  }

  private void solve(double[] x, double[] y, int degree) {
    this.degree = degree;

    int n = x.length;
    QRDecomposition qr = null;
    Matrix matrixX = null;

    while (true) {
      double[][] vandermonde = new double[n][this.degree + 1];
      for (int i = 0; i < n; i++) {
        for (int j = 0; j <= this.degree; j++) {
          vandermonde[i][j] = Math.pow(x[i], j);
        }
      }
      matrixX = new Matrix(vandermonde);

      qr = new QRDecomposition(matrixX);
      if (qr.isFullRank()) break;

      this.degree--;
    }

    Matrix matrixY = new Matrix(y, n);

    beta = qr.solve(matrixY);

    double sum = 0.0;
    for (int i = 0; i < n; i++) sum += y[i];
    double mean = sum / n;

    for (int i = 0; i < n; i++) {
      double dev = y[i] - mean;
      sst += dev * dev;
    }

    Matrix residuals = matrixX.times(beta).minus(matrixY);
    sse = residuals.norm2() * residuals.norm2();
  }

  public double beta(int j) {
    if (Math.abs(beta.get(j, 0)) < 1E-4) {
      return 0.0;
    }
    return beta.get(j, 0);
  }

  public int degree() {
    return degree;
  }

  public double R2() {
    if (sst == 0.0) return 1.0;
    return 1.0 - sse / sst;
  }

  public double predict(double x) {
    double y = 0.0;
    for (int j = degree; j >= 0; j--) y = beta(j) + (x * y);
    return y;
  }

  @Override
  public String toString() {
    StringBuilder s = new StringBuilder();
    int j = degree;

    while (j >= 0 && Math.abs(beta(j)) < 1E-5) j--;

    while (j >= 0) {
      if (j == 0) s.append(String.format("%.2f ", beta(j)));
      else if (j == 1) s.append(String.format("%.2f x + ", beta(j)));
      else s.append(String.format("%.2f x^%d + ", beta(j), j));
      j--;
    }
    s = s.append("  (R^2 = " + String.format("%.3f", R2()) + ")");
    return s.toString();
  }
}
