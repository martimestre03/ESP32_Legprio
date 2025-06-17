#ifndef POLYNOMIALFIT_H
#define POLYNOMIALFIT_H

#include <ArduinoEigen.h>
#include <vector>

class PolynomialFit {
public:
    // --- Fit & Evaluate ---
    void fit(const std::vector<double>& x, const std::vector<double>& y, int degree);
    double evaluate(double t);

    // --- Debug ---
    void printCoefficients();

    // --- Utility ---
    std::pair<double, double> calculateAandB(double x1, double y1, double x_rel);

private:
    double coeffs[4];  // Coefficients for up to a cubic polynomial
    int currentDegree; // Degree of the currently fitted polynomial
};

#endif
