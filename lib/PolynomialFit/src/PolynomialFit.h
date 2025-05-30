#ifndef POLYNOMIALFIT_H
#define POLYNOMIALFIT_H

#include <ArduinoEigen.h>
#include <vector>

class PolynomialFit {
public:
    void fit(const std::vector<double>& x, const std::vector<double>& y, int degree);
    double evaluate(double t);
    void printCoefficients();
    std::pair<double, double> calculateAandB(double x1, double y1, double x2, double y2, double x_rel);


private:
    double coeffs[4];  // Supports cubic polynomial
    int currentDegree;
};

#endif
