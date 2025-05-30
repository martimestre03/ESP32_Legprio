#include "PolynomialFit.h"
#include <Arduino.h>

void PolynomialFit::fit(const std::vector<double>& x, const std::vector<double>& y, int degree) {
    Serial.println("\n🔢 Debug Matrix Construction (with filtering ±0.1 on x):");

    // Step 1: Filter out (x, y) pairs where |x| > 0.1
    std::vector<double> fx, fy;
    for (int i = 0; i < x.size(); ++i) {
        if (fabs(x[i]) <= 0.1) {
            fx.push_back(x[i]);
            fy.push_back(y[i]);
        } else {
            Serial.printf("❌ Removed point (x=%.6f, y=%.6f)\n", x[i], y[i]);
        }
    }

    Serial.printf("✅ Number of points after filtering: %d, Degree: %d\n", fx.size(), degree);

    if (fx.size() <= degree) {
        Serial.println("⚠️ Not enough points to fit the requested degree.");
        return;
    }

    // Step 2: Construct matrix A and vector B
    Eigen::MatrixXd A(fx.size(), degree + 1);
    Eigen::VectorXd B(fx.size());

    Serial.println("\nConstructing A matrix:");
    for (int i = 0; i < fx.size(); ++i) {
        for (int j = 0; j <= degree; ++j) {
            A(i, j) = pow(fx[i], j);
            Serial.printf("A(%d,%d) = %.6f ", i, j, A(i, j));
        }
        B(i) = fy[i];
        Serial.printf("| B(%d) = %.6f\n", i, B(i));
    }

    // Step 3: Solve the linear system
    Serial.println("\nSolving system...");
    Eigen::VectorXd result = A.colPivHouseholderQr().solve(B);

    Serial.println("Coefficients found:");
    for (int i = 0; i <= degree; ++i) {
        coeffs[i] = result(i);  // assumes coeffs is pre-sized
        Serial.printf("coeff[%d] = %.6f\n", i, coeffs[i]);
    }

    currentDegree = degree;
}


double PolynomialFit::evaluate(double t) {
    double result = 0.0;
    Serial.printf("\n📊 Evaluating polynomial at t = %.6f\n", t);
    for (int i = 0; i <= currentDegree; ++i) {
        double term = coeffs[i] * pow(t, i);
        result += term;
        Serial.printf("Term %d: %.6f * %.6f^%d = %.6f\n", i, coeffs[i], t, i, term);
        Serial.printf("Running sum: %.6f\n", result);
    }
    return result;
}

void PolynomialFit::printCoefficients() {
    Serial.println("Polynomial Coefficients:");
    for (int i = 0; i <= currentDegree; ++i) {
        Serial.print("x^");
        Serial.print(i);
        Serial.print(" : ");
        Serial.println(coeffs[i], 6);
    }
}

std::pair<double, double> PolynomialFit::calculateAandB(double x1, double y1, double x2, double y2, double x_rel) {
    Serial.println("Calculating A and B for cubic polynomial fit...");
    Serial.print("x1: "); Serial.print(x1, 6);
    Serial.print(" | y1: "); Serial.print(y1, 6);   
    Serial.print(" | x2: "); Serial.print(x2, 6);
    Serial.print(" | y2: "); Serial.print(y2, 6);   
    Serial.print(" | x_rel: "); Serial.println(x_rel, 6);
    
    double a = (y1 - y2) / ((pow(x1, 3) - 3 * pow(x_rel, 2) * x1) - (pow(x2, 3) - 3 * pow(x_rel, 2) * x2));
    double b = -3 * a * pow(x_rel, 2);
    return std::make_pair(a, b);
}
