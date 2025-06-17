#include "PolynomialFit.h"
#include <Arduino.h>

/**
 * @brief Fits a polynomial of given degree to the input (x, y) data.
 * 
 * Filters out points where |x| > 0.1, constructs the design matrix and solves
 * for the polynomial coefficients using QR decomposition.
 * 
 * @param x Input x values.
 * @param y Input y values.
 * @param degree Degree of the polynomial to fit.
 */
void PolynomialFit::fit(const std::vector<double>& x, const std::vector<double>& y, int degree) {
    // Serial.println("\n🔢 Debug Matrix Construction (with filtering ±0.1 on x):");

    // Step 1: Filter out (x, y) pairs where |x| > 0.1
    std::vector<double> fx, fy;
    for (int i = 0; i < x.size(); ++i) {
        if (fabs(x[i]) <= 0.1) {
            fx.push_back(x[i]);
            fy.push_back(y[i]);
        } else {
            // Serial.printf("❌ Removed point (x=%.6f, y=%.6f)\n", x[i], y[i]);
        }
    }

    // Serial.printf("✅ Number of points after filtering: %d, Degree: %d\n", fx.size(), degree);

    if (fx.size() <= degree) {
        // Serial.println("⚠️ Not enough points to fit the requested degree.");
        return;
    }

    // Step 2: Construct matrix A and vector B
    Eigen::MatrixXd A(fx.size(), degree + 1);
    Eigen::VectorXd B(fx.size());

    Serial.println("\nConstructing A matrix:");
    for (int i = 0; i < fx.size(); ++i) {
        for (int j = 0; j <= degree; ++j) {
            A(i, j) = pow(fx[i], j);
            // Serial.printf("A(%d,%d) = %.6f ", i, j, A(i, j));
        }
        B(i) = fy[i];
        // Serial.printf("| B(%d) = %.6f\n", i, B(i));
    }

    // Step 3: Solve the linear system
    // Serial.println("\nSolving system...");
    Eigen::VectorXd result = A.colPivHouseholderQr().solve(B);

    // Serial.println("Coefficients found:");
    for (int i = 0; i <= degree; ++i) {
        coeffs[i] = result(i);  // assumes coeffs is pre-sized
        // Serial.printf("coeff[%d] = %.6f\n", i, coeffs[i]);
    }

    currentDegree = degree;
}

/**
 * @brief Evaluates the fitted polynomial at a given value t.
 * 
 * @param t The input value at which to evaluate the polynomial.
 * @return Result of the polynomial evaluated at t.
 */
double PolynomialFit::evaluate(double t) {
    double result = 0.0;
    // Serial.printf("\n📊 Evaluating polynomial at t = %.6f\n", t);
    for (int i = 0; i <= currentDegree; ++i) {
        double term = coeffs[i] * pow(t, i);
        result += term;
        // Serial.printf("Term %d: %.6f * %.6f^%d = %.6f\n", i, coeffs[i], t, i, term);
        // Serial.printf("Running sum: %.6f\n", result);
    }
    return result;
}

/**
 * @brief Prints the coefficients of the currently fitted polynomial.
 */
void PolynomialFit::printCoefficients() {
    // Serial.println("Polynomial Coefficients:");
    for (int i = 0; i <= currentDegree; ++i) {
        // Serial.print("x^");
        // Serial.print(i);
        // Serial.print(" : ");
        // Serial.println(coeffs[i], 6);
    }
}

/**
 * @brief Calculates cubic parameters a and b for a constrained polynomial.
 * 
 * Used to define a polynomial such that it passes through a known point and 
 * has zero derivative at a symmetrical point.
 * 
 * @param x1 Position where value is known.
 * @param y1 Value at position x1.
 * @param x_rel Symmetry point where slope is zero.
 * @return A pair (a, b) for use in custom polynomial definition.
 */
std::pair<double, double> PolynomialFit::calculateAandB(double x1, double y1, double x_rel) {
    // Serial.println("Calculating A and B for cubic polynomial fit...");
    // Serial.print("x1: "); Serial.print(x1, 6);
    // Serial.print(" | y1: "); Serial.print(y1, 6);   
    // Serial.print(" | x_rel: "); Serial.println(x_rel, 6);
    
    double a = (y1) / ((pow(x1, 3) - 3 * pow(x_rel, 2) * x1) );
    double b = -3 * a * pow(x_rel, 2);
    return std::make_pair(a, b);
}
