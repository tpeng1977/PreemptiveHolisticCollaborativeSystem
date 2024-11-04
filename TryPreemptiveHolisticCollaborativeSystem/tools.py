import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize


def scale_series(t, x_original, a, b, b_alter):
    """
    Transforms the given function x(t) to have a new value at t = T while maintaining
    continuity and similarity of first and second derivatives at the boundaries.

    Parameters:
    - t: A 1D numpy array representing the time values at which the function is evaluated.
         These values are evenly spaced in the interval (0, T).
    - x_original: A 1D numpy array representing the original values of the function x(t).
                  These values are generated based on a linear interpolation from a to b
                  and an added sine component.
    - a: The function value at t[0].
    - b: The original function value at t[-1].
    - b_alter: The new function value at t = T that we want to achieve after the transformation.

    Returns:
    - A 1D numpy array representing the transformed values of the function x(t).
    """

    def scale_function(x_original, a, b, b_alter):
        """
        Scales the original function values to fit the new end value b_alter.

        Parameters:
        - x_original: A 1D numpy array representing the original values of the function x(t).
        - a: The function value at t = 0.
        - b: The original function value at t = T.
        - b_alter: The new function value at t = T.

        Returns:
        - A 1D numpy array representing the scaled values of the function x(t).
        """
        scale_factor = (b_alter - a) / (b - a)
        return a + (x_original - a) * scale_factor

    # Scale the values
    x_scaled = scale_function(x_original, a, b, b_alter)

    # Create a cubic spline with the scaled function values
    cs = CubicSpline(t, x_scaled)

    # Calculate the first and second derivatives at the boundaries
    d1_0 = cs(0, 1)
    d1_T = cs(len(t), 1)
    d2_0 = cs(0, 2)
    d2_T = cs(len(t), 2)

    # Modify the function value at t=T to b_alter
    # x_scaled[-1] = b_alter

    # Create a cubic spline interpolation with specified first derivative boundary conditions
    cs = CubicSpline(t, x_scaled, bc_type=((1, d1_0), (1, d1_T)))

    # Function to minimize the difference in second derivatives
    def objective(params):
        """
        Defines an objective function to minimize the difference in second derivatives
        between the original and transformed functions.

        Parameters:
        - params: A 1D numpy array representing the coefficients to be adjusted.

        Returns:
        - A scalar value representing the sum of squared differences in second derivatives
          at the boundaries.
        """
        cs.c[:, 0] = params[:4]  # Adjust the coefficients
        d2_0_new = cs(0, 2)
        d2_T_new = cs(len(t), 2)
        return (d2_0_new - d2_0) ** 2 + (d2_T_new - d2_T) ** 2

    # Initial guess for the coefficients
    initial_guess = cs.c[:, 0]

    # Optimize the coefficients to minimize the difference in second derivatives
    result = minimize(objective, initial_guess, method='L-BFGS-B')

    # Update the spline with the optimized coefficients
    cs.c[:, 0] = result.x[:4]
    t_res = cs(t)
    t_res = t_res - min(t_res) + a
    t_b = max(t_res)
    t_res = scale_function(t_res, a, t_b, b_alter)
    return t_res


def test_scale_series():
    # Given parameters
    T = 100  # Example interval end. Represents the maximum time value for the function.
    a = 1
    b = 1008  # Original function value at t=T. The value of the function before transformation at the end of the interval.
    b_alter = 2800  # New function value at t=T. The desired value of the function after transformation at the end of the interval.
    n = 100000  # Number of discrete points. Determines the granularity of the function representation.

    # Generate t values
    t = np.linspace(0, T, n) + 10.0

    # Generate the original x(t) with an added sine component
    x_original = np.linspace(a, b, n) + 10 * np.sin(t / 20 * np.pi) + 100.0
    low_end = min(x_original)
    high_end = max(x_original)

    output = scale_series(t, x_original, low_end, high_end, b_alter)

    # Plot the original and modified functions
    plt.plot(t, x_original, label='Original x(t)')
    plt.plot(t, output, label='Modified x(t)')
    plt.scatter([min(t), max(t)], [low_end, b_alter], color='red')  # Highlight the endpoints
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('x(t)')
    plt.title('Function Transformation with Scaling and Optimized Smoothness')
    plt.show()


if __name__ == "__main__":
    test_scale_series()
