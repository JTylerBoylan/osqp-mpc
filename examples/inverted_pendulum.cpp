#include "osqp_mpc.hpp"

// Inverted Pendulum on a Cart MPC Example
// https://ctms.engin.umich.edu/CTMS/?example=InvertedPendulum&section=SystemModeling

#define WINDOW_SIZE 11
#define TIME_STEP 0.1

#define NUM_STATES 4
#define NUM_CONTROLS 1
#define NUM_CONSTRAINTS_X 0
#define NUM_CONSTRAINTS_U 1

#define M 0.5
#define m 0.2
#define b 0.1
#define l 0.3
#define I 0.006
#define g 9.81

#define Fmax 10.0

static inline OSQPVector getX0()
{
    OSQPVector x0(NUM_STATES);
    x0 << 0.05, 0.0, -0.05, 0.0; // modify initial state here
    return x0;
}

static inline OSQPVector getXref(const double t)
{
    return OSQPVector::Zero(NUM_STATES);
}

static inline OSQPMatrix getQ(const double t)
{
    OSQPMatrix Q(NUM_STATES, NUM_STATES);
    Q << 1.0, 0.0, 0.0, 0.0,
        0.0, 0.1, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.1;
    return Q;
}

static inline OSQPMatrix getR(const double t)
{
    OSQPMatrix R(NUM_CONTROLS, NUM_CONTROLS);
    R << 0.01;
    return R;
}

static inline OSQPMatrix getA(const double t)
{
    constexpr double D = I * (M + m) + M * m * l * l;
    constexpr double A22 = -b * (I + m * l * l) / D;
    constexpr double A23 = m * m * l * l * g / D;
    constexpr double A42 = -m * l * b / D;
    constexpr double A43 = (M + m) * m * g * l / D;

    OSQPMatrix A(NUM_STATES, NUM_STATES);
    A << 0.0, 1.0, 0.0, 0.0,
        0.0, A22, A23, 0.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, A42, A43, 0.0;
    A = A * TIME_STEP + OSQPMatrix::Identity(NUM_STATES, NUM_STATES);
    return A;
}

static inline OSQPMatrix getB(const double t)
{
    constexpr double D = I * (M + m) + M * m * l * l;
    constexpr double B21 = (I + m * l * l) / D;
    constexpr double B41 = m * l / D;

    OSQPMatrix B(NUM_STATES, NUM_CONTROLS);
    B << 0.0, B21, 0.0, B41;
    B = B * TIME_STEP;
    return B;
}

static inline OSQPMatrix getC(const double t)
{
    return OSQPMatrix(NUM_CONSTRAINTS_X, NUM_STATES);
}

static inline OSQPVector getLBx(const double t)
{
    return OSQPVector(NUM_CONSTRAINTS_X);
}

static inline OSQPVector getUBx(const double t)
{
    return OSQPVector(NUM_CONSTRAINTS_X);
}

static inline OSQPMatrix getD(const double t)
{
    OSQPMatrix D(NUM_CONSTRAINTS_U, NUM_CONTROLS);
    D << 1.0;
    return D;
}

static inline OSQPVector getLBu(const double t)
{
    return OSQPVector::Constant(NUM_CONSTRAINTS_U, -Fmax);
}

static inline OSQPVector getUBu(const double t)
{
    return OSQPVector::Constant(NUM_CONSTRAINTS_U, +Fmax);
}

int main()
{
    printf("Inverted Pendulum on a Cart MPC Example\n");

    // set OSQP settings to default
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = false;

    // initialize MPC problem
    MPCProblem mpc(WINDOW_SIZE);
    mpc.x0 = getX0();
    for (std::size_t k = 0; k < WINDOW_SIZE; k++)
    {
        const double t = k * TIME_STEP;
        mpc.xref[k] = getXref(t);
        mpc.Q[k] = getQ(t);
        mpc.C[k] = getC(t);
        mpc.lbx[k] = getLBx(t);
        mpc.ubx[k] = getUBx(t);
        if (k < WINDOW_SIZE - 1)
        {
            mpc.R[k] = getR(t);
            mpc.A[k] = getA(t);
            mpc.B[k] = getB(t);

            mpc.D[k] = getD(t);
            mpc.lbu[k] = getLBu(t);
            mpc.ubu[k] = getUBu(t);
        }
    }

    // simulate MPC
    double total_runtime = 0.0;
    const int Nsim = 100;
    for (int i = 0; i < Nsim; i++)
    {
        // print current state and error
        const double error = (mpc.x0.transpose() * mpc.Q[0] * mpc.x0).value();
        printf("Iteration %d: x = %.5f, dxdt = %.5f, theta = %.5f, dthetadt = %.5f, err = %.5f",
               i, mpc.x0(0), mpc.x0(1), mpc.x0(2), mpc.x0(3), error);

        // solve
        const MPCSolution mpc_sol = solveMPC(mpc, &settings);

        // check for errors
        if (mpc_sol.exit_flag != OSQP_SOLVED)
        {
            printf("\nSolver error: (%lld) %s\n", mpc_sol.exit_flag, mpc_sol.exit_message.c_str());
            return 1;
        }
        total_runtime += mpc_sol.run_time;

        // get control input
        const OSQPVector ustar = mpc_sol.ustar[0];
        printf(", F = %.5f\n", ustar(0));

        // simulate system with control input
        mpc.x0 = mpc.A[0] * mpc.x0 + mpc.B[0] * ustar;
    }
    printf("Average runtime: %.2f us\n", total_runtime * 1E6 / static_cast<double>(Nsim));

    return 0;
}