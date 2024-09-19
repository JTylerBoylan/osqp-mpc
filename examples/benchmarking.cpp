#include "osqp_mpc.hpp"

#include <chrono>

#define START_WINDOW_SIZE 5
#define END_WINDOW_SIZE 250
#define WINDOW_SIZE_STEP 5

#define NUM_STATES 12
#define NUM_CONTROLS 12
#define NUM_CONSTRAINTS_X 6
#define NUM_CONSTRAINTS_U 6

MPCProblem generateRandomMPCProblem(const std::size_t N,
                                    const std::size_t nx, const std::size_t nu,
                                    const std::size_t nc, const std::size_t nd)
{
    MPCProblem mpc(N);
    mpc.x0 = OSQPVector::Random(nx);
    for (std::size_t k = 0; k < N; k++)
    {
        mpc.xref[k] = OSQPVector::Random(nx);
        mpc.Q[k] = OSQPVector::Random(nx).cwiseAbs().asDiagonal();
        mpc.C[k] = OSQPMatrix::Random(nc, nx);
        mpc.lbx[k] = -OSQPVector::Random(nc).cwiseAbs();
        mpc.ubx[k] = OSQPVector::Random(nc).cwiseAbs();
        if (k < N - 1)
        {
            mpc.R[k] = OSQPVector::Random(nu).cwiseAbs().asDiagonal();
            mpc.A[k] = OSQPMatrix::Random(nx, nx);
            mpc.B[k] = OSQPMatrix::Random(nx, nu);

            mpc.D[k] = OSQPMatrix::Random(nd, nu);
            mpc.lbu[k] = -OSQPVector::Random(nd).cwiseAbs();
            mpc.ubu[k] = OSQPVector::Random(nd).cwiseAbs();
        }
    }
    return mpc;
}

using namespace std::chrono;

int main()
{
    printf("MPC Benchmarking\n");

    // set OSQP settings to default
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = false;

    for (std::size_t N = START_WINDOW_SIZE; N <= END_WINDOW_SIZE; N += WINDOW_SIZE_STEP)
    {
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        MPCProblem mpc = generateRandomMPCProblem(N, NUM_STATES, NUM_CONTROLS, NUM_CONSTRAINTS_X, NUM_CONSTRAINTS_U);
        high_resolution_clock::time_point t2 = high_resolution_clock::now();

        high_resolution_clock::time_point t3 = high_resolution_clock::now();
        QPProblem qp = getQPProblem(mpc);
        high_resolution_clock::time_point t4 = high_resolution_clock::now();

        high_resolution_clock::time_point t5 = high_resolution_clock::now();
        QPSolution solution = solveOSQP(qp, &settings);
        high_resolution_clock::time_point t6 = high_resolution_clock::now();

        duration<double> time_span1 = duration_cast<duration<double>>(t2 - t1);
        duration<double> time_span2 = duration_cast<duration<double>>(t4 - t3);
        duration<double> time_span3 = duration_cast<duration<double>>(t6 - t5);

        duration<double> total_time_span = duration_cast<duration<double>>(t6 - t1);
        double estimated_frequency = 1.0 / total_time_span.count(); // Hz

        printf("[N = %lu] Generation Time: %.2f ms, Conversion Time: %.2f ms, Solve Time: %.2f ms,"
               " Estimated run frequency : % .2f Hz\n",
               N, time_span1.count() * 1e3, time_span2.count() * 1e3, time_span3.count() * 1e3, estimated_frequency);
    }

    printf("Done\n");
    return 0;
}