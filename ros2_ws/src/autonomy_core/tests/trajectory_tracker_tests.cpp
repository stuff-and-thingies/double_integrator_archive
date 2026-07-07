#include <cassert>

#include "autonomy_control/trajectory_tracker.hpp"
#include "autonomy_core/pose3.hpp"
#include "autonomy_core/time.hpp"
#include "autonomy_core/trajectory.hpp"
#include "autonomy_core/twist.hpp"
#include "autonomy_core/vector3.hpp"
#include "autonomy_sim/double_integrator.hpp"
#include "autonomy_sim/fake_drone_state.hpp"
#include "test_utils.hpp"

using autonomy_control::TrajectoryTracker;
using autonomy_control::TrajectoryTrackerGains;
using autonomy_core::Pose3;
using autonomy_core::Timestamp;
using autonomy_core::TrajectoryPoint;
using autonomy_core::Twist;
using autonomy_core::Vector3;
using autonomy_sim::AccelerationCommand;
using autonomy_sim::DoubleIntegrator;
using autonomy_sim::DoubleIntegratorLimits;
using autonomy_sim::FakeDroneState;

bool accel_commands_are_equal(const AccelerationCommand& a,
                              const AccelerationCommand& b)
{
    const bool x_check = nearly_equal(a.acceleration.x, b.acceleration.x);
    const bool y_check = nearly_equal(a.acceleration.y, b.acceleration.y);
    const bool z_check = nearly_equal(a.acceleration.z, b.acceleration.z);

    return x_check && y_check && z_check;
}

void test_zero_error()
{
    TrajectoryTracker trajectory_tracker;

    const FakeDroneState state{Vector3{}, Vector3{}};
    const TrajectoryPoint reference{Timestamp{0.0}, Pose3{Vector3{}, Vector3{}},
                                    Twist{Vector3{}, Vector3{}}};

    const AccelerationCommand accel_command =
        trajectory_tracker.compute_command(state, reference);
    const AccelerationCommand true_command{Vector3{0.0, 0.0, 0.0}};

    assert(accel_commands_are_equal(accel_command, true_command));
}

void test_positive_x_position_error()
{
    TrajectoryTracker trajectory_tracker{TrajectoryTrackerGains{1.0, 1.0, 5.0}};

    const FakeDroneState state{Vector3{}, Vector3{}};
    const TrajectoryPoint reference{Timestamp{0.0},
                                    Pose3{Vector3{1.0, 0.0, 0.0}, Vector3{}},
                                    Twist{Vector3{}, Vector3{}}};

    const AccelerationCommand accel_command =
        trajectory_tracker.compute_command(state, reference);
    const AccelerationCommand true_command{Vector3{1.0, 0.0, 0.0}};

    assert(accel_commands_are_equal(accel_command, true_command));
}

void test_positive_y_position_error()
{
    TrajectoryTracker trajectory_tracker{TrajectoryTrackerGains{1.0, 1.0, 5.0}};

    const FakeDroneState state{Vector3{}, Vector3{}};
    const TrajectoryPoint reference{Timestamp{0.0},
                                    Pose3{Vector3{0.0, 1.0, 0.0}, Vector3{}},
                                    Twist{Vector3{}, Vector3{}}};

    const AccelerationCommand accel_command =
        trajectory_tracker.compute_command(state, reference);
    const AccelerationCommand true_command{Vector3{0.0, 1.0, 0.0}};

    assert(accel_commands_are_equal(accel_command, true_command));
}

void test_positive_velocity_reference()
{
    TrajectoryTracker trajectory_tracker{TrajectoryTrackerGains{1.0, 1.0, 5.0}};

    const FakeDroneState state{Vector3{}, Vector3{}};
    const TrajectoryPoint reference{
        Timestamp{0.0}, Pose3{Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}},
        Twist{Vector3{1.0, 1.0, 0.0}, Vector3{}}};

    const AccelerationCommand accel_command =
        trajectory_tracker.compute_command(state, reference);
    const AccelerationCommand true_command{Vector3{1.0, 1.0, 0.0}};

    assert(accel_commands_are_equal(accel_command, true_command));
}

void test_large_error()
{
    TrajectoryTracker trajectory_tracker{TrajectoryTrackerGains{1.0, 1.0, 5.0}};

    const FakeDroneState state{Vector3{}, Vector3{}};
    const TrajectoryPoint reference{
        Timestamp{0.0}, Pose3{Vector3{100.0, 100.0, 0.0}, Vector3{}},
        Twist{Vector3{}, Vector3{}}};

    const AccelerationCommand accel_command =
        trajectory_tracker.compute_command(state, reference);

    assert(nearly_equal(accel_command.acceleration.norm(), 5.0));
}

void test_3d_error()
{
    TrajectoryTracker trajectory_tracker{TrajectoryTrackerGains{1.0, 1.0, 5.0}};

    const FakeDroneState state{Vector3{}, Vector3{}};
    const TrajectoryPoint reference{
        Timestamp{0.0}, Pose3{Vector3{1.0, 1.0, 1.0}, Vector3{0.0, 0.0, 0.0}},
        Twist{Vector3{}, Vector3{}}};

    const AccelerationCommand accel_command =
        trajectory_tracker.compute_command(state, reference);
    const AccelerationCommand true_command{Vector3{1.0, 1.0, 1.0}};

    assert(accel_commands_are_equal(accel_command, true_command));
}

void test_gain_weight()
{
    TrajectoryTracker trajectory_tracker{TrajectoryTrackerGains{2.0, 3.0, 5.0}};

    const FakeDroneState state{Vector3{}, Vector3{}};
    const TrajectoryPoint reference{
        Timestamp{0.0}, Pose3{Vector3{1.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}},
        Twist{Vector3{0.0, 1.0, 0.0}, Vector3{}}};

    const AccelerationCommand accel_command =
        trajectory_tracker.compute_command(state, reference);
    const AccelerationCommand true_command{Vector3{2.0, 3.0, 0.0}};

    assert(accel_commands_are_equal(accel_command, true_command));
}

void test_closed_loop()
{
    const double max_accel_mps2 = 10.0;
    const double max_vel_mps = 10.0;
    DoubleIntegrator model{DoubleIntegratorLimits{max_accel_mps2, max_vel_mps}};

    TrajectoryTracker trajectory_tracker{};
    const TrajectoryPoint reference{
        Timestamp{2.0}, Pose3{Vector3{1.0, 1.0, 0.0}, Vector3{0.0, 0.0, 0.0}},
        Twist{Vector3{1.0, 1.0, 0.0}, Vector3{}}};

    FakeDroneState state{};
    AccelerationCommand command{};

    double t_end = 2.0;
    double dt = 0.001;

    const int num_steps = static_cast<int>(t_end / dt);

    const Vector3 initial_pos_error = reference.pose.position - state.position;
    const Vector3 initial_vel_error = reference.twist.linear - state.velocity;

    for (int curr_step = 0; curr_step < num_steps; ++curr_step)
    {
        command = trajectory_tracker.compute_command(state, reference);
        model.update(state, command, dt);
    }

    const Vector3 final_pos_error = reference.pose.position - state.position;
    const Vector3 final_vel_error = reference.twist.linear - state.velocity;

    assert(final_pos_error.norm() < initial_pos_error.norm());
    assert(final_vel_error.norm() < initial_vel_error.norm());
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    test_zero_error();
    test_positive_x_position_error();
    test_positive_y_position_error();
    test_positive_velocity_reference();
    test_large_error();
    test_3d_error();
    test_gain_weight();
    test_closed_loop();

    return 0;
}
