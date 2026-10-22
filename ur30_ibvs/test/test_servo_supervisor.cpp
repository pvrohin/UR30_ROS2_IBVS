#include <gtest/gtest.h>

#include "ur30_ibvs/servo_supervisor.hpp"

namespace
{
using ur30_ibvs::ServoSupervisor;
using ur30_ibvs::SupervisorVerdict;

constexpr double kImageTimeout = 2.0;
constexpr double kEdgeTimeout = 2.0;

// Feed frames at `hz` from t0 to t1 (exclusive), calling `per_frame` with each time.
template<typename F>
void frames(double t0, double t1, double hz, F per_frame)
{
  for (double t = t0; t < t1; t += 1.0 / hz) {
    per_frame(t);
  }
}
}  // namespace

TEST(ServoSupervisor, StaysOkWhileImagesArriveAndTheEdgeIsLocked)
{
  ServoSupervisor s(kImageTimeout, kEdgeTimeout);
  s.start(0.0);
  frames(0.0, 30.0, 30.0, [&](double t) {
      s.onImage(t);
      s.onEdgeLocked();
      ASSERT_EQ(s.check(t), SupervisorVerdict::OK);
    });
}

TEST(ServoSupervisor, ReportsALostCameraOnlyAfterTheImageTimeout)
{
  ServoSupervisor s(kImageTimeout, kEdgeTimeout);
  s.start(0.0);
  s.onImage(10.0);
  s.onEdgeLocked();
  EXPECT_EQ(s.check(11.9), SupervisorVerdict::OK);
  EXPECT_EQ(s.check(12.0), SupervisorVerdict::OK);  // exactly at the limit is still fine
  EXPECT_EQ(s.check(12.01), SupervisorVerdict::CAMERA_LOST);
  EXPECT_NEAR(s.secondsSinceImage(12.5), 2.5, 1e-12);
}

TEST(ServoSupervisor, ANewImageClearsTheCameraTimeout)
{
  ServoSupervisor s(kImageTimeout, kEdgeTimeout);
  s.start(0.0);
  s.onEdgeLocked();
  s.onImage(1.0);
  EXPECT_EQ(s.check(3.5), SupervisorVerdict::CAMERA_LOST);
  s.onImage(3.6);
  EXPECT_EQ(s.check(3.7), SupervisorVerdict::OK);
}

TEST(ServoSupervisor, ToleratesAnEdgeGapShorterThanTheTimeoutWhateverTheFrameCount)
{
  // The reason for a duration rather than a count: 1.5 s of missing edge is 45
  // frames at 30 Hz and only 8 at 5 Hz, and both must be tolerated.
  for (const double hz : {5.0, 30.0, 120.0}) {
    ServoSupervisor s(kImageTimeout, kEdgeTimeout);
    s.start(0.0);
    frames(0.0, 1.0, hz, [&](double t) {s.onImage(t); s.onEdgeLocked();});
    frames(1.0, 2.5, hz, [&](double t) {
        s.onImage(t);
        s.onEdgeMissing(t);
        ASSERT_EQ(s.check(t), SupervisorVerdict::OK) << hz << " Hz, t=" << t;
      });
    s.onImage(2.5);
    s.onEdgeLocked();  // found again
    EXPECT_EQ(s.check(2.5), SupervisorVerdict::OK) << hz << " Hz";
  }
}

TEST(ServoSupervisor, GivesUpOnceTheEdgeHasBeenMissingLongerThanTheTimeout)
{
  ServoSupervisor s(kImageTimeout, kEdgeTimeout);
  s.start(0.0);
  s.onEdgeLocked();
  bool gave_up = false;
  double when = 0.0;
  frames(1.0, 6.0, 30.0, [&](double t) {
      s.onImage(t);
      s.onEdgeMissing(t);
      if (!gave_up && s.check(t) == SupervisorVerdict::EDGE_LOST) {
        gave_up = true;
        when = t;
      }
    });
  ASSERT_TRUE(gave_up);
  EXPECT_NEAR(when, 1.0 + kEdgeTimeout, 0.05);  // 2 s after it was first missing
}

TEST(ServoSupervisor, RelockingRestartsTheEdgeClock)
{
  ServoSupervisor s(kImageTimeout, kEdgeTimeout);
  s.start(0.0);
  s.onImage(0.0);
  s.onEdgeLocked();
  s.onImage(1.0);
  s.onEdgeMissing(1.0);
  s.onImage(2.5);
  s.onEdgeLocked();  // recovered after 1.5 s
  s.onImage(4.0);
  s.onEdgeMissing(4.0);  // a new gap starts now, not at 1.0
  s.onImage(5.9);
  EXPECT_EQ(s.check(5.9), SupervisorVerdict::OK);
  s.onImage(6.1);
  EXPECT_EQ(s.check(6.1), SupervisorVerdict::EDGE_LOST);
}

TEST(ServoSupervisor, TheEdgeCountsAsMissingUntilItIsFirstLocked)
{
  // Entering SERVO without ever finding the edge must not wait forever.
  ServoSupervisor s(kImageTimeout, kEdgeTimeout);
  s.start(10.0);
  frames(10.0, 12.0, 30.0, [&](double t) {s.onImage(t); ASSERT_EQ(s.check(t), SupervisorVerdict::OK);});
  s.onImage(12.5);
  EXPECT_EQ(s.check(12.5), SupervisorVerdict::EDGE_LOST);
}

TEST(ServoSupervisor, ALostCameraTakesPrecedenceOverALostEdge)
{
  // With no images the edge state is unknown, so report the camera.
  ServoSupervisor s(kImageTimeout, kEdgeTimeout);
  s.start(0.0);
  s.onImage(1.0);
  s.onEdgeMissing(1.0);
  EXPECT_EQ(s.check(10.0), SupervisorVerdict::CAMERA_LOST);
}

TEST(ServoSupervisor, ReportsHowLongTheEdgeHasBeenMissing)
{
  ServoSupervisor s(kImageTimeout, kEdgeTimeout);
  s.start(0.0);
  s.onEdgeLocked();
  EXPECT_DOUBLE_EQ(s.secondsEdgeMissing(5.0), 0.0);
  s.onEdgeMissing(5.0);
  s.onEdgeMissing(5.4);  // repeated reports do not restart the clock
  EXPECT_NEAR(s.secondsEdgeMissing(6.0), 1.0, 1e-12);
}
