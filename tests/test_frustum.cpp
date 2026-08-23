#include <gtest/gtest.h>
#include "../mathc.h"

using Math::Box;
using Math::Frustum;
using Math::Mat4;
using Math::Vec3;

// Camera at (0,0,5) looking toward -Z (origin), matching Mat4::LookAt/Perspective
// which were already verified against glm — this only tests plane extraction
// and the containment tests on top of an already-correct matrix.
static Frustum MakeTestFrustum() {
    Mat4 view = Mat4::LookAt(Vec3(0, 0, 5), Vec3(0, 0, 0), Vec3(0, 1, 0));
    Mat4 proj = Mat4::Perspective(Math::PI / 3.0f, 1.0f, 0.1f, 100.0f); // 60 degree FOV
    return Frustum::FromViewProjection(proj * view);
}

TEST(Frustum, ContainsPointInFrontOfCamera) {
    Frustum f = MakeTestFrustum();
    EXPECT_TRUE(f.ContainsPoint(Vec3(0.0f, 0.0f, 0.0f))); // straight ahead, mid-range
}

TEST(Frustum, RejectsPointBehindCamera) {
    Frustum f = MakeTestFrustum();
    EXPECT_FALSE(f.ContainsPoint(Vec3(0.0f, 0.0f, 10.0f))); // behind the eye at z=5
}

TEST(Frustum, RejectsPointOutsideHorizontalFov) {
    Frustum f = MakeTestFrustum();
    EXPECT_FALSE(f.ContainsPoint(Vec3(100.0f, 0.0f, 0.0f))); // far outside a 60deg FOV
}

TEST(Frustum, RejectsPointCloserThanNear) {
    Frustum f = MakeTestFrustum();
    EXPECT_FALSE(f.ContainsPoint(Vec3(0.0f, 0.0f, 4.99f))); // within 0.1 of the eye at z=5
}

TEST(Frustum, RejectsPointFartherThanFar) {
    Frustum f = MakeTestFrustum();
    EXPECT_FALSE(f.ContainsPoint(Vec3(0.0f, 0.0f, -200.0f))); // beyond far=100
}

TEST(Frustum, AcceptsPointJustInsideNearPlane) {
    Frustum f = MakeTestFrustum();
    EXPECT_TRUE(f.ContainsPoint(Vec3(0.0f, 0.0f, 4.85f))); // eye - 0.15, inside near=0.1
}

TEST(Frustum, IntersectsSphereAtOrigin) {
    Frustum f = MakeTestFrustum();
    EXPECT_TRUE(f.IntersectsSphere(Vec3(0.0f, 0.0f, 0.0f), 1.0f));
    EXPECT_FALSE(f.IntersectsSphere(Vec3(0.0f, 0.0f, 10.0f), 1.0f)); // behind camera, out of reach
}

TEST(Frustum, IntersectsBoxAroundOrigin) {
    Frustum f = MakeTestFrustum();
    Box inside(Vec3(-0.5f, -0.5f, -0.5f), Vec3(0.5f, 0.5f, 0.5f));
    Box farAway(Vec3(1000.0f, 1000.0f, 1000.0f), Vec3(1001.0f, 1001.0f, 1001.0f));

    EXPECT_TRUE(f.IntersectsBox(inside));
    EXPECT_FALSE(f.IntersectsBox(farAway));
}
