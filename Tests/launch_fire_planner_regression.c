#include "launch_fire_planner.h"

#include <assert.h>
#include <math.h>

static int float_equal(float left, float right)
{
    return fabsf(left - right) < 0.001f;
}

int main(void)
{
    LaunchFirePlanner_t planner = {0};

    /* 单发目标已经下发后，差速门控瞬时关闭只能暂停后续步进，
     * 不能清除当前 40 度目标或已下发计数。 */
    LaunchFirePlanner_Reset(&planner, 0.0f, 0U);
    LaunchFirePlanner_Update(&planner, 0.0f, 66U, 66U, 1U, 1U);
    assert(float_equal(planner.start_deg, 0.0f));
    assert(float_equal(planner.target_deg, 40.0f));
    assert(planner.issued_count == 1U);

    LaunchFirePlanner_Update(&planner, 20.0f, 67U, 66U, 1U, 0U);
    assert(float_equal(planner.start_deg, 0.0f));
    assert(float_equal(planner.target_deg, 40.0f));
    assert(planner.issued_count == 1U);

    LaunchFirePlanner_Update(&planner, 20.0f, 200U, 66U, 1U, 1U);
    assert(float_equal(planner.target_deg, 40.0f));
    assert(planner.issued_count == 1U);

    /* 首发前门控关闭时保持零目标，并冻结发射间隔计时。 */
    LaunchFirePlanner_Reset(&planner, 0.0f, 0U);
    LaunchFirePlanner_Update(&planner, 0.0f, 100U, 66U, 1U, 0U);
    LaunchFirePlanner_Update(&planner, 0.0f, 150U, 66U, 1U, 1U);
    assert(float_equal(planner.target_deg, 0.0f));
    assert(planner.issued_count == 0U);
    LaunchFirePlanner_Update(&planner, 0.0f, 166U, 66U, 1U, 1U);
    assert(float_equal(planner.target_deg, 40.0f));
    assert(planner.issued_count == 1U);

    return 0;
}
