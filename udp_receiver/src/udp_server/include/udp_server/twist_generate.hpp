#include <geometry_msgs/Twist.h>

class TwistGenerate {
  public:
    TwistGenerate(double max_linear_x, double max_angular_z, const char* buffer);
    ~TwistGenerate() = default;

#pragma region Deleted
    TwistGenerate(const TwistGenerate&) = delete;
    TwistGenerate& operator=(const TwistGenerate&) = delete;
    TwistGenerate(TwistGenerate&&) = delete;
    TwistGenerate& operator=(TwistGenerate&&) = delete;
#pragma endregion

    inline bool error() const noexcept { return error_; }
    inline geometry_msgs::Twist twist() const noexcept { return twist_; }

  private:
    bool error_{false};
    geometry_msgs::Twist twist_{};
};