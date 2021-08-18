#ifndef TELECARLA_RPC_BOOL_H
#define TELECARLA_RPC_BOOL_H

#include <rpc/msgpack.hpp>
#include <std_msgs/Bool.h>

namespace lmt
{
namespace data
{
class Bool
{
  public:
    Bool() = default;
    explicit Bool(const std_msgs::BoolConstPtr& boolMsg) noexcept;

    std_msgs::Bool toROSMessage() const noexcept;

    MSGPACK_DEFINE_MAP(value);

  private:
    bool value;
};
}  // namespace data
}  // namespace lmt

#endif  // TELECARLA_RPC_BOOL_H
