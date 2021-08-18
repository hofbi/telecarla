#include "bool.h"

using namespace lmt::data;

Bool::Bool(const std_msgs::BoolConstPtr& boolMsg) noexcept : value(boolMsg->data) {}

std_msgs::Bool Bool::toROSMessage() const noexcept
{
    std_msgs::Bool boolMsg;
    boolMsg.data = value;
    return boolMsg;
}
