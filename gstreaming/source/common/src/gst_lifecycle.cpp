#include "gst_lifecycle.h"

#include <gst/gst.h>

using namespace lmt::common;

GstLifecycle::GstLifecycle(int argc, char** argv)
{
    gst_init(&argc, &argv);
}

GstLifecycle::~GstLifecycle()
{
    gst_deinit();
}
