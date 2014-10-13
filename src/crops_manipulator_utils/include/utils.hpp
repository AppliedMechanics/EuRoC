#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#define msg_error(format,...) \
			ROS_ERROR("%s - l:%d: "format,__FUNCTION__,__LINE__,##__VA_ARGS__)

#define msg_info(format,...) \
			ROS_INFO("%s - l:%d: "format,__FUNCTION__,__LINE__,##__VA_ARGS__)

#define msg_warn(format,...) \
			ROS_WARN("%s - l:%d: "format,__FUNCTION__,__LINE__,##__VA_ARGS__)

#define msg_dbg(format,...) \
			ROS_DEBUG("%s - l:%d: "format,__FUNCTION__,__LINE__,##__VA_ARGS__)

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
#endif /* __UTILS_HPP__ */
