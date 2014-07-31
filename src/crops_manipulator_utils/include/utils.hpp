#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#define msg_error(format,...) \
			ROS_ERROR("%s - l:%d: "format,__FUNCTION__,__LINE__,##__VA_ARGS__)

#define msg_warn(format,...) \
			ROS_WARN("%s - l:%d: "format,__FUNCTION__,__LINE__,##__VA_ARGS__)

#define msg_dbg(format,...) \
			ROS_DEBUG("%s - l:%d: "format,__FUNCTION__,__LINE__,##__VA_ARGS__)

#endif /* __UTILS_HPP__ */
