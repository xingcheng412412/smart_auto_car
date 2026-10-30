#ifndef RTK_GPS_DRIVER_CPP_GNSS_ORIGIN_STORAGE_H
#define RTK_GPS_DRIVER_CPP_GNSS_ORIGIN_STORAGE_H

#include <string>

namespace rtk_gps_driver_cpp {

bool update_gnss_origin_yaml(const std::string& config_path,
                             double latitude,
                             double longitude,
                             double altitude,
                             std::string* error);

}  // namespace rtk_gps_driver_cpp

#endif  // RTK_GPS_DRIVER_CPP_GNSS_ORIGIN_STORAGE_H
