## Options

Example .yaml configuration files are included in `config_examples` folder. Consult the u-blox documentation for your device for the recommended settings.

The `ublox_gps` node supports the following parameters for all products and firmware versions:
* `device`: Path to the device port. Defaults to `/dev/ttyACM0`.
* `raw_data`: Whether the device is a raw data product. Defaults to false. Firmware <= 7.03 only.
* `load`: Parameters for loading the configuration to non-volatile memory. See `ublox_msgs/CfgCFG.msg`
    * `load/mask`: uint32_t. Mask of the configurations to load.
    * `load/device`: uint32_t. Mask which selects the devices for the load command.
* `save`: Parameters for saving the configuration to non-volatile memory. See `ublox_msgs/CfgCFG.msg`
    * `save/mask`: uint32_t. Mask of the configurations to save.
    * `save/device`: uint32_t. Mask which selects the devices for the save command.
* `uart1/baudrate`: Bit rate of the serial communication. Defaults to 9600.
* `uart1/in`: UART1 in communication protocol. Defaults to UBX, NMEA & RTCM. See `CfgPRT` message for possible values.
* `uart1/out`: UART1 out communication protocol. Defaults to UBX, NMEA & RTCM. See `CfgPRT` message for possible values.
* `frame_id`: ROS name prepended to frames produced by the node. Defaults to `gps`.
* `rate`: Rate in Hz of measurements. Defaults to 4.
* `nav_rate`: How often navigation solutions are published in number of measurement cycles. Defaults to 1.
* `enable_ppp`: Enable precise-point-positioning system. Defaults to false.
* `gnss/sbas`: Enable satellite-based augmentation system. Defaults to false.
* `sbas/max`: Maximum number of SBAS channels. Defaults to 0.
* `sbas/usage`: See `CfgSBAS` message for details. Defaults to 0.
* `dynamic_model`: Possible values below. Defaults to `portable`. See u-blox documentation for further description.
    * `portable`
    * `stationary`
    * `pedestrian`
    * `automotive`
    * `sea`
    * `airborne1`: Airborne, max acceleration = 1G
    * `airborne2`: Airborne, max acceleration = 2G
    * `airborne4`: Airborne, max acceleration = 4G
    * `wristwatch`
* `fix_mode`: Type of fixes supported: `2d`, `3d` or `both`.
* `dr_limit`: Max time in seconds to use dead reckoning after signal is lost. Defaults to 0.
* `dat`: Configuring the datum type (optional). See the CfgDAT message.
    * `dat/set`: If true, the node will the datum based on the parameters below (required if true). Defaults to false. 
    * `dat/majA`: Semi-major Axis [m]
    * `dat/flat`: 1.0 / Flattening
    * `dat/shift`: [X-axis, Y-axis, Z-axis] shift [m]
    * `dat/rot`: [X, Y, Z] rotation [s]
    * `dat/scale`: scale change [ppm]

### For firmware version 6:
* `nmea/set`: If true, the NMEA will be configured with the parameters below.
* `nmea/version`: NMEA version. Must be set if `nmea/set` is true.
* `nmea/num_sv`: Maximum Number of SVs to report per TalkerId. Must be set if `nmea/set` is true.
* `nmea/compat`: Enable compatibility mode. Must be set if `nmea/set` is true.
* `nmea/consider`: Enable considering mode. Must be set if `nmea/set` is true.
* `nmea/filter`: Namespace for filter flags.
    * `nmea/filter/pos`: Disable position filtering. Defaults to false.
    * `nmea/filter/msk_pos`: Disable masked position filtering. Defaults to false.
    * `nmea/filter/time`: Disable time filtering. Defaults to false.
    * `nmea/filter/date`: Disable date filtering. Defaults to false.
    * `nmea/filter/sbas`: Enable SBAS filtering. Defaults to false.
    * `nmea/filter/track`: Disable track filtering. Defaults to false.

### For devices with firmware >= 7:
* `gnss` parameters:
    * `gnss/gps`: Enable GPS receiver. Defaults to true.
    * `gnss/glonass`: Enable GLONASS receiver. Defaults to false.
    * `gnss/beidou`: Enable BeiDou receiver. Defaults to false.
    * `gnss/qzss`: Enable QZSS receiver. Defaults to false.
    * `gnss/qzss_sig_cfg`: QZSS signal configuration. Defaults to L1CA. See `CfgGNSS` message for constants.
* `nmea` parameters:
    * `nmea/set`: If true, the NMEA will be configured.
    * `nmea/version`: NMEA version. Must be set if `nmea/set` is true.
    * `nmea/num_sv`: Maximum Number of SVs to report per TalkerId. Must be set if `nmea/set` is true.
    * `nmea/sv_numbering`: Configures the display of satellites that do not have an NMEA-defined value. Must be set if `nmea/set` is true.
    * `nmea/compat`: Enable compatibility mode. Must be set if `nmea/set` is true.
    * `nmea/consider`: Enable considering mode. Must be set if `nmea/set` is true.
    * `nmea/limit82`: Enable strict limit to 82 characters maximum. Defaults to false.
    * `nmea/high_prec`: Enable high precision mode. Defaults to false.
    * `nmea/filter`: Namespace for filter flags.
        * `nmea/filter/pos`: Enable position output for failed or invalid fixes. Defaults to false.
        * `nmea/filter/msk_pos`: Enable position output for invalid fixes. Defaults to false.
        * `nmea/filter/time`: Enable time output for invalid times. Defaults to false.
        * `nmea/filter/date`: Enable date output for invalid dates. Defaults to false.
        * `nmea/filter/gps_only`: Restrict output to GPS satellites only. Defaults to false.
        * `nmea/filter/track`: Enable COG output even if COG is frozen. Defaults to false.
    * `nmea/gnssToFilt`: Filters out satellites based on their GNSS.
        * `nmea/gnssToFilt/gps`: Disable reporting of GPS satellites. Defaults to false.
        * `nmea/gnssToFilt/sbas`: Disable reporting of SBAS satellites. Defaults to false.
        * `nmea/gnssToFilt/qzss`: Disable reporting of QZSS satellites. Defaults to false.
        * `nmea/gnssToFilt/glonass`: Disable reporting of GLONASS satellites. Defaults to false.
        * `nmea/gnssToFilt/beidou`: Disable reporting of BeiDou satellites. Defaults to false.
    * `nmea/main_talker_id`: This field enables the main Talker ID to be overridden. Defaults to 0.
    * `nmea/gsv_talker_id`:  This field enables the GSV Talker ID to be overridden. Defaults to [0, 0].

### For devices with firmware >= 8:
* `save_on_shutdown`: If true, the node will send a `UBX-UPD-SOS` command to save the BBR to flash memory on shutdown. Defaults to false. 
* `clear_bbr`: If true, the node will send a `UBX-UPD-SOS` command to clear the flash memory during configuration. Defaults to false.
* Additional `gnss` params
  * `gnss/galileo`: Enable Galileo receiver. Defaults to false.
  * `gnss/imes`: Enable IMES receiver. Defaults to false.
* `nmea/bds_talker_id`: (See other NMEA configuration parameters above) Sets the two characters that should be used for the BeiDou Talker ID.

### For UDR/ADR devices:
* `use_adr`: Enable ADR/UDR. Defaults to true.
* `nav_rate` should be set to 1 Hz.

### For HPG Reference devices:
* `tmode3`: Time Mode. Required. See CfgTMODE3 for constants.
* `arp/lla_flag`: True if the Fixed position is in Lat, Lon, Alt coordinates. False if ECEF. Required if `tmode3` is set to fixed. 
* `arp/position`: Antenna Reference Point position in [m] or [deg]. Required if `tmode3` is set to fixed. 
* `arp/position_hp`: Antenna Reference Point High Precision position in [0.1 mm] or [deg * 1e-9]. Required if tmode3 is set to fixed. 
* `arp/acc`: Fixed position accuracy in [m]. Required if `tmode3` is set to fixed. 
* `sv_in/reset`: Whether or not to reset the survey in upon initialization. If false, it will only reset if the TMODE is disabled. Defaults to true.
* `sv_in/min_dur`: The minimum Survey-In Duration time in seconds. Required tmode3 is set to survey in.
* `sv_in/acc_lim`: The minimum accuracy level of the survey in position in meters. Required `tmode3` is set to survey in.

### For HPG Rover devices:
* `dgnss_mode`: The Differential GNSS mode. Defaults to RTK FIXED. See `CfgDGNSS` message for constants.

### For TIM devices:
* `tim_tm2`: Enable this message for GPS time-stamps everytime an external interrupt signal is received.

### For FTS devices:
* currently unimplemented. See `FtsProduct` class in `ublox_gps` package `node.h` & `node.cpp` files.

## Fix Topics

`~fix`([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))

Navigation Satellite fix.

`~fix_velocity`([geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html))

Velocity in local ENU frame.

## INF messages
To enable printing INF messages to the ROS console, set the parameters below.
* `inf/all`: This is the default value for the INF parameters below, which enable printing u-blox `INF` messages to the ROS console. It defaults to true. Individual message types can be turned off by setting their corresponding parameter to false.
* `inf/debug`: Whether to configure the UBX and NMEA ports to send Debug messages and print received `INF-Debug` messages to `ROS_DEBUG` console.
* `inf/error`: Whether to enable Error messages for the UBX and NMEA ports and print received `INF-Error` messages to `ROS_ERROR` console.
* `inf/notice`: Whether to enable Notice messages for the UBX and NMEA ports and print received `INF-Notice messages to `ROS_INFO` console.
* `inf/test`: Whether to enable Test messages for the UBX and NMEA ports and print received `INF-Test` messages to `ROS_INFO` console.
* `inf/warning`: Whether to enable Warning messages for the UBX and NMEA ports and print received `INF-Warning` messages to the `ROS_WARN` console.

## Additional Topics
To publish a given u-blox message to a ROS topic, set the parameter shown below to true. The node sets the rate of the u-blox messages to 1 measurement cycle. 

### All messages
* `publish/all`: This is the default value for `publish/<class>/all` parameters below. It defaults to false. Individual message classes and messages can be enabled or disabled by setting the parameters described below to false.

### AID messages
* `publish/aid/all`: This is the default value for the `publish/aid/<message>` parameters below. It defaults to `publish/all`. Individual messages can be enabled or disabled by setting the parameters below.
* `publish/aid/alm`: Topic `~aidalm`
* `publish/aid/eph`: Topic `~aideph`
* `publish/aid/hui`: Topic `~aidhui`

### RXM messages
* `publish/rxm/all`: This is the default value for the `publish/rxm/<message>` parameters below. It defaults to `publish/all`. Individual messages can be enabled or disabled by setting the parameters below.
* `publish/rxm/alm`: Topic `~rxmalm`
* `publish/rxm/eph`: Topic `~rxmeph`
* `publish/rxm/raw`: Topic `~rxmraw`. Type is either `RxmRAW` or `RxmRAWX` depending on firmware version.
* `publish/rxm/rtcm`: Topic `~rxmrtcm`. **Firmware >= 8 only**
* `publish/rxm/sfrb`: Topic `~rxmsfrb`. Type is either `RxmSFRB` or `RxmSFRBX` depending on firmware version.

### MON messages
* `publish/mon/all`: This is the default value for the `publish/mon/<message>` parameters below. It defaults to `publish/all`. Individual messages can be enabled or disabled by setting the parameters below.
* `publish/mon/hw`: Topic `~monhw`

### NAV messages
* `publish/nav/all`: This is the default value for the `publish/mon/<message>` parameters below. It defaults to `publish/all`. Individual messages can be enabled or disabled by setting the parameters below.
* `publish/nav/att`: Topic `~navatt`. **ADR/UDR devices only**
* `publish/nav/clock`: Topic `~navclock`
* `publish/nav/posecef`: Topic `~navposecef`
* `publish/nav/posllh`: Topic `~navposllh`. **Firmware <= 6 only.** For firmware 7 and above, see NavPVT
* `publish/nav/pvt`: Topic `~navpvt`. **Firmware >= 7 only.**
* `publish/nav/relposned`: Topic `~navrelposned`. **HPG Rover devices only**
* `publish/nav/heading`: Topic `~navheading`. **HP Position receiver devices only.** For firmware 9 and above
* `publish/nav/sat`: Topic `~navsat`
* `publish/nav/sol`: Topic `~navsol`. **Firmware <= 6 only.** For firmware 7 and above, see NavPVT
* `publish/nav/status`: Topic `~navstatus`
* `publish/nav/svin`: Topic `~navsvin`. **HPG Reference Station Devices only**
* `publish/nav/svinfo`: Topic `~navsvinfo`
* `publish/nav/velned`: Topic `~navvelned`. **Firmware <= 6 only.** For firmware 7 and above, see NavPVT

### ESF messages
* `publish/esf/all`: This is the default value for the `publish/esf/<message>` parameters below. It defaults to `publish/all` for **ADR/UDR devices**. Individual messages can be enabled or disabled by setting the parameters below.
* `publish/esf/ins`: Topic `~esfins`
* `publish/esf/meas`: Topic `~esfmeas`
* `publish/esf/raw`: Topic `~esfraw`
* `publish/esf/status`: Topic `~esfstatus`

### HNR messages
* `publish/hnr/pvt`: Topic `~hnrpvt`. **ADR/UDR devices only**

### TIM messages
* `publish/tim/tm2`: Topic `timtm2`. **TIM devices only**

## Launch

A sample launch file `ublox_device.launch` loads the parameters from a `zed_f9p.yaml` file in the current folder, sample configuration files are in the `config_examples` folder.
The two topics to which you should subscribe are `~fix` and `~fix_velocity`. The angular component of `fix_velocity` is unused.
