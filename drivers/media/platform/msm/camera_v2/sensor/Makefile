GCOV_PROFILE := y
ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/zte_io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci

#obj-$(CONFIG_MSMB_CAMERA) += cci/ io/ csiphy/ csid/ actuator/ flash/ eeprom/ ois/
obj-$(CONFIG_MSMB_CAMERA) += cci/ io/ csiphy/ csid/
obj-$(CONFIG_MSM_CAMERA_SENSOR) += zte_io/
obj-$(CONFIG_MSM_CAMERA_SENSOR) += msm_sensor_init.o msm_sensor_driver.o msm_sensor.o
obj-$(CONFIG_MT9M114) += mt9m114.o
obj-$(CONFIG_OV5645) += ov5645.o
obj-$(CONFIG_OV2685) += ov2685.o
obj-$(CONFIG_OV2685_SCV3B4035) += ov2685_scv3b4035.o
obj-$(CONFIG_OV5645) += ov7695.o
obj-$(CONFIG_GC0310) += gc0310.o
#
# Modified for camera sensor by ZTE_JIA_20130627 jia.jia
# To avoid side effect on initialization of camera sensor
#
obj-$(CONFIG_MSMB_CAMERA) += eeprom/
obj-$(CONFIG_MSMB_CAMERA) += actuator/
obj-$(CONFIG_MSMB_CAMERA) += flash/
obj-$(CONFIG_MSMB_CAMERA) += ois/