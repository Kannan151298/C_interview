# This is an automatically generated record.
# The area between QNX Internal Start and QNX Internal End is controlled by
# the QNX IDE properties.

ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

ifndef AMSS_DISPLAY_DEFS
AMSS_DISPLAY_DEFS=$(AMSS_ROOT)/multimedia/display/display_defs.mk
endif
include $(AMSS_DISPLAY_DEFS)

#===== PINFO - information about the project.
define PINFO
PINFO DESCRIPTION=This project is the QC implementation of the control logic for DS90UH981Q serializer chip, instance 0.
endef

#===== USEFILE - the file containing the usage message for the application.
USEFILE=

#===== CCFLAGS - add the flags to the C compiler command line.
CCFLAGS+=-Werror

#===== EXTRA_INCVPATH - a space-separated list of directories to search for include files as part of the staging process.
EXTRA_INCVPATH+=                                                                      \
    $(PROJECT_ROOT)/../../../../bosch/bsp/display/bridge_chip/plugins/bridge-communication-api

#===== INCVPATH - a space-separated list of directories to search for include files.
# FOR PUBLIC API HEADER FILES
INCVPATH+=                                                                            \
    $(INSTALL_ROOT_nto)/usr/include/amss                                              \
    $(INSTALL_ROOT_nto)/usr/include/amss/core                                         \
# FOR DISPLAY SYSTEM COMPONENT
INCVPATH+=                                                                            \
    $(BSP_ROOT)/boards/display/common/bridge_chip/drv/platform/inc                    \
    $(BSP_ROOT)/boards/display/common/bridge_chip/drv/main/inc                        \
    $(CUST)/bosch/boards/core/dalconfig/gm_vcu_sda8155/config                         \
    $(BSP_ROOT)/bosch/bsp/supply_handling
# FOR DISPLAY INTERNAL COMPONENT
INCVPATH+=                                                                            \
    $(PROJECT_ROOT)/inc                                                               \

#integrate display infras
EXTRA_INCVPATH+= $(BSP_ROOT)/bosch/bsp/display/api/public/sus_res_comm_lib/public
EXTRA_INCVPATH+= $(BSP_ROOT)/bosch/bsp/display/api/public/wakeup_sigs_lib/public
EXTRA_INCVPATH+= $(BSP_ROOT)/bosch/bsp/display/display_susd_pm/inc

#===== EXTRA_SRCVPATH - a space-separated list of directories to search for source files.
EXTRA_SRCVPATH+=                                                                      \
    $(PROJECT_ROOT)/src                                                               \
    $(PROJECT_ROOT)/../../../../bosch/bsp/display/bridge_chip/plugins/bridge-communication-api

#===== EXTRA_LIBVPATH - a space-separated list of directories to search for library files.
EXTRA_LIBVPATH+=$(INSTALLDIR_S8195_LIB)                                             \

#===== VERSION_TAG_SO - version tag for SONAME. Use it only if you don't like SONAME_VERSION
override VERSION_TAG_SO=

#===== LIBS - a space-separated list of library items to be included in the link.
LIBS+= bridgechip_drv_platformS i2c_clientS\
       dalsysS dalcfgS dalconfig nspinclockcyclesS\
       dll_utilsS slog2 sus_res_comm_lib  wakeup_sigs_lib

NAME=bridge_983t_ipd_aux_mst_dp1

#===== INSTALLDIR - Subdirectory where the executable or library is to be installed.
INSTALLDIR=$(INSTALLDIR_S8195_LIB)


PRE_TARGET=sus_res_comm_lib wakeup_sigs_lib
sus_res_comm_lib:
	$(MAKE)  -C $(BSP_ROOT)/bosch/bsp/display/api/public/sus_res_comm_lib install
wakeup_sigs_lib:
	$(MAKE)  -C $(BSP_ROOT)/bosch/bsp/display/api/public/wakeup_sigs_lib install

include $(MKFILES_ROOT)/qmacros.mk
ifndef QNX_INTERNAL
QNX_INTERNAL=$(PROJECT_ROOT)/.qnx_internal.mk
#FIXME: This will be removed when platform support is added
QNX_INTERNAL=$(BSP_ROOT)/boards/display/.qnx_internal.mk
endif
include $(QNX_INTERNAL)

include $(MKFILES_ROOT)/qtargets.mk

OPTIMIZE_TYPE_g=none
OPTIMIZE_TYPE=$(OPTIMIZE_TYPE_$(filter g, $(VARIANTS)))
