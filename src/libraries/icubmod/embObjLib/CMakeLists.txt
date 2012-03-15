# Create a library for all embObj stuff

SET(PROJECTNAME embObj-lib)
PROJECT(${PROJECTNAME})

set(BASE ${CMAKE_CURRENT_SOURCE_DIR}/)

#set(CFG_FOLDER  	${BASE}body/embobj/embobj/embobj-cfg)
#set(SRC_FOLDER  	${BASE}body/embobj/embobj/embobj-src)
#set(NV_FOLDER   	${BASE}body/embobj/embobj/embobj-src)
#set(SYS_FOLDER  	${BASE}sys)
#set(HAL-FAKE_FOLDER ${BASE}hal-fake)

#include_directories(${SYS_FOLDER}/env/api)
#include_directories(${SYS_FOLDER}/env/api)
#include_directories(${BASE}body/porting)
#include_directories(${NV_FOLDER})
#include_directories(${HAL-FAKE_FOLDER})
#include_directories(${CFG_FOLDER})

##include_directories(${SRC_FOLDER})

# set(embObj_includes ${SYS_FOLDER}/env/api ${SYS_FOLDER}/env/api ${BASE}body/porting ${NV_FOLDER} ${HAL-FAKE_FOLDER} ${CFG_FOLDER} ${SRC_FOLDER} )  

include(../embObjLib/embObjLib.conf)
#message(INFO " lib_cmake - embObj_includes: ${embObj_includes}")
  	
  	
# get all files needed, collected in different lists for better organization and handling

# EnvCom
# set(ENVCOM_SOURCE ${BASE}arm/embody/body/embenv/envcom/eEcommon.c)
# set(ENVCOM_HEADER ${BASE}arm/embody/body/embenv/envcom/eEcommon.h)

# EmbObj SYS
set(EMBOBJ-SYS_SOURCE 	${SYS_FOLDER}/env/src/eEcommon.c)
set(EMBOBJ-SYS_HEADER 	${SYS_FOLDER}/env/api/eEcommon.h)

# EmbObj-Cfg
set(EMBOBJ-CFG_SOURCE 	${CFG_FOLDER}/eOcfg_nvsEP_base_con.c
						${CFG_FOLDER}/eOcfg_nvsEP_mngmnt_con.c 
						${CFG_FOLDER}/eOcfg_EPs_rem_board.c
						${CFG_FOLDER}/eOcfg_nvsEP_base_usr_rem_anydev.c
						${CFG_FOLDER}/eOcfg_nvsEP_mngmnt_usr_rem_board.c
						${CFG_FOLDER}/eOcfg_EPs_loc_board.c
#						${CFG_FOLDER}/eOcfg_nvsEP_base_usr_loc_anydev.c
#						${CFG_FOLDER}/eOcfg_nvsEP_mngmnt_usr_loc_board.c
						)
						
set(CFG-BOARD01_SOURCE 	${CFG_FOLDER}/eOcfg_EPs_FakeBoard01.c
						${CFG_FOLDER}/eOcfg_nvsEP_base_FakeBoard01.c
						${CFG_FOLDER}/eOcfg_nvsEP_mngmnt_FakeBoard01.c)
				
set(CFG-BOARD01_HEADER 	${CFG_FOLDER}/eOcfg_EPs_FakeBoard01.h
						${CFG_FOLDER}/eOcfg_nvsEP_base_FakeBoard01.h
						${CFG_FOLDER}/eOcfg_nvsEP_mngmnt_FakeBoard01.h)						

set(EMBOBJ-CFG_HEADER	${CFG_FOLDER}/eOcfg_nvsEP_base_con.h
						${CFG_FOLDER}/eOcfg_nvsEP_mngmnt_con.h 
						${CFG_FOLDER}/eOcfg_EPs_rem_board.h
						${CFG_FOLDER}/eOcfg_nvsEP_base_usr_rem_anydev.h
						${CFG_FOLDER}/eOcfg_nvsEP_mngmnt_usr_rem_board.h
						${CFG_FOLDER}/eOcfg_EPs_loc_board.h
						${CFG_FOLDER}/eOcfg_nvsEP_base_usr_loc_anydev.h
						${CFG_FOLDER}/eOcfg_nvsEP_mngmnt_usr_loc_board.h)

# EmbObj-src
set(EMBOBJ-SRC_SOURCE  	${SRC_FOLDER}/EOarray.c
						${SRC_FOLDER}/EoCommon.c
						${SRC_FOLDER}/EOconstarray.c
						${SRC_FOLDER}/EOconstvector.c
						${SRC_FOLDER}/EOdeque.c
						${SRC_FOLDER}/EOfifo.c
						${SRC_FOLDER}/EOfifoByte.c
						${SRC_FOLDER}/EOfifoWord.c
						${SRC_FOLDER}/EOlist.c
						${SRC_FOLDER}/EOtheErrorManager.c
						${SRC_FOLDER}/EOtheMemoryPool.c
						${SRC_FOLDER}/EOvector.c
						${SRC_FOLDER}/EOVmutex.c
						${SRC_FOLDER}/EOVstorage.c
						${SRC_FOLDER}/EOVtask.c
						${SRC_FOLDER}/EOVtheSystem.c
						${SRC_FOLDER}/EOmatrix3d.c)
						
set(EMBOBJ-SRC_HEADER  	${SRC_FOLDER}/EOarray_hid.h
#						${SRC_FOLDER}/EoCommon_hid.h
						${SRC_FOLDER}/EOconstarray_hid.h
						${SRC_FOLDER}/EOconstvector_hid.h
						${SRC_FOLDER}/EOdeque_hid.h
						${SRC_FOLDER}/EOfifo_hid.h
						${SRC_FOLDER}/EOfifoByte_hid.h
						${SRC_FOLDER}/EOfifoWord_hid.h
						${SRC_FOLDER}/EOlist_hid.h
						${SRC_FOLDER}/EOtheErrorManager_hid.h
						${SRC_FOLDER}/EOtheMemoryPool_hid.h
						${SRC_FOLDER}/EOvector_hid.h
						${SRC_FOLDER}/EOVmutex_hid.h
						${SRC_FOLDER}/EOVstorage_hid.h
						${SRC_FOLDER}/EOVtask_hid.h
						${SRC_FOLDER}/EOVtheSystem_hid.h
						${SRC_FOLDER}/EOmatrix3d_hid.h)						

# EmbObj-NV
set(EMBOBJ-NV_SOURCE  	${NV_FOLDER}/EOhostTransceiver.c
						${NV_FOLDER}/EOnv.c
						${NV_FOLDER}/EOnvsCfg.c
						${NV_FOLDER}/EOpacket.c
						${NV_FOLDER}/EOreceiver.c
						${NV_FOLDER}/EOrop.c
						${NV_FOLDER}/EOropframe.c
						${NV_FOLDER}/EOtheAgent.c
						${NV_FOLDER}/EOtheFormer.c
						${NV_FOLDER}/EOtheParser.c
						${NV_FOLDER}/EOtransceiver.c
						${NV_FOLDER}/EOtransmitter.c
						${NV_FOLDER}/EOtreenode.c)

set(EMBOBJ-NV_HEADER  	${NV_FOLDER}/EOhostTransceiver_hid.h
						${NV_FOLDER}/EOnv_hid.h
						${NV_FOLDER}/EOnvsCfg_hid.h
						${NV_FOLDER}/EOpacket_hid.h
						${NV_FOLDER}/EOreceiver_hid.h
						${NV_FOLDER}/EOrop_hid.h
						${NV_FOLDER}/EOropframe_hid.h
						${NV_FOLDER}/EOtheAgent_hid.h
						${NV_FOLDER}/EOtheFormer_hid.h
						${NV_FOLDER}/EOtheParser_hid.h
						${NV_FOLDER}/EOtransceiver_hid.h
						${NV_FOLDER}/EOtransmitter_hid.h
						${NV_FOLDER}/EOtreenode_hid.h)

set(HAL-FAKE_SOURCE		${HAL-FAKE_FOLDER}/hal_fake.c)

set(EMBOBJ-EMS_SOURCE	${SRC_FOLDER}/EOtheBOARDtransceiver.c)


set(embobj_source ${EMBOBJ-NV_SOURCE} ${EMBOBJ-SRC_SOURCE} ${EMBOBJ-CFG_SOURCE} ${EMBOBJ-SYS_SOURCE} ${HAL-FAKE_SOURCE} ${EMBOBJ-EMS_SOURCE} ${CFG-BOARD01_SOURCE})
set(embobj_header ${EMBOBJ-NV_HEADER} ${EMBOBJ-SRC_HEADER} ${EMBOBJ-CFG_HEADER} ${EMBOBJ-SYS_HEADER} ${CFG-BOARD01_HEADER})

SOURCE_GROUP("EmbObj Source Files" FILES ${folder_source})
SOURCE_GROUP("EmbObj Header Files" FILES ${folder_header})
  
add_library(${PROJECTNAME} ${embobj_source} ${embobj_header})


icub_export_library(${PROJECTNAME})
