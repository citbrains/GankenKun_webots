project(HCServer)

IF(WIN32)
    set(Boost_USE_STATIC_LIBS ON)
ELSE(WIN32)
    set(Boost_USE_STATIC_LIBS OFF)
ENDIF(WIN32)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)

SET(Boost_DEBUG TRUE)
FIND_PACKAGE(Boost REQUIRED COMPONENTS system thread)

find_package(Qt4 REQUIRED)
set(QT_USE_QTMAIN true)
set(QT_USE_QTGUI true)
include(${QT_USE_FILE})

set(ICE_DIR $ENV{ICE_HOME} CACHE PATH "ZeroC ICE installation dir(Default: $ICE_HOME)")

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/gen)

include_directories(
	${ICE_DIR}/include
	${QT_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
	${PROJECT_BINARY_DIR}
	${CMAKE_BINARY_DIR}/gen
	${BOOST_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
)

link_directories(
	${ICE_DIR}/lib
	${Boost_LIBRARY_DIRS}
	${Boost_LIBRARY_DIR}
	${QT_LIBRARY_DIR}
)

MACRO (DEFINE_ICE_SOURCES idlfilename)
	SET(outsources)
	GET_FILENAME_COMPONENT(it ${idlfilename} ABSOLUTE)
	GET_FILENAME_COMPONENT(nfile ${idlfilename} NAME_WE)
	SET(outsources ${outsources} ${CMAKE_BINARY_DIR}/gen/${nfile}.cpp ${CMAKE_BINARY_DIR}/gen/${nfile}.h)
ENDMACRO(DEFINE_ICE_SOURCES)

MACRO (PROCESS_SLICE idlfilename)
	GET_FILENAME_COMPONENT(it ${idlfilename} ABSOLUTE)
	GET_FILENAME_COMPONENT(filename ${idlfilename} NAME)
	GET_FILENAME_COMPONENT(incpath ${idlfilename} PATH)
	DEFINE_ICE_SOURCES(${ARGV})
	ADD_CUSTOM_COMMAND (
		OUTPUT ${outsources}
		COMMAND slice2cpp
		ARGS --output-dir ${CMAKE_BINARY_DIR}/gen ${it}
		DEPENDS ${it}
	)
ENDMACRO (PROCESS_SLICE)


SET(HCServer_SRCS
	HCServer.cpp
)

PROCESS_SLICE(${PROJECT_SOURCE_DIR}/StringSender.ice)
DEFINE_ICE_SOURCES(${PROJECT_SOURCE_DIR}/StringSender.ice)
foreach(f ${outsources})
set_source_files_properties(${CMAKE_BINARY_DIR}/${f} PROPERTIES GENERATED 1)
GET_FILENAME_COMPONENT(absf ${f} ABSOLUTE)
#set(ICE_DATAMODEL ${ICE_DATAMODEL} ${CMAKE_BINARY_DIR}/${f})
set(ICE_DATAMODEL ${ICE_DATAMODEL} ${absf})
endforeach(f)


IF(WIN32)
add_definitions(-D__WIN32__ -D__x86__ -DWINVER=0x0502 -D_WIN32_WINNT=0x0501 -D__NT__ -D__OSVERSION__=5 -DNOMINMAX -DEIGEN_NO_DEBUG)
ENDIF(WIN32)

add_library(HCLib
	${ICE_DATAMODEL}
	HCIPC.cpp
	HCIPC.h
	hajime_command.h
	hajime_command.c
)

target_link_libraries(HCLib
	pthread
	${Boost_LIBRARIES}
	debug Iced
	optimized Ice
	debug IceUtild
	optimized IceUtil
)

add_executable(HCServer
	${HCServer_SRCS}
)

add_dependencies(HCServer
	HCLib
)

target_link_libraries(HCServer
	pthread
	${Boost_LIBRARIES}
	HCLib
)

add_executable(HCServerDemoclient
	HCServerDemoclient.cpp
)

add_dependencies(HCServerDemoclient
	HCLib
)

target_link_libraries(HCServerDemoclient
	${Boost_LIBRARIES}
	HCLib
)

set(HCDelegate_MOC_HDRS
	HCDelegate.h
)

QT4_ADD_RESOURCES( HCDelegate_RC_SRCS HCDelegate.qrc )
QT4_WRAP_UI( HCDelegate_UIS_H HCDelegate.ui )
QT4_WRAP_CPP( HCDelegate_MOC_SRCS ${HCDelegate_MOC_HDRS} )

add_executable(HCDelegate WIN32
	${HCDelegate_UIS_H}
	${HCDelegate_MOC_HDRS}
	${HCDelegate_MOC_SRCS}
	hcdelegate_main.cpp
	HCDelegate.h
	HCDelegate.cpp
)

target_link_libraries(HCDelegate
	${Boost_LIBRARIES}
	${QT_LIBRARIES}
	HCLib
)

add_executable(HCProxy
	HCProxy.cpp
)

target_link_libraries(HCProxy
	${Boost_LIBRARIES}
	HCLib
)

