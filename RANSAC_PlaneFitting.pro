QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += main.cpp \
    planefitting.cpp

#Opencv
INCLUDEPATH+=E:/opencv4.5.2/opencv/build/include
LIBS+= -LE:/opencv4.5.2/opencv/build/x64/vc14/lib -lopencv_world452


#Pcl
INCLUDEPATH += E:/pcl1.8.1/PCL1.8.1/include/pcl-1.8\

INCLUDEPATH += E:/pcl1.8.1/PCL1.8.1/include/pcl-1.8/pcl\

INCLUDEPATH += E:/pcl1.8.1/PCL1.8.1/3rdParty/Boost/include/boost-1_64\

INCLUDEPATH += E:/pcl1.8.1/PCL1.8.1/3rdParty/Boost/include/boost-1_64/boost\

INCLUDEPATH += E:/pcl1.8.1/PCL1.8.1/3rdParty/Eigen/eigen3\

INCLUDEPATH += E:/pcl1.8.1/PCL1.8.1/3rdParty/FLANN/include\

INCLUDEPATH += E:/pcl1.8.1/PCL1.8.1/3rdParty/FLANN/include/flann\

INCLUDEPATH += E:/pcl1.8.1/PCL1.8.1/3rdParty/OpenNI2/Include\

INCLUDEPATH += E:/pcl1.8.1/PCL1.8.1/3rdParty/Qhull/include\

INCLUDEPATH += E:/pcl1.8.1/VTK/include/vtk-8.0\

CONFIG(debug,debug|release){
LIBS += -LE:\pcl1.8.1\PCL1.8.1\lib\
        -lpcl_common_debug\
        -lpcl_features_debug\
        -lpcl_filters_debug\
        -lpcl_io_debug\
        -lpcl_io_ply_debug\
        -lpcl_kdtree_debug\
        -lpcl_keypoints_debug\
        -lpcl_ml_debug\
        -lpcl_octree_debug\
        -lpcl_outofcore_debug\
        -lpcl_people_debug\
        -lpcl_recognition_debug\
        -lpcl_registration_debug\
        -lpcl_sample_consensus_debug\
        -lpcl_search_debug\
        -lpcl_segmentation_debug\
        -lpcl_stereo_debug\
        -lpcl_surface_debug\
        -lpcl_tracking_debug\
        -lpcl_visualization_debug

LIBS += -LE:\pcl1.8.1\PCL1.8.1\3rdParty\Boost\lib\
        -llibboost_atomic-vc140-mt-gd-1_64\
        -llibboost_bzip2-vc140-mt-gd-1_64\
        -llibboost_chrono-vc140-mt-gd-1_64\
        -llibboost_container-vc140-mt-gd-1_64\
        -llibboost_context-vc140-mt-gd-1_64\
        -llibboost_coroutine-vc140-mt-gd-1_64\
        -llibboost_date_time-vc140-mt-gd-1_64\
        -llibboost_exception-vc140-mt-gd-1_64\
        -llibboost_fiber-vc140-mt-gd-1_64\
        -llibboost_filesystem-vc140-mt-gd-1_64\
        -llibboost_graph-vc140-mt-gd-1_64\
        -llibboost_graph_parallel-vc140-mt-gd-1_64\
        -llibboost_iostreams-vc140-mt-gd-1_64\
        -llibboost_locale-vc140-mt-gd-1_64\
        -llibboost_log-vc140-mt-gd-1_64\
        -llibboost_log_setup-vc140-mt-gd-1_64\
        -llibboost_math_c99-vc140-mt-gd-1_64\
        -llibboost_math_c99f-vc140-mt-gd-1_64\
        -llibboost_math_c99l-vc140-mt-gd-1_64\
        -llibboost_math_tr1-vc140-mt-gd-1_64\
        -llibboost_math_tr1f-vc140-mt-gd-1_64\
        -llibboost_math_tr1l-vc140-mt-gd-1_64\
        -llibboost_mpi-vc140-mt-gd-1_64\
        -llibboost_numpy-vc140-mt-gd-1_64\
        -llibboost_numpy3-vc140-mt-gd-1_64\
        -llibboost_prg_exec_monitor-vc140-mt-gd-1_64\
        -llibboost_program_options-vc140-mt-gd-1_64\
        -llibboost_python-vc140-mt-gd-1_64\
        -llibboost_python3-vc140-mt-gd-1_64\
        -llibboost_random-vc140-mt-gd-1_64\
        -llibboost_regex-vc140-mt-gd-1_64\
        -llibboost_serialization-vc140-mt-gd-1_64\
        -llibboost_signals-vc140-mt-gd-1_64\
        -llibboost_system-vc140-mt-gd-1_64\
        -llibboost_test_exec_monitor-vc140-mt-gd-1_64\
        -llibboost_thread-vc140-mt-gd-1_64\
        -llibboost_timer-vc140-mt-gd-1_64\
        -llibboost_type_erasure-vc140-mt-gd-1_64\
        -llibboost_unit_test_framework-vc140-mt-gd-1_64\
        -llibboost_wave-vc140-mt-gd-1_64\
        -llibboost_wserialization-vc140-mt-gd-1_64\
        -llibboost_zlib-vc140-mt-gd-1_64

LIBS += -LE:\pcl1.8.1\PCL1.8.1\3rdParty\FLANN\lib\
        -lflann-gd\
        -lflann_cpp-gd\
        -lflann_cpp_s-gd\
        -lflann_s-gd

LIBS += -LE:\pcl1.8.1\PCL1.8.1\3rdParty\OpenNI2\Lib\
        -lOpenNI2

LIBS += -LE:\pcl1.8.1\PCL1.8.1\3rdParty\Qhull\lib\
        -lqhull_d\
        -lqhullcpp_d\
        -lqhullstatic_d\
        -lqhullstatic_r_d\
        -lqhull_p_d\
        -lqhull_r_d

LIBS += -LE:\Qt5.9.5\5.9.5\msvc2015_64\lib\
        -lQt5OpenGL

LIBS += -LE:\pcl1.8.1\VTK\lib\
        -lvtkalglib-8.0\
        -lvtkChartsCore-8.0\
        -lvtkCommonColor-8.0\
        -lvtkCommonComputationalGeometry-8.0\
        -lvtkCommonCore-8.0\
        -lvtkCommonDataModel-8.0\
        -lvtkCommonExecutionModel-8.0\
        -lvtkCommonMath-8.0\
        -lvtkCommonMisc-8.0\
        -lvtkCommonSystem-8.0\
        -lvtkCommonTransforms-8.0\
        -lvtkDICOMParser-8.0\
        -lvtkDomainsChemistry-8.0\
        -lvtkexoIIc-8.0\
        -lvtkexpat-8.0\
        -lvtkFiltersAMR-8.0\
        -lvtkFiltersCore-8.0\
        -lvtkFiltersExtraction-8.0\
        -lvtkFiltersFlowPaths-8.0\
        -lvtkFiltersGeneral-8.0\
        -lvtkFiltersGeneric-8.0\
        -lvtkFiltersGeometry-8.0\
        -lvtkFiltersHybrid-8.0\
        -lvtkFiltersHyperTree-8.0\
        -lvtkFiltersImaging-8.0\
        -lvtkFiltersModeling-8.0\
        -lvtkFiltersParallel-8.0\
        -lvtkFiltersParallelImaging-8.0\
        -lvtkFiltersPoints-8.0\
        -lvtkFiltersProgrammable-8.0\
        -lvtkFiltersSelection-8.0\
        -lvtkFiltersSMP-8.0\
        -lvtkFiltersSources-8.0\
        -lvtkFiltersStatistics-8.0\
        -lvtkFiltersTexture-8.0\
        -lvtkFiltersTopology-8.0\
        -lvtkFiltersVerdict-8.0\
        -lvtkfreetype-8.0\
        -lvtkGeovisCore-8.0\
        -lvtkgl2ps-8.0\
        -lvtkGUISupportQt-8.0\
        #-lvtkGUISupportQtOpenGL-8.0\
        -lvtkGUISupportQtSQL-8.0\
        -lvtkhdf5-8.0\
        -lvtkhdf5_hl-8.0\
        -lvtkImagingColor-8.0\
        -lvtkImagingCore-8.0\
        -lvtkImagingFourier-8.0\
        -lvtkImagingGeneral-8.0\
        -lvtkImagingHybrid-8.0\
        -lvtkImagingMath-8.0\
        -lvtkImagingMorphological-8.0\
        -lvtkImagingSources-8.0\
        -lvtkImagingStatistics-8.0\
        -lvtkImagingStencil-8.0\
        -lvtkInfovisCore-8.0\
        -lvtkInfovisLayout-8.0\
        -lvtkInteractionImage-8.0\
        -lvtkInteractionStyle-8.0\
        -lvtkInteractionWidgets-8.0\
        -lvtkIOAMR-8.0\
        -lvtkIOCore-8.0\
        -lvtkIOEnSight-8.0\
        -lvtkIOExodus-8.0\
        -lvtkIOExport-8.0\
        -lvtkIOExportOpenGL2-8.0\
        -lvtkIOGeometry-8.0\
        -lvtkIOImage-8.0\
        -lvtkIOImport-8.0\
        -lvtkIOInfovis-8.0\
        -lvtkIOLegacy-8.0\
        -lvtkIOLSDyna-8.0\
        -lvtkIOMINC-8.0\
        -lvtkIOMovie-8.0\
        -lvtkIONetCDF-8.0\
        -lvtkIOParallel-8.0\
        -lvtkIOParallelXML-8.0\
        -lvtkIOPLY-8.0\
        -lvtkIOSQL-8.0\
        -lvtkIOTecplotTable-8.0\
        -lvtkIOVideo-8.0\
        -lvtkIOXML-8.0\
        -lvtkIOXMLParser-8.0\
        -lvtkjpeg-8.0\
        -lvtkjsoncpp-8.0\
        -lvtklibharu-8.0\
        -lvtklibxml2-8.0\
        -lvtklz4-8.0\
        -lvtkmetaio-8.0\
        -lvtkNetCDF-8.0\
        -lvtknetcdf_c++\
        -lvtkoggtheora-8.0\
        -lvtkParallelCore-8.0\
        -lvtkpng-8.0\
        -lvtkproj4-8.0\
        -lvtkRenderingAnnotation-8.0\
        -lvtkRenderingContext2D-8.0\
        -lvtkRenderingContextOpenGL2-8.0\
        -lvtkRenderingCore-8.0\
        -lvtkRenderingFreeType-8.0\
        -vtkRenderingGL2PSOpenGL2-8.0\
        -lvtkRenderingImage-8.0\
        -lvtkRenderingLabel-8.0\
        #-lvtkRenderingLIC-8.0\
        -lvtkRenderingLOD-8.0\
        -lvtkRenderingOpenGL2-8.0\
        -lvtkRenderingQt-8.0\
        -lvtkRenderingVolume-8.0\
        -lvtkRenderingVolumeOpenGL2-8.0\
        -lvtksqlite-8.0\
        -lvtksys-8.0\
        -lvtktiff-8.0\
        -lvtkverdict-8.0\
        -lvtkViewsContext2D-8.0\
        -lvtkViewsCore-8.0\
        -lvtkViewsInfovis-8.0\
        -lvtkViewsQt-8.0\
        -lvtkzlib-8.0
} else {
LIBS += -LE:\pcl1.8.1\PCL1.8.1\lib\
        -lpcl_common_release\
        -lpcl_features_release\
        -lpcl_filters_release\
        -lpcl_io_ply_release\
        -lpcl_io_release\
        -lpcl_kdtree_release\
        -lpcl_keypoints_release\
        -lpcl_ml_release\
        -lpcl_octree_release\
        -lpcl_outofcore_release\
        -lpcl_people_release\
        -lpcl_recognition_release\
        -lpcl_registration_release\
        -lpcl_sample_consensus_release\
        -lpcl_search_release\
        -lpcl_segmentation_release\
        -lpcl_stereo_release\
        -lpcl_surface_release\
        -lpcl_tracking_release\
        -lpcl_visualization_release

LIBS += -LE:\pcl1.8.1\PCL1.8.1\3rdParty\Boost\lib\
        -llibboost_atomic-vc140-mt-1_64\
        -llibboost_bzip2-vc140-mt-1_64\
        -llibboost_chrono-vc140-mt-1_64\
        -llibboost_container-vc140-mt-1_64\
        -llibboost_context-vc140-mt-1_64\
        -llibboost_coroutine-vc140-mt-1_64\
        -llibboost_date_time-vc140-mt-1_64\
        -llibboost_exception-vc140-mt-1_64\
        -llibboost_fiber-vc140-mt-1_64\
        -llibboost_filesystem-vc140-mt-1_64\
        -llibboost_graph-vc140-mt-1_64\
        -llibboost_graph_parallel-vc140-mt-1_64\
        -llibboost_iostreams-vc140-mt-1_64\
        -llibboost_locale-vc140-mt-1_64\
        -llibboost_log-vc140-mt-1_64\
        -llibboost_log_setup-vc140-mt-1_64\
        -llibboost_math_c99-vc140-mt-1_64\
        -llibboost_math_c99f-vc140-mt-1_64\
        -llibboost_math_c99l-vc140-mt-1_64\
        -llibboost_math_tr1-vc140-mt-1_64\
        -llibboost_math_tr1f-vc140-mt-1_64\
        -llibboost_math_tr1l-vc140-mt-1_64\
        -llibboost_mpi-vc140-mt-1_64\
        -llibboost_numpy-vc140-mt-1_64\
        -llibboost_numpy3-vc140-mt-1_64\
        -llibboost_prg_exec_monitor-vc140-mt-1_64\
        -llibboost_program_options-vc140-mt-1_64\
        -llibboost_python-vc140-mt-1_64\
        -llibboost_python3-vc140-mt-1_64\
        -llibboost_random-vc140-mt-1_64\
        -llibboost_regex-vc140-mt-1_64\
        -llibboost_serialization-vc140-mt-1_64\
        -llibboost_signals-vc140-mt-1_64\
        -llibboost_system-vc140-mt-1_64\
        -llibboost_test_exec_monitor-vc140-mt-1_64\
        -llibboost_thread-vc140-mt-1_64\
        -llibboost_timer-vc140-mt-1_64\
        -llibboost_type_erasure-vc140-mt-1_64\
        -llibboost_unit_test_framework-vc140-mt-1_64\
        -llibboost_wave-vc140-mt-1_64\
        -llibboost_wserialization-vc140-mt-1_64\
        -llibboost_zlib-vc140-mt-1_64

LIBS += -LE:\pcl1.8.1\PCL1.8.1\3rdParty\FLANN\lib\
        -lflann\
        -lflann_cpp\
        -lflann_cpp_s\
        -lflann_s

LIBS += -LE:\pcl1.8.1\PCL1.8.1\3rdParty\OpenNI2\Lib\
        -lOpenNI2

LIBS += -LE:\pcl1.8.1\PCL1.8.1\3rdParty\Qhull\lib\
        -lqhull\
        -lqhullcpp\
        -lqhullstatic\
        -lqhullstatic_r\
        -lqhull_p\
        -lqhull_r

LIBS += -LE:\pcl1.8.1\VTK\lib\
        -lvtkalglib-8.0\
        -lvtkChartsCore-8.0\
        -lvtkCommonColor-8.0\
        -lvtkCommonComputationalGeometry-8.0\
        -lvtkCommonCore-8.0\
        -lvtkCommonDataModel-8.0\
        -lvtkCommonExecutionModel-8.0\
        -lvtkCommonMath-8.0\
        -lvtkCommonMisc-8.0\
        -lvtkCommonSystem-8.0\
        -lvtkCommonTransforms-8.0\
        -lvtkDICOMParser-8.0\
        -lvtkDomainsChemistry-8.0\
        -lvtkDomainsChemistryOpenGL2-8.0\
        -lvtkexoIIc-8.0\
        -lvtkexpat-8.0\
        -lvtkFiltersAMR-8.0\
        -lvtkFiltersCore-8.0\
        -lvtkFiltersExtraction-8.0\
        -lvtkFiltersFlowPaths-8.0\
        -lvtkFiltersGeneral-8.0\
        -lvtkFiltersGeneric-8.0\
        -lvtkFiltersGeometry-8.0\
        -lvtkFiltersHybrid-8.0\
        -lvtkFiltersHyperTree-8.0\
        -lvtkFiltersImaging-8.0\
        -lvtkFiltersModeling-8.0\
        -lvtkFiltersParallel-8.0\
        -lvtkFiltersParallelImaging-8.0\
        -lvtkFiltersProgrammable-8.0\
        -lvtkFiltersSelection-8.0\
        -lvtkFiltersSMP-8.0\
        -lvtkFiltersSources-8.0\
        -lvtkFiltersStatistics-8.0\
        -lvtkFiltersTexture-8.0\
        -lvtkFiltersVerdict-8.0\
        -lvtkfreetype-8.0\
        -lvtkGeovisCore-8.0\
        -lvtkglew-8.0\
        -lvtkGUISupportQt-8.0\
        -lvtkGUISupportQtSQL-8.0\
        -lvtkhdf5-8.0\
        -lvtkhdf5_hl-8.0\
        -lvtkImagingColor-8.0\
        -lvtkImagingCore-8.0\
        -lvtkImagingFourier-8.0\
        -lvtkImagingGeneral-8.0\
        -lvtkImagingHybrid-8.0\
        -lvtkImagingMath-8.0\
        -lvtkImagingMorphological-8.0\
        -lvtkImagingSources-8.0\
        -lvtkImagingStatistics-8.0\
        -lvtkImagingStencil-8.0\
        -lvtkInfovisCore-8.0\
        -lvtkInfovisLayout-8.0\
        -lvtkInteractionImage-8.0\
        -lvtkInteractionStyle-8.0\
        -lvtkInteractionWidgets-8.0\
        -lvtkIOAMR-8.0\
        -lvtkIOCore-8.0\
        -lvtkIOEnSight-8.0\
        -lvtkIOExodus-8.0\
        -lvtkIOExport-8.0\
        -lvtkIOGeometry-8.0\
        -lvtkIOImage-8.0\
        -lvtkIOImport-8.0\
        -lvtkIOInfovis-8.0\
        -lvtkIOLegacy-8.0\
        -lvtkIOLSDyna-8.0\
        -lvtkIOMINC-8.0\
        -lvtkIOMovie-8.0\
        -lvtkIONetCDF-8.0\
        -lvtkIOParallel-8.0\
        -lvtkIOParallelXML-8.0\
        -lvtkIOPLY-8.0\
        -lvtkIOSQL-8.0\
        -lvtkIOVideo-8.0\
        -lvtkIOXML-8.0\
        -lvtkIOXMLParser-8.0\
        -lvtkjpeg-8.0\
        -lvtkjsoncpp-8.0\
        -lvtklibxml2-8.0\
        -lvtkmetaio-8.0\
        -lvtkNetCDF-8.0\
        -lvtknetcdf_c++\		#因为我的VTK库文件中不存在该文件，所以把它注释掉
        -lvtkoggtheora-8.0\
        -lvtkParallelCore-8.0\
        -lvtkpng-8.0\
        -lvtkproj4-8.0\
        -lvtkRenderingAnnotation-8.0\
        -lvtkRenderingContext2D-8.0\
        -lvtkRenderingContextOpenGL2-8.0\
        -lvtkRenderingCore-8.0\
        -lvtkRenderingFreeType-8.0\
        -lvtkRenderingImage-8.0\
        -lvtkRenderingLabel-8.0\
        -lvtkRenderingLOD-8.0\
        -lvtkRenderingOpenGL2-8.0\
        -lvtkRenderingQt-8.0\
        -lvtkRenderingVolume-8.0\
        -lvtkRenderingVolumeOpenGL2-8.0\
        -lvtksqlite-8.0\
        -lvtksys-8.0\
        -lvtktiff-8.0\
        -lvtkverdict-8.0\
        -lvtkViewsContext2D-8.0\
        -lvtkViewsCore-8.0\
        -lvtkViewsInfovis-8.0\
        -lvtkViewsQt-8.0\
        -lvtkzlib-8.0
}

HEADERS += \
    planefitting.h
