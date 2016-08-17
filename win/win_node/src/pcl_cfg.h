#pragma once

// インクルード
//#pragma region Includes
//#include <pcl\point_types.h>
//#include <pcl\common\io.h>
//#include <pcl\io\pcd_io.h>
//#include <pcl\io\vtk_lib_io.h>
//#include <pcl\visualization\cloud_viewer.h>          // PCLVisualizer
//#include <pcl\keypoints\harris_3d.h>             // Harris特徴検出器
//#include <pcl\features\fpfh.h>
//#include <pcl\features\normal_3d.h>
//#include <pcl/segmentation/sac_segmentation.h>       // 平面検出
//#pragma endregion

// ビルドモード
#pragma region Build Mode
#ifdef _DEBUG
#define RSSDK_EXT_STR "_d.lib"
#define PCL_EXT_STR "_debug.lib"
#define PCL_BOOST_EXT_STR "-vc140-mt-gd-1_57.lib"
#define PCL_FLANN_EXT_STR "-gd.lib"
#define PCL_QHULL_EXT_STR "_d.lib"
#define PCL_VTK_EXT_STR "-6.2-gd.lib"
#else
#define RSSDK_EXT_STR ".lib"
#define PCL_EXT_STR "_release.lib"
#define PCL_BOOST_EXT_STR "-vc140-mt-1_57.lib"
#define PCL_FLANN_EXT_STR ".lib"
#define PCL_QHULL_EXT_STR ".lib"
#define PCL_VTK_EXT_STR "-6.2.lib"
#endif
#pragma endregion


// ライブラリのリンク（不要な物はコメントアウト）
#pragma region Intel RealSense
#pragma comment(lib, "libpxc"           RSSDK_EXT_STR)
#pragma comment(lib, "libpxcmd"         RSSDK_EXT_STR)
#pragma endregion

#pragma region PCL Linker Script
#pragma comment(lib, "pcl_common"           PCL_EXT_STR)
#pragma comment(lib, "pcl_features"         PCL_EXT_STR)
#pragma comment(lib, "pcl_filters"          PCL_EXT_STR)
#pragma comment(lib, "pcl_io"               PCL_EXT_STR)
#pragma comment(lib, "pcl_io_ply"           PCL_EXT_STR)
#pragma comment(lib, "pcl_kdtree"           PCL_EXT_STR)
#pragma comment(lib, "pcl_keypoints"        PCL_EXT_STR)
#pragma comment(lib, "pcl_octree"           PCL_EXT_STR)
#pragma comment(lib, "pcl_outofcore"        PCL_EXT_STR)
#pragma comment(lib, "pcl_people"           PCL_EXT_STR)
#pragma comment(lib, "pcl_recognition"      PCL_EXT_STR)
#pragma comment(lib, "pcl_registration"     PCL_EXT_STR)
#pragma comment(lib, "pcl_sample_consensus" PCL_EXT_STR)
#pragma comment(lib, "pcl_search"           PCL_EXT_STR)
#pragma comment(lib, "pcl_segmentation"     PCL_EXT_STR)
#pragma comment(lib, "pcl_surface"          PCL_EXT_STR)
#pragma comment(lib, "pcl_tracking"         PCL_EXT_STR)
#pragma comment(lib, "pcl_visualization"    PCL_EXT_STR)
#pragma endregion
#pragma region LibBoost Linker Script
#pragma comment(lib, "libboost_atomic"      PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_chrono"      PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_container"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_context"     PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_coroutine"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_date_time"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_exception"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_filesystem"      PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_graph"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_iostreams"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_locale"      PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_log_setup"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_log"     PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_math_c99f"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_math_c99l"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_math_c99"        PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_math_tr1f"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_math_tr1l"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_math_tr1"        PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_mpi"     PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_prg_exec_monitor"        PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_program_options"     PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_random"      PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_regex"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_serialization"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_signals"     PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_system"      PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_test_exec_monitor"       PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_thread"      PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_unit_test_framework"     PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_wave"        PCL_BOOST_EXT_STR)
#pragma comment(lib, "libboost_wserialization"      PCL_BOOST_EXT_STR)
#pragma endregion
#pragma region FLANN Linker Script
#pragma comment(lib, "flann"        PCL_FLANN_EXT_STR)
#pragma comment(lib, "flann_s"      PCL_FLANN_EXT_STR)
#pragma comment(lib, "flann_cpp_s"  PCL_FLANN_EXT_STR)
#pragma endregion
#pragma region QHull Linker Script
#pragma comment(lib, "qhull"    PCL_QHULL_EXT_STR)
#pragma comment(lib, "qhull_p"  PCL_QHULL_EXT_STR)
#pragma comment(lib, "qhullcpp" PCL_QHULL_EXT_STR)
//#pragma comment(lib, "qhullstatic"  PCL_QHULL_EXT_STR)
//#pragma comment(lib, "qhullstatic_p"    PCL_QHULL_EXT_STR)
#pragma endregion
#pragma region VTK Linker Script
#pragma comment(lib, "vtkalglib"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkChartsCore"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkCommonColor"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkCommonComputationalGeometry"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkCommonCore"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkCommonDataModel"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkCommonExecutionModel"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkCommonMath"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkCommonMisc"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkCommonSystem"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkCommonTransforms"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkDICOMParser"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkDomainsChemistry"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkexoIIc"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkexpat" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersAMR"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersCore"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersExtraction" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersFlowPaths"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersGeneral"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersGeneric"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersGeometry"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersHybrid" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersHyperTree"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersImaging"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersModeling"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersParallel"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersParallelImaging"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersProgrammable"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersSelection"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersSMP"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersSources"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersStatistics" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersTexture"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkFiltersVerdict"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkfreetype"  PCL_VTK_EXT_STR)
//#pragma comment(lib, "vtkftgl"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkGeovisCore"    PCL_VTK_EXT_STR)
//#pragma comment(lib, "vtkgl2ps" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkhdf5_hl"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkhdf5"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkImagingColor"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkImagingCore"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkImagingFourier"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkImagingGeneral"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkImagingHybrid" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkImagingMath"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkImagingMorphological"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkImagingSources"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkImagingStatistics" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkImagingStencil"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkInfovisCore"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkInfovisLayout" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkInteractionImage"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkInteractionStyle"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkInteractionWidgets"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOAMR" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOCore"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOEnSight" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOExodus"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOExport"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOGeometry"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOImage"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOImport"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOInfovis" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOLegacy"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOLSDyna"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOMinc"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOMovie"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIONetCDF"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOParallel"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOParallelXML" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOPLY" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOSQL" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOVideo"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOXML" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkIOXMLParser"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkjpeg"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkjsoncpp"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtklibxml2"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkmetaio"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkNetCDF_cxx"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkNetCDF"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkoggtheora" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkParallelCore"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkpng"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkproj4" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkRenderingAnnotation"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkRenderingContext2D"    PCL_VTK_EXT_STR)
//#pragma comment(lib, "vtkRenderingContextOpenGL"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkRenderingCore" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkRenderingFreeType" PCL_VTK_EXT_STR)
//#pragma comment(lib, "vtkRenderingFreeTypeOpenGL"   PCL_VTK_EXT_STR)
//#pragma comment(lib, "vtkRenderingGL2PS"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkRenderingImage"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkRenderingLabel"    PCL_VTK_EXT_STR)
//#pragma comment(lib, "vtkRenderingLIC"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkRenderingLOD"  PCL_VTK_EXT_STR)
//#pragma comment(lib, "vtkRenderingOpenGL"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkRenderingVolume"   PCL_VTK_EXT_STR)
//#pragma comment(lib, "vtkRenderingVolumeOpenGL" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtksqlite"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtksys"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtktiff"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkverdict"   PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkViewsContext2D"    PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkViewsCore" PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkViewsInfovis"  PCL_VTK_EXT_STR)
#pragma comment(lib, "vtkzlib"  PCL_VTK_EXT_STR)
#pragma endregion


