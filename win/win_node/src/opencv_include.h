//バージョンの異なるOpenCVを扱うとき、複数のソースで統一的にインクルード、リンクするためのヘッダ

#pragma once

//ライブラリのパス（インクルード、ライブラリ）は、メニュー[表示]→[プロパティ マネージャ]→DebugやReleaseの中の、Microsoft.Cpp.x64.userやMicrosoft.Cpp.Win32.userをダブルクリック、
//ダイアログ中の上のほう、VC++ディレクトリ、にそれぞれ指定する。
//同一ユーザであれば、一度設定すれば、すべてのソリューションで有効となる。


#include <opencv2/opencv.hpp>
//#include <opencv2/gpu/gpu.hpp>


#ifdef _DEBUG
	//Debugモードの場合
	#pragma comment(lib,"opencv_world310d.lib")
#else
	//Releaseモードの場合
	#pragma comment(lib,"opencv_world310.lib")
#endif


//#define CV_VERSION_NUMBER CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
//
//#ifdef _DEBUG
//	//Debugモードの場合
//	#pragma comment(lib,"opencv_core"CV_VERSION_NUMBER"d.lib")
//	#pragma comment(lib,"opencv_imgproc"CV_VERSION_NUMBER"d.lib")
//	#pragma comment(lib,"opencv_highgui"CV_VERSION_NUMBER"d.lib")
//	#pragma comment(lib,"opencv_gpu"CV_VERSION_NUMBER"d.lib")
//	#pragma comment(lib,"opencv_contrib"CV_VERSION_NUMBER"d.lib")
//	#pragma comment(lib,"opencv_objdetect"CV_VERSION_NUMBER"d.lib")
//#else
//	//Releaseモードの場合
//	#pragma comment(lib,"opencv_core"CV_VERSION_NUMBER".lib")
//	#pragma comment(lib,"opencv_imgproc"CV_VERSION_NUMBER".lib")
//	#pragma comment(lib,"opencv_highgui"CV_VERSION_NUMBER".lib")
//	#pragma comment(lib,"opencv_gpu"CV_VERSION_NUMBER".lib")
//	#pragma comment(lib,"opencv_contrib"CV_VERSION_NUMBER".lib")
//	#pragma comment(lib,"opencv_objdetect"CV_VERSION_NUMBER".lib")
//#endif

