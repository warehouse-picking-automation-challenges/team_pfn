#pragma once

#pragma region QT5
#define	QT_HEAD	"Qt5"
#ifdef _DEBUG
#define QT_EXT_STR "d.lib"
#else
#define QT_EXT_STR ".lib"
#endif

#pragma comment(lib, "qtmain"				QT_EXT_STR)
#pragma comment(lib, QT_HEAD "Core"			QT_EXT_STR)
#pragma comment(lib, QT_HEAD "OpenGL"		QT_EXT_STR)
#pragma comment(lib, QT_HEAD "OpenGLExtensions"			QT_EXT_STR)
#pragma comment(lib, QT_HEAD "Gui"			QT_EXT_STR)
#pragma comment(lib, QT_HEAD "Widgets"		QT_EXT_STR)
#pragma comment(lib, QT_HEAD "Widgets"		QT_EXT_STR)
#pragma endregion