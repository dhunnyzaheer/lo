# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# compile CXX with /usr/bin/c++
CXX_DEFINES = -DDISABLE_PCAP -DDISABLE_PNG -DGFLAGS_IS_A_DLL=0 -DGLOG_CUSTOM_PREFIX_SUPPORT -DQT_CORE_LIB -DQT_GUI_LIB -DQT_NO_DEBUG -DQT_WIDGETS_LIB -Dqh_QHpointer -DvtkRenderingContext2D_AUTOINIT="1(vtkRenderingContextOpenGL2)" -DvtkRenderingCore_AUTOINIT="3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL2)"

CXX_INCLUDES = -I/home/zaheer/aru-core/src/lo -I/home/zaheer/aru-core/src/lo/include -I/home/zaheer/aru-core/src/lo/build/datatypes -I/home/zaheer/aru-core/src/lo/build -isystem /home/zaheer/zaheer/software/vtk/build/Charts/Core -isystem /home/zaheer/zaheer/software/vtk/Charts/Core -isystem /home/zaheer/zaheer/software/vtk/build/Common/Color -isystem /home/zaheer/zaheer/software/vtk/Common/Color -isystem /home/zaheer/zaheer/software/vtk/build/Common/Core -isystem /home/zaheer/zaheer/software/vtk/Common/Core -isystem /home/zaheer/zaheer/software/vtk/build/Utilities/KWIML -isystem /home/zaheer/zaheer/software/vtk/Utilities/KWIML -isystem /home/zaheer/zaheer/software/vtk/build/Utilities/KWSys -isystem /home/zaheer/zaheer/software/vtk/Utilities/KWSys -isystem /home/zaheer/zaheer/software/vtk/build/ThirdParty/utf8 -isystem /home/zaheer/zaheer/software/vtk/ThirdParty/utf8 -isystem /home/zaheer/zaheer/software/vtk/build/Common/DataModel -isystem /home/zaheer/zaheer/software/vtk/Common/DataModel -isystem /home/zaheer/zaheer/software/vtk/build/Common/Math -isystem /home/zaheer/zaheer/software/vtk/Common/Math -isystem /home/zaheer/zaheer/software/vtk/build/Common/Misc -isystem /home/zaheer/zaheer/software/vtk/Common/Misc -isystem /home/zaheer/zaheer/software/vtk/build/Common/System -isystem /home/zaheer/zaheer/software/vtk/Common/System -isystem /home/zaheer/zaheer/software/vtk/build/Common/Transforms -isystem /home/zaheer/zaheer/software/vtk/Common/Transforms -isystem /home/zaheer/zaheer/software/vtk/build/Common/ExecutionModel -isystem /home/zaheer/zaheer/software/vtk/Common/ExecutionModel -isystem /home/zaheer/zaheer/software/vtk/build/Filters/General -isystem /home/zaheer/zaheer/software/vtk/Filters/General -isystem /home/zaheer/zaheer/software/vtk/build/Common/ComputationalGeometry -isystem /home/zaheer/zaheer/software/vtk/Common/ComputationalGeometry -isystem /home/zaheer/zaheer/software/vtk/build/Filters/Core -isystem /home/zaheer/zaheer/software/vtk/Filters/Core -isystem /home/zaheer/zaheer/software/vtk/build/Infovis/Core -isystem /home/zaheer/zaheer/software/vtk/Infovis/Core -isystem /home/zaheer/zaheer/software/vtk/build/Filters/Extraction -isystem /home/zaheer/zaheer/software/vtk/Filters/Extraction -isystem /home/zaheer/zaheer/software/vtk/build/Filters/Statistics -isystem /home/zaheer/zaheer/software/vtk/Filters/Statistics -isystem /home/zaheer/zaheer/software/vtk/build/Imaging/Fourier -isystem /home/zaheer/zaheer/software/vtk/Imaging/Fourier -isystem /home/zaheer/zaheer/software/vtk/build/Imaging/Core -isystem /home/zaheer/zaheer/software/vtk/Imaging/Core -isystem /home/zaheer/zaheer/software/vtk/build/ThirdParty/eigen -isystem /home/zaheer/zaheer/software/vtk/ThirdParty/eigen -isystem /usr/local/include/eigen3 -isystem /home/zaheer/zaheer/software/vtk/build/Rendering/Context2D -isystem /home/zaheer/zaheer/software/vtk/Rendering/Context2D -isystem /home/zaheer/zaheer/software/vtk/build/Rendering/Core -isystem /home/zaheer/zaheer/software/vtk/Rendering/Core -isystem /home/zaheer/zaheer/software/vtk/build/Filters/Geometry -isystem /home/zaheer/zaheer/software/vtk/Filters/Geometry -isystem /home/zaheer/zaheer/software/vtk/build/Filters/Sources -isystem /home/zaheer/zaheer/software/vtk/Filters/Sources -isystem /home/zaheer/zaheer/software/vtk/build/Rendering/FreeType -isystem /home/zaheer/zaheer/software/vtk/Rendering/FreeType -isystem /home/zaheer/zaheer/software/vtk/build/ThirdParty/freetype -isystem /home/zaheer/zaheer/software/vtk/ThirdParty/freetype -isystem /usr/include/freetype2 -isystem /home/zaheer/zaheer/software/vtk/build/ThirdParty/zlib -isystem /home/zaheer/zaheer/software/vtk/ThirdParty/zlib -isystem /home/zaheer/zaheer/software/vtk/build/Filters/Modeling -isystem /home/zaheer/zaheer/software/vtk/Filters/Modeling -isystem /home/zaheer/zaheer/software/vtk/build/Imaging/Sources -isystem /home/zaheer/zaheer/software/vtk/Imaging/Sources -isystem /home/zaheer/zaheer/software/vtk/build/Interaction/Image -isystem /home/zaheer/zaheer/software/vtk/Interaction/Image -isystem /home/zaheer/zaheer/software/vtk/build/Imaging/Color -isystem /home/zaheer/zaheer/software/vtk/Imaging/Color -isystem /home/zaheer/zaheer/software/vtk/build/Interaction/Style -isystem /home/zaheer/zaheer/software/vtk/Interaction/Style -isystem /home/zaheer/zaheer/software/vtk/build/Interaction/Widgets -isystem /home/zaheer/zaheer/software/vtk/Interaction/Widgets -isystem /home/zaheer/zaheer/software/vtk/build/Filters/Hybrid -isystem /home/zaheer/zaheer/software/vtk/Filters/Hybrid -isystem /home/zaheer/zaheer/software/vtk/build/Imaging/General -isystem /home/zaheer/zaheer/software/vtk/Imaging/General -isystem /home/zaheer/zaheer/software/vtk/build/Imaging/Hybrid -isystem /home/zaheer/zaheer/software/vtk/Imaging/Hybrid -isystem /home/zaheer/zaheer/software/vtk/build/IO/Image -isystem /home/zaheer/zaheer/software/vtk/IO/Image -isystem /home/zaheer/zaheer/software/vtk/build/Utilities/DICOMParser -isystem /home/zaheer/zaheer/software/vtk/Utilities/DICOMParser -isystem /home/zaheer/zaheer/software/vtk/build/Utilities/MetaIO/vtkmetaio -isystem /home/zaheer/zaheer/software/vtk/build/Utilities/MetaIO -isystem /home/zaheer/zaheer/software/vtk/Utilities/MetaIO -isystem /home/zaheer/zaheer/software/vtk/build/ThirdParty/jpeg -isystem /home/zaheer/zaheer/software/vtk/ThirdParty/jpeg -isystem /home/zaheer/zaheer/software/vtk/build/ThirdParty/png -isystem /home/zaheer/zaheer/software/vtk/ThirdParty/png -isystem /home/zaheer/zaheer/software/vtk/build/ThirdParty/tiff -isystem /home/zaheer/zaheer/software/vtk/ThirdParty/tiff -isystem /home/zaheer/zaheer/software/vtk/build/Rendering/Annotation -isystem /home/zaheer/zaheer/software/vtk/Rendering/Annotation -isystem /home/zaheer/zaheer/software/vtk/build/Rendering/Volume -isystem /home/zaheer/zaheer/software/vtk/Rendering/Volume -isystem /home/zaheer/zaheer/software/vtk/build/IO/XML -isystem /home/zaheer/zaheer/software/vtk/IO/XML -isystem /home/zaheer/zaheer/software/vtk/build/IO/Core -isystem /home/zaheer/zaheer/software/vtk/IO/Core -isystem /home/zaheer/zaheer/software/vtk/build/ThirdParty/doubleconversion -isystem /home/zaheer/zaheer/software/vtk/ThirdParty/doubleconversion -isystem /usr/include/double-conversion -isystem /home/zaheer/zaheer/software/vtk/build/ThirdParty/lz4 -isystem /home/zaheer/zaheer/software/vtk/ThirdParty/lz4 -isystem /home/zaheer/zaheer/software/vtk/build/ThirdParty/lzma -isystem /home/zaheer/zaheer/software/vtk/ThirdParty/lzma -isystem /home/zaheer/zaheer/software/vtk/build/IO/XMLParser -isystem /home/zaheer/zaheer/software/vtk/IO/XMLParser -isystem /home/zaheer/zaheer/software/vtk/build/ThirdParty/expat -isystem /home/zaheer/zaheer/software/vtk/ThirdParty/expat -isystem /home/zaheer/zaheer/software/vtk/build/IO/Geometry -isystem /home/zaheer/zaheer/software/vtk/IO/Geometry -isystem /home/zaheer/zaheer/software/vtk/build/IO/Legacy -isystem /home/zaheer/zaheer/software/vtk/IO/Legacy -isystem /home/zaheer/zaheer/software/vtk/build/IO/PLY -isystem /home/zaheer/zaheer/software/vtk/IO/PLY -isystem /home/zaheer/zaheer/software/vtk/build/Rendering/LOD -isystem /home/zaheer/zaheer/software/vtk/Rendering/LOD -isystem /home/zaheer/zaheer/software/vtk/build/Views/Core -isystem /home/zaheer/zaheer/software/vtk/Views/Core -isystem /home/zaheer/zaheer/software/vtk/build/Views/Context2D -isystem /home/zaheer/zaheer/software/vtk/Views/Context2D -isystem /home/zaheer/zaheer/software/vtk/build/Rendering/OpenGL2 -isystem /home/zaheer/zaheer/software/vtk/Rendering/OpenGL2 -isystem /home/zaheer/zaheer/software/vtk/build/ThirdParty/glew -isystem /home/zaheer/zaheer/software/vtk/ThirdParty/glew -isystem /home/zaheer/zaheer/software/vtk/build/Rendering/ContextOpenGL2 -isystem /home/zaheer/zaheer/software/vtk/Rendering/ContextOpenGL2 -isystem /home/zaheer/zaheer/software/vtk/build/GUISupport/Qt -isystem /home/zaheer/zaheer/software/vtk/GUISupport/Qt -isystem /usr/local/include/pcl-1.12 -isystem /usr/include/ni -isystem /usr/include/openni2 -isystem /usr/include/x86_64-linux-gnu/qt5 -isystem /usr/include/x86_64-linux-gnu/qt5/QtWidgets -isystem /usr/include/x86_64-linux-gnu/qt5/QtGui -isystem /usr/include/x86_64-linux-gnu/qt5/QtCore -isystem /usr/lib/x86_64-linux-gnu/qt5/mkspecs/linux-g++

CXX_FLAGS = -std=c++14  -O3 -Wall -g -msse4.2 -mfpmath=sse -march=native -mavx2 -fPIC
