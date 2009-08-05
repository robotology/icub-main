# Microsoft Developer Studio Project File - Name="newmat" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=newmat - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "newmat.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "newmat.mak" CFG="newmat - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "newmat - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "newmat - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "newmat - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /MD /W3 /GX /O2 /I "newmat" /I "source" /D "NDEBUG" /D "WIN32" /D "_MBCS" /D "_LIB" /D "_STANDARD_" /YX /FD /c
# ADD BASE RSC /l 0xc0c /d "NDEBUG"
# ADD RSC /l 0xc0c /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo

!ELSEIF  "$(CFG)" == "newmat - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GX /ZI /Od /I "newmat" /I "source" /D "_DEBUG" /D "WIN32" /D "_MBCS" /D "_LIB" /D "_STANDARD_" /YX /FD /GZ /c
# ADD BASE RSC /l 0xc0c /d "_DEBUG"
# ADD RSC /l 0xc0c /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo

!ENDIF 

# Begin Target

# Name "newmat - Win32 Release"
# Name "newmat - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\newmat\bandmat.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\cholesky.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\evalue.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\fft.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\hholder.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\jacobi.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\myexcept.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\newfft.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\newmat1.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\newmat2.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\newmat3.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\newmat4.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\newmat5.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\newmat6.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\newmat7.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\newmat8.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\newmat9.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\newmatex.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\newmatnl.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\newmatrm.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\nm_misc.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\solution.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\sort.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\submat.cpp
# End Source File
# Begin Source File

SOURCE=.\newmat\svd.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# End Group
# End Target
# End Project
