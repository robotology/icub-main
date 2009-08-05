# Microsoft Developer Studio Project File - Name="roboop" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=roboop - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "roboop.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "roboop.mak" CFG="roboop - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "roboop - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "roboop - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "roboop - Win32 Release"

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

!ELSEIF  "$(CFG)" == "roboop - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "roboop___Win32_Debug"
# PROP BASE Intermediate_Dir "roboop___Win32_Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "roboop___Win32_Debug"
# PROP Intermediate_Dir "roboop___Win32_Debug"
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

# Name "roboop - Win32 Release"
# Name "roboop - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\source\clik.cpp
# End Source File
# Begin Source File

SOURCE=.\source\comp_dq.cpp
# End Source File
# Begin Source File

SOURCE=.\source\comp_dqp.cpp
# End Source File
# Begin Source File

SOURCE=.\source\config.cpp
# End Source File
# Begin Source File

SOURCE=.\source\control_select.cpp
# End Source File
# Begin Source File

SOURCE=.\source\control_select.h
# End Source File
# Begin Source File

SOURCE=.\source\controller.cpp
# End Source File
# Begin Source File

SOURCE=.\source\controller.h
# End Source File
# Begin Source File

SOURCE=.\source\delta_t.cpp
# End Source File
# Begin Source File

SOURCE=.\source\dynamics.cpp
# End Source File
# Begin Source File

SOURCE=.\source\dynamics_sim.cpp
# End Source File
# Begin Source File

SOURCE=.\source\gnugraph.cpp
# End Source File
# Begin Source File

SOURCE=.\source\homogen.cpp
# End Source File
# Begin Source File

SOURCE=.\source\invkine.cpp
# End Source File
# Begin Source File

SOURCE=.\source\kinemat.cpp
# End Source File
# Begin Source File

SOURCE=.\source\quaternion.cpp
# End Source File
# Begin Source File

SOURCE=.\source\robot.cpp
# End Source File
# Begin Source File

SOURCE=.\source\sensitiv.cpp
# End Source File
# Begin Source File

SOURCE=.\source\stewart.cpp
# End Source File
# Begin Source File

SOURCE=.\source\trajectory.cpp
# End Source File
# Begin Source File

SOURCE=.\source\utils.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=.\source\config.h
# End Source File
# Begin Source File

SOURCE=.\source\gnugraph.h
# End Source File
# Begin Source File

SOURCE=.\source\robot.h
# End Source File
# End Group
# End Target
# End Project
