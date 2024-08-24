workspace "FT-TDMA-Sim"
    configurations { "Debug", "Release" }
    platforms { "x64" }
    architecture "x64"

    startproject "FT-TDMA-Sim"


OutputDir = "%{cfg.buildcfg}-%{cfg.architecture}"

project "FT-TDMA-Sim"
    location "FT-TDMA-Sim"
    kind "ConsoleApp"
    language "C++"
    cppdialect "C++20"

    pchheader "PCH.h"
    pchsource "%{prj.name}/source/PCH.cpp"

    targetdir ("bin/" .. OutputDir .. "/%{prj.name}")
    objdir ("bin-itm/" .. OutputDir .. "/%{prj.name}")

    files {
        "%{prj.name}/source/**.h",
        "%{prj.name}/source/**.cpp",
        "%{prj.name}/interface/**.h",
        "%{prj.name}/interface/**.cpp"
    }

    includedirs { 
        "%{prj.name}/source",
        "%{prj.name}/interface"
     }

    systemversion "latest"

    filter "configurations:Debug"
        defines "_DEBUG"
        runtime "Debug"
        symbols "on"
    
    filter "configurations:Release"
        defines "_RELEASE"
        runtime "Release"
        optimize "on"

    filter "system:windows"
        defines { "PLATFORM_WINDOWS" }
    
    filter "system:linux"
        defines { "PLATFORM_LINUX" }
